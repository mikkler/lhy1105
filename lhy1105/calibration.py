"""One joint at a time leader/follower range and direction check."""

from __future__ import annotations

import csv
import sys
import time
from contextlib import closing
from datetime import datetime
from pathlib import Path

import numpy as np

from .config import Config
from .feetech import FeetechBus
from .mapping import SafetyStop, rate_limit, tick_delta


def selection(leader_id: int) -> tuple[str, int]:
    if not 1 <= leader_id <= 14:
        raise ValueError("leader ID must be 1..14")
    return ("left", leader_id - 1) if leader_id <= 7 else ("right", leader_id - 8)


def nominal_arm_limits() -> np.ndarray:
    """Use the model's actual joint ranges, before I2RT's ±0.15 rad buffer."""
    from i2rt.robots.get_robot import _load_joint_limits_from_xml
    from i2rt.robots.utils import ArmType

    limits = np.asarray(_load_joint_limits_from_xml(ArmType.YAM.get_xml_path())[:6], dtype=float)
    if limits.shape != (6, 2) or not np.all(np.isfinite(limits)):
        raise SafetyStop("nominal follower joint limits unavailable")
    return limits


def selected_target(start_q: np.ndarray, joint: int, delta_ticks: int,
                    sign: int, gripper_span_ticks: float) -> tuple[np.ndarray, float, float]:
    """Return a new target with only the selected follower axis changed."""
    is_gripper = joint == 6
    axis = 0 if is_gripper else joint
    expected = 1 if is_gripper else 6
    if start_q.shape != (expected,):
        raise ValueError(f"expected {expected} follower positions")
    delta_rad = delta_ticks * 2 * np.pi / 4096
    delta = delta_rad * sign if not is_gripper else (
        delta_ticks * sign / gripper_span_ticks)
    target = start_q.copy()
    target[axis] += delta
    return target, float(delta_rad), float(delta)


def fresh_state(robot, dofs: int, max_age: float) -> np.ndarray:
    q, stamps = robot.get_joint_pos_and_can_rx_ns()
    q = np.asarray(q, dtype=float).copy()
    stamps = np.asarray(stamps)
    if q.shape != (dofs,) or stamps.shape != (dofs,) or not np.all(np.isfinite(q)):
        raise SafetyStop("invalid follower feedback")
    age = (time.monotonic_ns() - stamps) / 1e9
    if np.any(stamps <= 0) or np.any(age < -0.01) or np.any(age > max_age):
        raise SafetyStop("follower CAN feedback stale")
    return q


def run(bus: FeetechBus, cfg: Config, leader_id: int, *, sign: int | None = None,
        duration: float | None = None, output: str | None = None) -> Path:
    """Follow only the corresponding joint and save the paired samples as CSV."""
    from i2rt.robots.get_robot import get_yam_robot
    from i2rt.robots.utils import ArmType, GripperType
    from .cli import key_now, keyboard

    side, joint = selection(leader_id)
    if sign is None:
        sign = int(cfg.mappings[side].gripper_sign if joint == 6 else cfg.mappings[side].joint_signs[joint])
    if sign not in (-1, 1) or (duration is not None and duration <= 0):
        raise ValueError("sign must be ±1; duration must be positive")
    if not sys.stdin.isatty():
        raise SafetyStop("calibrate requires an interactive terminal")
    is_gripper = joint == 6
    axis = 0 if is_gripper else joint
    channel = cfg.channels[side]
    label = f"{side} {'gripper' if is_gripper else f'joint {joint + 1}'}"
    print(f"Leader ID {leader_id} -> {label} on {channel}; sign={sign}")
    print("Support the follower before connecting. Its controller will release torque on exit.")
    if is_gripper:
        print("The selected gripper may move during I2RT's automatic limit calibration.")
    if input("Type CONNECT to connect the selected follower: ").strip() != "CONNECT":
        raise SafetyStop("cancelled before connecting")

    # Arm mode excludes the gripper, so its automatic limit calibration cannot
    # move it. Gripper mode connects only motor 7 on the selected CAN bus.
    options = ({"arm_type": ArmType.NO_ARM, "gripper_type": GripperType.LINEAR_4310}
               if is_gripper else {"gripper_type": GripperType.NO_GRIPPER})
    robot = get_yam_robot(channel=channel, zero_gravity_mode=False,
                          enable_auto_recovery=False, **options)
    with closing(robot):
        dofs = 1 if is_gripper else 6
        if robot.num_dofs() != dofs:
            raise SafetyStop(f"expected {dofs} follower DOFs, got {robot.num_dofs()}")
        interface = getattr(robot.motor_chain, "motor_interface", None)
        if interface is None or not hasattr(interface, "trace_timestamps_enabled"):
            raise SafetyStop("follower CAN receive timestamps unavailable")
        interface.trace_timestamps_enabled = True
        deadline = time.monotonic() + 2.
        while True:
            try:
                start_q = fresh_state(robot, dofs, cfg.max_feedback_age_s)
                break
            except SafetyStop as exc:
                if str(exc) != "follower CAN feedback stale" or time.monotonic() >= deadline:
                    raise
                time.sleep(.02)
        start_tick = int.from_bytes(bus.read(leader_id, 56, 2), "little")
        limits = None if is_gripper else np.asarray(robot.get_robot_info()["joint_limits"], dtype=float)
        motion_limits = None if is_gripper else nominal_arm_limits()
        if not is_gripper:
            if limits.shape != (6, 2) or not np.all(np.isfinite(limits)):
                raise SafetyStop("follower joint limits unavailable")
            if np.any(start_q < limits[:, 0]) or np.any(start_q > limits[:, 1]):
                raise SafetyStop("follower starts outside configured limits; holding other joints could move them")
            print(f"Nominal follower joint range: {motion_limits[joint, 0]:.4f}.."
                  f"{motion_limits[joint, 1]:.4f} rad")
        else:
            print("Configured follower gripper range: 0..1 normalized")
        print(f"Start: leader tick={start_tick}, follower={start_q[axis]:.4f}"
              + (" normalized" if is_gripper else " rad"))
        print("After ARM, move the selected leader joint slowly; press x or Ctrl+C to stop.")
        if input("Type ARM to begin one-joint following: ").strip() != "ARM":
            raise SafetyStop("cancelled before following")
        # Align at ARM, rather than at CONNECT: the operator may reposition the
        # leader while reviewing the start state.
        start_q = fresh_state(robot, dofs, cfg.max_feedback_age_s)
        start_tick = int.from_bytes(bus.read(leader_id, 56, 2), "little")
        if not is_gripper and (np.any(start_q < limits[:, 0]) or np.any(start_q > limits[:, 1])):
            raise SafetyStop("follower starts outside configured limits; holding other joints could move them")
        low, high = ((0., 1.) if is_gripper else tuple(motion_limits[joint]))
        print(f"Follower target will stay within {low:.3f}..{high:.3f}; "
              "leader travel is unrestricted. Press x or q to stop.")

        path = (Path(output) if output else Path("calibration_runs") /
                f"{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}_id{leader_id}.csv")
        path.parent.mkdir(parents=True, exist_ok=True)
        minimum = [start_tick, float(start_q[axis])]
        maximum = minimum.copy()
        delta_min = delta_max = 0.
        previous = start_q.copy()
        previous_tick = start_tick
        cumulative_ticks = 0
        last = time.monotonic()
        started = last
        count = 0
        with path.open("w", newline="") as file, keyboard():
            writer = csv.writer(file)
            writer.writerow(["elapsed_s", "leader_id", "follower_side", "follower_joint",
                             "leader_tick", "leader_delta_ticks", "leader_delta_rad", "raw_target",
                             "limited_target", "command", "measured", "limit_hit", "other_joint_max_drift"])
            try:
                while duration is None or time.monotonic() - started < duration:
                    cycle = time.monotonic()
                    if key_now().lower() in ("x", "q", "\x03"):
                        break
                    tick = int.from_bytes(bus.read(leader_id, 56, 2), "little")
                    measured = fresh_state(robot, dofs, cfg.max_feedback_age_s)
                    now = time.monotonic()
                    dt = now - last
                    if dt <= 0 or dt > .25 or now - cycle > .2:
                        raise SafetyStop("calibration loop stalled")
                    cumulative_ticks += tick_delta(tick, previous_tick)
                    desired, delta_rad, _ = selected_target(
                        start_q, joint, cumulative_ticks, sign, cfg.mappings[side].gripper_span_ticks)
                    raw_target = float(desired[axis])
                    desired[axis] = np.clip(raw_target, low, high)
                    target = float(desired[axis])
                    limit_hit = raw_target != target
                    other_drift = 0.
                    if not is_gripper:
                        others = [i for i in range(6) if i != joint]
                        other_drift = float(np.max(np.abs(measured[others] - start_q[others])))
                        if other_drift > .08:
                            raise SafetyStop(f"unselected follower joint moved {other_drift:.3f} rad")
                    command = rate_limit(desired, previous, cfg.max_velocity[:dofs] if not is_gripper
                                         else cfg.max_velocity[6:7], dt)
                    robot.command_joint_pos(command)
                    writer.writerow([f"{now-started:.3f}", leader_id, side, joint + 1, tick,
                                     cumulative_ticks, f"{delta_rad:.5f}", f"{raw_target:.5f}",
                                     f"{target:.5f}", f"{command[axis]:.5f}", f"{measured[axis]:.5f}",
                                     int(limit_hit), f"{other_drift:.5f}"])
                    count += 1
                    minimum[0] = min(minimum[0], tick)
                    maximum[0] = max(maximum[0], tick)
                    delta_min = min(delta_min, delta_rad)
                    delta_max = max(delta_max, delta_rad)
                    minimum[1] = min(minimum[1], measured[axis])
                    maximum[1] = max(maximum[1], measured[axis])
                    unit = "norm" if is_gripper else "rad"
                    print(f"\rID {leader_id}: leader={tick:4d} tick, Δ={delta_rad:+.3f} rad | "
                          f"follower={measured[axis]:+.3f} {unit}, command={command[axis]:+.3f}, "
                          f"target={target:+.3f}{' LIMIT' if limit_hit else ''}\x1b[K",
                          end="", flush=True)
                    previous, previous_tick, last = command, tick, now
                    time.sleep(max(0., 1 / cfg.hz - (time.monotonic() - cycle)))
            except KeyboardInterrupt:
                pass
            finally:
                file.flush()
                print(f"\nRecorded {count} samples to {path}")
                print(f"Leader tick range: {minimum[0]}..{maximum[0]}; "
                      f"leader delta range: {delta_min:.4f}..{delta_max:.4f} rad; "
                      f"follower measured range: {minimum[1]:.4f}..{maximum[1]:.4f}")
                print("Controller closing; keep the follower supported as torque is released.")
        return path
