"""Interactive multi-joint leader/follower mapping check."""

from __future__ import annotations

import csv
import sys
import time
from contextlib import ExitStack
from datetime import datetime
from pathlib import Path

import numpy as np

from .calibration import fresh_state, nominal_arm_limits, selection
from .config import Config
from .feetech import FeetechBus
from .mapping import SafetyStop, rate_limit, tick_delta


def validate_ids(ids: list[int], reverse: list[int]) -> tuple[int, ...]:
    if not ids or len(set(ids)) != len(ids) or any(not 1 <= ident <= 14 for ident in ids):
        raise ValueError("--ids needs unique servo IDs from 1..14")
    if len(set(reverse)) != len(reverse) or not set(reverse).issubset(ids):
        raise ValueError("--reverse IDs must be unique and included in --ids")
    return tuple(sorted(ids))


def follower_axis(leader_id: int, dofs: int) -> int:
    _, joint = selection(leader_id)
    return 0 if joint == 6 and dofs == 1 else joint


def connect_side(cfg: Config, side: str, ids: tuple[int, ...], stack: ExitStack,
                 lock_all: bool = False):
    from i2rt.robots.get_robot import get_yam_robot
    from i2rt.robots.utils import ArmType, GripperType

    joints = [selection(ident)[1] for ident in ids]
    has_arm = lock_all or any(joint < 6 for joint in joints)
    has_gripper = lock_all or 6 in joints
    if has_arm and has_gripper:
        options = {"gripper_type": GripperType.LINEAR_4310}
        dofs = 7
    elif has_arm:
        options = {"gripper_type": GripperType.NO_GRIPPER}
        dofs = 6
    else:
        options = {"arm_type": ArmType.NO_ARM, "gripper_type": GripperType.LINEAR_4310}
        dofs = 1
    robot = get_yam_robot(channel=cfg.channels[side], zero_gravity_mode=False,
                          enable_auto_recovery=False, **options)
    stack.callback(robot.close)
    if robot.num_dofs() != dofs:
        raise SafetyStop(f"{side}: expected {dofs} follower DOFs")
    interface = getattr(robot.motor_chain, "motor_interface", None)
    if interface is None or not hasattr(interface, "trace_timestamps_enabled"):
        raise SafetyStop(f"{side}: CAN receive timestamps unavailable")
    interface.trace_timestamps_enabled = True
    deadline = time.monotonic() + 2.
    while True:
        try:
            fresh_state(robot, dofs, cfg.max_feedback_age_s)
            break
        except SafetyStop as exc:
            if str(exc) != "follower CAN feedback stale" or time.monotonic() >= deadline:
                raise
            time.sleep(.02)
    limits = None
    if has_arm:
        limits = np.asarray(robot.get_robot_info()["joint_limits"], dtype=float)
        if limits.shape != (6, 2) or not np.all(np.isfinite(limits)):
            raise SafetyStop(f"{side}: follower joint limits unavailable")
    return robot, dofs, limits


def zero_target(dofs: int) -> np.ndarray:
    if dofs == 7:
        return np.array([0., 0., 0., 0., 0., 0., .99])
    if dofs == 6:
        return np.zeros(6)
    if dofs == 1:
        return np.array([.99])
    raise ValueError(f"unsupported follower DOFs: {dofs}")


def move_to_zero(robots: dict, dofs: dict, limits: dict, cfg: Config) -> None:
    """Slowly return connected followers to configured arm zero and open gripper."""
    starts = {side: fresh_state(robot, dofs[side], cfg.max_feedback_age_s)
              for side, robot in robots.items()}
    targets = {side: zero_target(dofs[side]) for side in robots}
    velocities = {side: (cfg.max_velocity if dofs[side] == 7 else
                         cfg.max_velocity[:6] if dofs[side] == 6 else cfg.max_velocity[6:7])
                  for side in robots}
    for side in robots:
        if limits[side] is not None:
            arm_target = targets[side][:6]
            if np.any(arm_target < limits[side][:, 0]) or np.any(arm_target > limits[side][:, 1]):
                raise SafetyStop(f"{side}: configured zero is outside follower joint limits")
    needed = max(float(np.max(np.abs(targets[side] - starts[side]) / velocities[side]))
                 for side in robots)
    duration = max(3., needed)
    steps = max(100, int(duration * cfg.hz))
    print(f"Returning connected followers to arm 0 rad / gripper 0.99 over {duration:.1f}s.")
    for step in range(1, steps + 1):
        cycle = time.monotonic()
        alpha = step / steps
        for side, robot in robots.items():
            fresh_state(robot, dofs[side], cfg.max_feedback_age_s)
            robot.command_joint_pos(starts[side] * (1 - alpha) + targets[side] * alpha)
        time.sleep(max(0., duration / steps - (time.monotonic() - cycle)))
    for side, robot in robots.items():
        actual = fresh_state(robot, dofs[side], cfg.max_feedback_age_s)
        if np.any(np.abs(actual - targets[side]) > .15):
            raise SafetyStop(f"{side}: follower did not reach configured zero; keep supporting it")
    print("Connected followers reached configured zero within 0.15 tolerance.")


def run(bus: FeetechBus, cfg: Config, ids: list[int], *, reverse: list[int] | None = None,
        duration: float | None = None, output: str | None = None,
        lock_others: bool = False, show_cycle_timing: bool = False,
        limit_velocity: bool = True) -> Path:
    """Follow selected matching joints on one or both follower arms."""
    from .cli import key_now, keyboard

    reverse = reverse or []
    chosen = validate_ids(ids, reverse)
    if duration is not None and duration <= 0:
        raise ValueError("duration must be positive")
    if not sys.stdin.isatty():
        raise SafetyStop("multi requires an interactive terminal")
    grouped = {side: tuple(i for i in chosen if selection(i)[0] == side)
               for side in ("left", "right")}
    active = {side: group for side, group in grouped.items() if group}
    connected = grouped if lock_others else active
    signs = {}
    for ident in chosen:
        side, joint = selection(ident)
        normal = cfg.mappings[side].gripper_sign if joint == 6 else cfg.mappings[side].joint_signs[joint]
        signs[ident] = int(-normal if ident in reverse else normal)
        print(f"ID {ident:2d} -> {side} {'gripper' if joint == 6 else f'joint {joint + 1}'}, sign={signs[ident]}")
    print("Support all connected followers; torque releases after returning to zero.")
    if lock_others:
        print("LOCK OTHERS: both full follower arms will hold unselected joints at their ARM pose.")
        print("I2RT may move either gripper during automatic limit calibration before ARM.")
    elif 7 in chosen or 14 in chosen:
        print("A selected gripper may move during I2RT automatic limit calibration.")
    if input("Type CONNECT to connect the selected follower side(s): ").strip() != "CONNECT":
        raise SafetyStop("cancelled before connecting")

    with ExitStack() as stack:
        robots = {}
        dofs = {}
        limits = {}
        for side, group in connected.items():
            robots[side], dofs[side], limits[side] = connect_side(
                cfg, side, group, stack, lock_all=lock_others)
        print("Move the leaders to a comfortable pose, then type ARM. Press x, q, or Ctrl+C to stop.")
        if input("Type ARM to begin following: ").strip() != "ARM":
            raise SafetyStop("cancelled before following")

        starts = {side: fresh_state(robots[side], dofs[side], cfg.max_feedback_age_s) for side in connected}
        motion_limits = {side: nominal_arm_limits() if limits[side] is not None else None
                         for side in connected}
        for ident in chosen:
            side, joint = selection(ident)
            low, high = ((0., 1.) if joint == 6 else tuple(motion_limits[side][joint]))
            print(f"ID {ident}: follower nominal target range {low:.3f}..{high:.3f}"
                  + (" normalized" if joint == 6 else " rad"))
        for side in connected:
            if limits[side] is not None:
                arm_start = starts[side][:6]
                if np.any(arm_start < limits[side][:, 0]) or np.any(arm_start > limits[side][:, 1]):
                    raise SafetyStop(f"{side}: follower starts outside configured limits")
        first_ticks = bus.sample(chosen).positions
        last_ticks = first_ticks.copy()
        cumulative = {ident: 0 for ident in chosen}
        previous = {side: q.copy() for side, q in starts.items()}
        path = (Path(output) if output else Path("calibration_runs") /
                f"{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}_multi.csv")
        path.parent.mkdir(parents=True, exist_ok=True)
        last = started = time.monotonic()
        last_display = 0.
        last_cycle_start = None
        count = 0
        graceful = False
        with path.open("w", newline="") as file, keyboard():
            writer = csv.writer(file)
            writer.writerow(["elapsed_s", "leader_id", "follower_side", "follower_joint",
                             "leader_tick", "leader_delta_ticks", "leader_delta_rad", "raw_target",
                             "limited_target", "command", "measured", "limit_hit", "other_joint_max_drift"])
            try:
                while duration is None or time.monotonic() - started < duration:
                    cycle = time.monotonic()
                    cycle_ns = time.monotonic_ns()
                    period_ms = ((cycle - last_cycle_start) * 1000
                                 if last_cycle_start is not None else None)
                    last_cycle_start = cycle
                    if key_now().lower() in ("x", "q", "\x03"):
                        break
                    ticks = bus.sample(chosen).positions
                    sample_ns = time.monotonic_ns()
                    measured = {side: fresh_state(robots[side], dofs[side], cfg.max_feedback_age_s)
                                for side in connected}
                    feedback_ns = time.monotonic_ns()
                    now = time.monotonic()
                    dt = now - last
                    if dt <= 0 or dt > .5 or now - cycle > .4:
                        raise SafetyStop("multi-joint control loop stalled")
                    desired = {side: q.copy() for side, q in starts.items()}
                    display = []
                    rows = []
                    for ident in chosen:
                        side, joint = selection(ident)
                        axis = follower_axis(ident, dofs[side])
                        cumulative[ident] += tick_delta(ticks[ident], last_ticks[ident])
                        delta_rad = cumulative[ident] * 2 * np.pi / 4096
                        delta = (cumulative[ident] * signs[ident] / cfg.mappings[side].gripper_span_ticks
                                 if joint == 6 else delta_rad * signs[ident])
                        raw = float(starts[side][axis] + delta)
                        low, high = ((0., 1.) if joint == 6 else tuple(motion_limits[side][joint]))
                        bounded = float(np.clip(raw, low, high))
                        desired[side][axis] = bounded
                        rows.append((ident, side, joint, axis, ticks[ident], delta_rad, raw,
                                     bounded, raw != bounded))
                    commands = {}
                    drifts = {}
                    for side, group in connected.items():
                        selected_arm_axes = {selection(ident)[1] for ident in group
                                             if selection(ident)[1] < 6}
                        other = [axis for axis in range(6) if axis not in selected_arm_axes] if dofs[side] >= 6 else []
                        if dofs[side] == 7 and not any(selection(i)[1] == 6 for i in group):
                            other.append(6)
                        drifts[side] = float(np.max(np.abs(measured[side][other] - starts[side][other]))) if other else 0.
                        if drifts[side] > .08:
                            raise SafetyStop(f"{side}: unselected follower joint moved {drifts[side]:.3f} rad")
                        if limit_velocity:
                            velocities = cfg.max_velocity if dofs[side] == 7 else (
                                cfg.max_velocity[:6] if dofs[side] == 6 else cfg.max_velocity[6:7])
                            commands[side] = rate_limit(desired[side], previous[side], velocities, dt)
                        else:
                            commands[side] = desired[side].copy()
                    compute_ns = time.monotonic_ns()
                    for side in connected:
                        robots[side].command_joint_pos(commands[side])
                    command_ns = time.monotonic_ns()
                    for ident, side, joint, axis, tick, delta_rad, raw, bounded, hit in rows:
                        actual = float(measured[side][axis])
                        command = float(commands[side][axis])
                        writer.writerow([f"{now-started:.3f}", ident, side, joint + 1, tick,
                                         cumulative[ident], f"{delta_rad:.5f}", f"{raw:.5f}",
                                         f"{bounded:.5f}", f"{command:.5f}", f"{actual:.5f}",
                                         int(hit), f"{drifts[side]:.5f}"])
                        unit = "norm" if joint == 6 else "rad"
                        display.append(f"ID {ident:2d} {side[0]}{joint + 1}: leader {tick:4d} tick "
                                       f"(Δ{delta_rad:+.3f} rad) | follower {actual:+.3f} {unit} "
                                       f"cmd {command:+.3f} target {bounded:+.3f}"
                                       f"{' LIMIT' if hit else ''}")
                    record_ns = time.monotonic_ns()
                    count += 1
                    if show_cycle_timing or now - last_display >= .2:
                        if show_cycle_timing:
                            period_text = f"{period_ms:.2f} ms" if period_ms is not None else "首轮"
                            display.append(
                                f"第 {count} 轮耗时: 14舵机读取 {(sample_ns - cycle_ns) / 1e6:.2f} ms | "
                                f"从臂反馈 {(feedback_ns - sample_ns) / 1e6:.2f} ms | "
                                f"计算 {(compute_ns - feedback_ns) / 1e6:.2f} ms | "
                                f"命令下发 {(command_ns - compute_ns) / 1e6:.2f} ms | "
                                f"记录 {(record_ns - command_ns) / 1e6:.2f} ms | "
                                f"本轮处理 {(record_ns - cycle_ns) / 1e6:.2f} ms | "
                                f"上一轮完整周期 {period_text}")
                        sys.stdout.write("\x1b[H\x1b[2J" + "\n".join(display) +
                                         "\nPress x/q/Ctrl+C to stop.\n")
                        sys.stdout.flush()
                        last_display = now
                    previous, last_ticks, last = commands, ticks, now
                    time.sleep(max(0., 1 / cfg.hz - (time.monotonic() - cycle)))
                graceful = True
            except KeyboardInterrupt:
                graceful = True
            finally:
                file.flush()
                print(f"\nRecorded {count} cycles for IDs {chosen} to {path}")
                if graceful:
                    move_to_zero(robots, dofs, limits, cfg)
                print("Controllers closing; keep the followers supported as torque is released.")
        return path
