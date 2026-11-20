"""Bimanual leader-to-YAM teleoperation command line."""

from __future__ import annotations

import argparse
import select
import sys
import termios
import time
import tty
from contextlib import ExitStack, contextmanager
from pathlib import Path

import numpy as np

from .config import Config, load_config
from .feetech import FeetechBus
from .mapping import SafetyStop, check_limits, rate_limit, tick_delta


DEFAULT_CONFIG = Path(__file__).resolve().parent.parent / "config.json"


@contextmanager
def keyboard():
    old = termios.tcgetattr(sys.stdin.fileno())
    try:
        tty.setcbreak(sys.stdin.fileno())
        yield
    finally:
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old)


def key_now() -> str:
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return ""


def verify_leaders(bus: FeetechBus, cfg: Config) -> None:
    for ident in cfg.ids:
        bus.ping(ident)
        model = bus.model(ident)
        if model != 777:
            raise SafetyStop(f"leader ID {ident}: model {model}, expected STS3215 (777)")


def print_sample(bus: FeetechBus, cfg: Config) -> None:
    sample = bus.sample(cfg.ids)
    for side in ("left", "right"):
        ids = cfg.mappings[side].ids
        print(f"{side}: " + " ".join(f"{ident}={sample.positions[ident]}" for ident in ids))


def connect_followers(cfg: Config, stack: ExitStack):
    from i2rt.robots.get_robot import get_yam_robot
    from i2rt.robots.utils import GripperType

    robots = {}
    for side in ("left", "right"):
        robot = get_yam_robot(channel=cfg.channels[side],
                              gripper_type=GripperType.LINEAR_4310,
                              zero_gravity_mode=False, enable_auto_recovery=False)
        stack.callback(robot.close)
        if robot.num_dofs() != 7:
            raise SafetyStop(f"{side}: expected 7 follower DOFs")
        enable_feedback_timestamps(robot, cfg)
        robots[side] = robot
    return robots


def enable_feedback_timestamps(robot, cfg: Config) -> None:
    """Enable I2RT CAN receive timestamps and wait for a fresh full sample."""
    interface = getattr(getattr(robot, "motor_chain", None), "motor_interface", None)
    if interface is None or not hasattr(interface, "trace_timestamps_enabled"):
        raise SafetyStop("follower CAN receive timestamps unavailable")
    interface.trace_timestamps_enabled = True
    deadline = time.monotonic() + 2.0
    while True:
        try:
            follower_state(robot, cfg)
            return
        except SafetyStop as exc:
            if str(exc) != "follower CAN feedback stale" or time.monotonic() >= deadline:
                raise
            time.sleep(0.02)


def follower_state(robot, cfg: Config) -> tuple[np.ndarray, np.ndarray]:
    q, timestamps = robot.get_joint_pos_and_can_rx_ns()
    q, timestamps = np.asarray(q, dtype=float).copy(), np.asarray(timestamps).copy()
    if q.shape != (7,) or timestamps.shape != (7,) or not np.all(np.isfinite(q)):
        raise SafetyStop("invalid follower state")
    ages = (time.monotonic_ns() - timestamps) / 1e9
    if np.any(timestamps <= 0) or np.any(ages < -0.01) or np.any(ages > cfg.max_feedback_age_s):
        raise SafetyStop("follower CAN feedback stale")
    limits = np.asarray(robot.get_robot_info()["joint_limits"], dtype=float)
    return q, limits


def move_to_rest(robots: dict, cfg: Config) -> None:
    """Controlled shutdown pose from the earlier I2RT teleop setup."""
    target = np.array([0., 0., 0., 0., 0., 0., .99])
    starts = {}
    for side, robot in robots.items():
        starts[side], limits = follower_state(robot, cfg)
        check_limits(target, limits, cfg.joint_limit_margin_rad, f"{side} rest target")
    # Use at least the old setup's 100 steps / 3 seconds, and obey speed limits.
    needed = max(float(np.max(np.abs(target - starts[side]) / cfg.max_velocity)) for side in robots)
    duration = max(3.0, needed)
    steps = max(100, int(duration * cfg.hz))
    print(f"Moving both followers toward configured rest over {duration:.1f}s; keep them supported.")
    for step in range(1, steps + 1):
        cycle_start = time.monotonic()
        alpha = step / steps
        for side, robot in robots.items():
            follower_state(robot, cfg)
            robot.command_joint_pos(starts[side] * (1 - alpha) + target * alpha)
        time.sleep(max(0., duration / steps - (time.monotonic() - cycle_start)))
    for side, robot in robots.items():
        measured, _ = follower_state(robot, cfg)
        if np.any(np.abs(measured[:6] - target[:6]) > .15) or abs(measured[6] - target[6]) > .15:
            raise SafetyStop(f"{side} did not reach rest; keep supporting the arm")
    print("Both followers reached the rest command and feedback is within tolerance.")


def teleop(bus: FeetechBus, cfg: Config, live: bool, duration: float | None) -> None:
    if live and not cfg.mapping_confirmed:
        raise SafetyStop("set mapping_confirmed=true after checking directions and joint order in config.json")
    if live and not sys.stdin.isatty():
        raise SafetyStop("live mode requires an interactive terminal")
    if live:
        from .multi_calibration import run as multi_calibrate

        multi_calibrate(bus, cfg, list(cfg.ids), duration=duration, lock_others=True,
                        show_cycle_timing=True, limit_velocity=False)
        return

    initial = bus.sample(cfg.ids).positions
    previous = {side: np.zeros(7) for side in ("left", "right")}
    cumulative = {ident: 0 for ident in cfg.ids}
    prior_ticks = initial
    last_tick = started = time.monotonic()
    print("Dry run: printing target offsets; no follower command will be sent.")
    while duration is None or time.monotonic() - started < duration:
        cycle_start = time.monotonic()
        sample = bus.sample(cfg.ids)
        dt = sample.completed_at - last_tick
        if dt <= 0 or dt > 0.25:
            raise SafetyStop("control loop stalled")
        if sample.completed_at - cycle_start > 0.2:
            raise SafetyStop("leader sample too slow")
        for ident in cfg.ids:
            cumulative[ident] += tick_delta(sample.positions[ident], prior_ticks[ident])
        commands = {}
        for side in ("left", "right"):
            mapping = cfg.mappings[side]
            target = np.zeros(7)
            for joint, source in enumerate(mapping.joint_sources):
                target[joint] = cumulative[source] * (2 * np.pi / 4096) * mapping.joint_signs[joint]
            target[6] = cumulative[mapping.ids[6]] * mapping.gripper_sign / mapping.gripper_span_ticks
            commands[side] = rate_limit(target, previous[side], cfg.max_velocity, dt)
        if int((sample.completed_at - started) * 2) != int((last_tick - started) * 2):
            print(" ".join(f"{side}={np.round(commands[side], 3)}"
                           for side in ("left", "right")))
        previous = commands
        prior_ticks = sample.positions
        last_tick = sample.completed_at
        time.sleep(max(0.0, 1 / cfg.hz - (time.monotonic() - cycle_start)))


def main() -> None:
    parser = argparse.ArgumentParser(description="STS3215 dual-arm teleoperation via FE-URT-2")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("scan", help="read-only ID and model check")
    sub.add_parser("monitor", help="read-only live position print")
    run = sub.add_parser("teleop", help="dry run unless --live is given")
    run.add_argument("--live", action="store_true", help="enable follower control after prompts")
    run.add_argument("--duration", type=float, help="stop after this many seconds")
    cal = sub.add_parser("calibrate", help="follow and record one matching leader/follower joint")
    cal.add_argument("--id", type=int, required=True, dest="leader_id", help="leader servo ID 1..14")
    cal.add_argument("--sign", type=int, choices=(-1, 1), help="follower direction (default: config.json)")
    cal.add_argument("--duration", type=float, help="optional maximum recording time in seconds")
    cal.add_argument("--output", help="CSV recording path")
    multi = sub.add_parser("multi", help="follow and record several matching leader/follower joints")
    multi.add_argument("--ids", type=int, nargs="+", required=True, help="selected leader IDs, e.g. --ids 1 2 8 9")
    multi.add_argument("--reverse", type=int, nargs="*", default=[], help="selected IDs whose direction to reverse")
    multi.add_argument("--lock-others", action="store_true", help="connect both full followers and hold unselected joints")
    multi.add_argument("--duration", type=float, help="optional maximum recording time in seconds")
    multi.add_argument("--output", help="CSV recording path")
    args = parser.parse_args()
    try:
        cfg = load_config(args.config)
        with FeetechBus(cfg.port, cfg.baudrate) as bus:
            verify_leaders(bus, cfg)
            print("14 leader servos online; model 777 (STS3215).")
            if args.command == "scan":
                print_sample(bus, cfg)
            elif args.command == "monitor":
                while True:
                    print_sample(bus, cfg)
                    time.sleep(0.5)
            elif args.command == "calibrate":
                from .calibration import run as calibrate
                calibrate(bus, cfg, args.leader_id, sign=args.sign,
                          duration=args.duration, output=args.output)
            elif args.command == "multi":
                from .multi_calibration import run as multi_calibrate
                multi_calibrate(bus, cfg, args.ids, reverse=args.reverse,
                                duration=args.duration, output=args.output,
                                lock_others=args.lock_others)
            else:
                teleop(bus, cfg, args.live, args.duration)
    except KeyboardInterrupt:
        print("\nStopped by operator.")
    except (OSError, ValueError, RuntimeError) as exc:
        print(f"SAFETY STOP: {exc}", file=sys.stderr)
        raise SystemExit(2) from exc


if __name__ == "__main__":
    main()
