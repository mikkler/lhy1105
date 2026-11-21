import unittest
from unittest.mock import patch
from pathlib import Path
import json
import tempfile
import time
from types import SimpleNamespace

import numpy as np

from lhy1105.feetech import FeetechBus, ServoError
from lhy1105.calibration import nominal_arm_limits, selected_target, selection
from lhy1105.multi_calibration import follower_axis, move_to_zero, run as multi_run, validate_ids, zero_target
from lhy1105.cli import enable_feedback_timestamps, move_to_rest, teleop, verify_leaders
from lhy1105.config import load_config
from lhy1105.mapping import ArmMapping, SafetyStop, check_limits, rate_limit, tick_delta


class CoreTests(unittest.TestCase):
    def test_hz_200_boundary(self):
        data = json.loads((Path(__file__).resolve().parents[1] / "config.json").read_text())
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.json"
            data["safety"]["hz"] = 200
            path.write_text(json.dumps(data))
            self.assertEqual(load_config(path).hz, 200)
            data["safety"]["hz"] = 200.01
            path.write_text(json.dumps(data))
            with self.assertRaisesRegex(ValueError, "hz must"):
                load_config(path)

    def test_single_joint_selection_and_target(self):
        self.assertEqual(selection(1), ("left", 0))
        self.assertEqual(selection(7), ("left", 6))
        self.assertEqual(selection(8), ("right", 0))
        self.assertEqual(selection(14), ("right", 6))
        with self.assertRaises(ValueError):
            selection(15)
        start = np.array([.1, .2, .3, .4, .5, .6])
        target, delta_rad, delta = selected_target(start, 4, 100, -1, 700)
        self.assertAlmostEqual(delta_rad, 100 * 2 * np.pi / 4096)
        self.assertAlmostEqual(delta, -delta_rad)
        self.assertTrue(np.allclose(target[[0, 1, 2, 3, 5]], start[[0, 1, 2, 3, 5]]))
        self.assertAlmostEqual(target[4], start[4] - delta_rad)
        grip, _, _ = selected_target(np.array([.5]), 6, 70, 1, 700)
        self.assertAlmostEqual(grip[0], .6)
        turns, delta_rad, _ = selected_target(start, 0, 4096, 1, 700)
        self.assertAlmostEqual(delta_rad, 2 * np.pi)
        self.assertAlmostEqual(turns[0], start[0] + 2 * np.pi)

    def test_multi_selection_and_gripper_axis(self):
        self.assertEqual(validate_ids([9, 1, 7, 2], [2]), (1, 2, 7, 9))
        self.assertEqual(follower_axis(2, 6), 1)
        self.assertEqual(follower_axis(7, 1), 0)
        self.assertEqual(follower_axis(7, 7), 6)
        with self.assertRaises(ValueError):
            validate_ids([1, 1], [])
        with self.assertRaises(ValueError):
            validate_ids([1, 2], [3])

    def test_joint2_motion_uses_nominal_nonnegative_range(self):
        limits = nominal_arm_limits()
        self.assertEqual(limits.shape, (6, 2))
        self.assertAlmostEqual(limits[1, 0], 0.)
        self.assertAlmostEqual(limits[1, 1], 3.66519)

    def test_multi_return_to_configured_zero(self):
        class FakeRobot:
            def __init__(self):
                self.q = np.array([.2, .1, -.1, .2, 0., .1, .5])
                self.commands = []

            def get_joint_pos_and_can_rx_ns(self):
                return self.q.copy(), np.full(7, time.monotonic_ns())

            def command_joint_pos(self, q):
                self.q = np.asarray(q).copy()
                self.commands.append(self.q)

        cfg = load_config(Path(__file__).resolve().parents[1] / "config.json")
        robots = {"left": FakeRobot(), "right": FakeRobot()}
        limits = {side: np.array([[-2., 2.]] * 6) for side in robots}
        with patch("lhy1105.multi_calibration.time.sleep"):
            move_to_zero(robots, {"left": 7, "right": 7}, limits, cfg)
        for robot in robots.values():
            self.assertGreaterEqual(len(robot.commands), 100)
            self.assertTrue(np.allclose(robot.q, zero_target(7)))

    def test_live_uses_full_multi_joint_path(self):
        cfg = load_config(Path(__file__).resolve().parents[1] / "config.json")
        self.assertTrue(cfg.mapping_confirmed)
        with patch("sys.stdin.isatty", return_value=True), patch("lhy1105.multi_calibration.run") as multi_run:
            teleop(object(), cfg, True, 10.)
        multi_run.assert_called_once()
        args, kwargs = multi_run.call_args
        self.assertEqual(args[2], list(range(1, 15)))
        self.assertEqual(kwargs, {"duration": 10., "lock_others": True,
                                  "show_cycle_timing": True, "limit_velocity": False})

    def test_six_joint_follower_configuration_avoids_missing_grippers(self):
        cfg = load_config(Path(__file__).resolve().parents[1] / "config_can_leader.json")
        self.assertEqual(cfg.channels, {"left": "can_leader_l", "right": "can_leader_r"})
        self.assertEqual(cfg.follower_has_gripper, {"left": False, "right": False})
        self.assertFalse(cfg.mapping_confirmed)
        with self.assertRaisesRegex(SafetyStop, "mapping_confirmed"):
            teleop(object(), cfg, True, None)
        with self.assertRaisesRegex(ValueError, "no follower gripper"):
            multi_run(object(), cfg, [7])
        bus = SimpleNamespace(ping=unittest.mock.Mock(), model=unittest.mock.Mock(return_value=777))
        verify_leaders(bus, cfg, cfg.control_ids)
        checked = [call.args[0] for call in bus.ping.call_args_list]
        self.assertEqual(checked, [1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13])
        with patch("sys.stdin.isatty", return_value=True), patch("lhy1105.multi_calibration.run") as run:
            teleop(object(), cfg, True, 10., trial=True)
        args, kwargs = run.call_args
        self.assertEqual(args[2], [1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13])
        self.assertEqual(kwargs, {"duration": 10., "lock_others": True,
                                  "show_cycle_timing": True, "limit_velocity": True})
        with patch("sys.stdin.isatty", return_value=True), patch("lhy1105.multi_calibration.run") as run:
            teleop(object(), cfg, True, 10., trial=True, output="/tmp/all.csv")
        self.assertEqual(run.call_args.kwargs["output"], "/tmp/all.csv")
        with self.assertRaisesRegex(SafetyStop, "--trial"):
            teleop(object(), cfg, False, None, trial=True)

    def test_ping_packet_and_reply(self):
        self.assertEqual(FeetechBus.packet(1, 1), bytes.fromhex("ff ff 01 02 01 fb"))
        self.assertEqual(FeetechBus.parse_reply(bytes.fromhex("ff ff 01 02 00 fc"), 1, 0), b"")
        with self.assertRaises(ServoError):
            FeetechBus.parse_reply(bytes.fromhex("ff ff 01 02 00 00"), 1, 0)

    def test_tick_wrap_and_mapping_limit(self):
        self.assertEqual(tick_delta(2, 4094), 4)
        mapping = ArmMapping((1, 2, 3, 4, 5, 6, 7), (1, 2, 3, 4, 6, 5),
                             (1, 1, -1, 1, -1, 1), 1, 700)
        origin = {i: 1000 for i in range(1, 8)}
        current = origin | {1: 1100, 7: 1070}
        target = mapping.target(current, origin, np.zeros(7), .35, .25)
        self.assertAlmostEqual(target[0], 100 * 2 * np.pi / 4096)
        self.assertAlmostEqual(target[6], .1)
        current[1] = 1400
        with self.assertRaises(SafetyStop):
            mapping.target(current, origin, np.zeros(7), .35, .25)

    def test_limits_and_velocity(self):
        limits = np.array([[-1., 1.]] * 6)
        check_limits(np.zeros(7), limits, .1)
        with self.assertRaisesRegex(SafetyStop, r"left target joint 1: 0.950 rad outside \[-0.900, 0.900\] rad, ARM start 0.800"):
            check_limits(np.array([.95, 0, 0, 0, 0, 0, 0]), limits, .1,
                         "left target", start=np.array([.8, 0, 0, 0, 0, 0, 0]))
        check_limits(np.array([-.099, 0, 0, 0, 0, 0, 0]),
                     np.array([[-.15, 3.292]] + [[-1., 1.]] * 5), 0.)
        with self.assertRaises(SafetyStop):
            check_limits(np.array([-.151, 0, 0, 0, 0, 0, 0]),
                         np.array([[-.15, 3.292]] + [[-1., 1.]] * 5), 0.)
        self.assertTrue(np.allclose(rate_limit(np.ones(7), np.zeros(7), np.ones(7), .1), .1))

    def test_graceful_shutdown_commands_both_followers_to_rest(self):
        class FakeRobot:
            def __init__(self):
                self.q = np.array([.2, .1, .1, -.1, 0., 0., .8])
                self.commands = []

            def get_joint_pos_and_can_rx_ns(self):
                return self.q.copy(), np.full(7, time.monotonic_ns())

            def get_robot_info(self):
                return {"joint_limits": np.array([[-2., 2.]] * 6)}

            def command_joint_pos(self, q):
                self.q = np.asarray(q).copy()
                self.commands.append(self.q)

        config = load_config(Path(__file__).resolve().parents[1] / "config.json")
        robots = {"left": FakeRobot(), "right": FakeRobot()}
        with patch("lhy1105.cli.time.sleep"):
            move_to_rest(robots, config)
        for robot in robots.values():
            self.assertGreaterEqual(len(robot.commands), 100)
            self.assertTrue(np.allclose(robot.q, [0, 0, 0, 0, 0, 0, .99]))

    def test_enables_can_timestamps_before_requiring_fresh_feedback(self):
        class FakeRobot:
            motor_chain = SimpleNamespace(motor_interface=SimpleNamespace(trace_timestamps_enabled=False))

            def get_joint_pos_and_can_rx_ns(self):
                timestamp = time.monotonic_ns() if self.motor_chain.motor_interface.trace_timestamps_enabled else 0
                return np.zeros(7), np.full(7, timestamp)

            def get_robot_info(self):
                return {"joint_limits": np.array([[-2., 2.]] * 6)}

        config = load_config(Path(__file__).resolve().parents[1] / "config.json")
        robot = FakeRobot()
        enable_feedback_timestamps(robot, config)
        self.assertTrue(robot.motor_chain.motor_interface.trace_timestamps_enabled)


if __name__ == "__main__":
    unittest.main()
