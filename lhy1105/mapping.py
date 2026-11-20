"""Pure mapping and limit checks. No hardware access here."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


class SafetyStop(RuntimeError):
    pass


def tick_delta(current: int, origin: int, ticks_per_turn: int = 4096) -> int:
    return ((current - origin + ticks_per_turn // 2) % ticks_per_turn) - ticks_per_turn // 2


@dataclass(frozen=True)
class ArmMapping:
    ids: tuple[int, ...]
    joint_sources: tuple[int, ...]
    joint_signs: tuple[float, ...]
    gripper_sign: float
    gripper_span_ticks: float

    def target(self, ticks: dict[int, int], start_ticks: dict[int, int], start_q: np.ndarray,
               max_arm_delta: float, max_gripper_delta: float) -> np.ndarray:
        q = np.asarray(start_q, dtype=float).copy()
        if q.shape != (7,):
            raise SafetyStop("follower must report 6 joints and one gripper")
        for joint, source in enumerate(self.joint_sources):
            delta = tick_delta(ticks[source], start_ticks[source]) * (2 * math.pi / 4096) * self.joint_signs[joint]
            if abs(delta) > max_arm_delta:
                raise SafetyStop(f"leader ID {source} exceeds allowed startup displacement")
            q[joint] += delta
        grip_id = self.ids[6]
        grip_delta = tick_delta(ticks[grip_id], start_ticks[grip_id]) * self.gripper_sign / self.gripper_span_ticks
        if abs(grip_delta) > max_gripper_delta:
            raise SafetyStop(f"gripper ID {grip_id} exceeds allowed startup displacement")
        q[6] = np.clip(q[6] + grip_delta, 0, 1)
        if not np.all(np.isfinite(q)):
            raise SafetyStop("nonfinite target")
        return q


def check_limits(target: np.ndarray, limits: np.ndarray, margin: float,
                 label: str = "follower target", start: np.ndarray | None = None) -> None:
    if limits.shape != (6, 2) or not np.all(np.isfinite(limits)):
        raise SafetyStop("follower joint limits unavailable")
    low, high = limits[:, 0] + margin, limits[:, 1] - margin
    outside = np.flatnonzero((target[:6] < low) | (target[:6] > high))
    if outside.size:
        joint = int(outside[0])
        origin = f", ARM start {start[joint]:.3f}" if start is not None else ""
        raise SafetyStop(
            f"{label} joint {joint + 1}: {target[joint]:.3f} rad outside "
            f"[{low[joint]:.3f}, {high[joint]:.3f}] rad{origin} "
            f"(includes {margin:.3f} rad safety margin)"
        )


def rate_limit(target: np.ndarray, previous: np.ndarray, max_velocity: np.ndarray, dt: float) -> np.ndarray:
    return previous + np.clip(target - previous, -max_velocity * dt, max_velocity * dt)
