"""Configuration validation and loading."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .mapping import ArmMapping


@dataclass(frozen=True)
class Config:
    port: str
    baudrate: int
    channels: dict[str, str]
    mappings: dict[str, ArmMapping]
    mapping_confirmed: bool
    hz: float
    max_feedback_age_s: float
    joint_limit_margin_rad: float
    max_velocity: np.ndarray

    @property
    def ids(self) -> tuple[int, ...]:
        return self.mappings["left"].ids + self.mappings["right"].ids


def load_config(path: str | Path) -> Config:
    data = json.loads(Path(path).read_text())
    mappings = {}
    for side in ("left", "right"):
        item = data["arms"][side]
        ids = tuple(int(x) for x in item["leader_ids"])
        sources = tuple(int(x) for x in item["joint_sources"])
        signs = tuple(float(x) for x in item["joint_signs"])
        if len(ids) != 7 or len(set(ids)) != 7 or len(sources) != 6 or len(set(sources)) != 6:
            raise ValueError(f"{side}: expected seven unique IDs and six unique joint sources")
        if not set(sources).issubset(set(ids[:6])) or any(abs(x) != 1 for x in signs):
            raise ValueError(f"{side}: joint mapping invalid")
        if float(item["gripper_span_ticks"]) <= 0 or abs(float(item["gripper_sign"])) != 1:
            raise ValueError(f"{side}: gripper mapping invalid")
        mappings[side] = ArmMapping(ids, sources, signs, float(item["gripper_sign"]), float(item["gripper_span_ticks"]))
    if len(set(mappings["left"].ids + mappings["right"].ids)) != 14:
        raise ValueError("leader IDs must be unique across both arms")
    safety = data["safety"]
    positive = ("hz", "max_feedback_age_s")
    if any(not 0 < float(safety[key]) < 100 for key in positive):
        raise ValueError("safety values must be positive and finite")
    if not 0 <= float(safety["joint_limit_margin_rad"]) < 100:
        raise ValueError("joint limit margin must be nonnegative and finite")
    velocity = np.asarray(safety["max_velocity"], dtype=float)
    if velocity.shape != (7,) or not np.all(np.isfinite(velocity)) or np.any(velocity <= 0):
        raise ValueError("max_velocity must contain seven positive finite values")
    channels = {side: data["arms"][side]["follower_can"] for side in ("left", "right")}
    if channels["left"] == channels["right"]:
        raise ValueError("left and right followers need different CAN interfaces")
    return Config(
        port=data["serial_port"], baudrate=int(data["baudrate"]), channels=channels,
        mappings=mappings, mapping_confirmed=data["mapping_confirmed"] is True,
        max_velocity=velocity,
        joint_limit_margin_rad=float(safety["joint_limit_margin_rad"]),
        **{key: float(safety[key]) for key in positive},
    )
