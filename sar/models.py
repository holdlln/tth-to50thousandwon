from dataclasses import dataclass, field
from math import atan2, cos, sin, hypot
from typing import Protocol
import numpy as np


def wrap(angle: float) -> float:
    return atan2(sin(angle), cos(angle))


@dataclass
class Pose:
    x: float
    y: float
    yaw: float

    def distance(self, xy) -> float:
        return hypot(self.x - xy[0], self.y - xy[1])

    def transform(self, points: np.ndarray) -> np.ndarray:
        c, s = cos(self.yaw), sin(self.yaw)
        return points @ np.array([[c, s], [-s, c]]) + [self.x, self.y]


@dataclass
class SensorFrame:
    time: float
    left_encoder: float
    right_encoder: float
    ranges: np.ndarray
    angles: np.ndarray
    rgb: np.ndarray | None = None
    camera_fov: float = 1.4
    bumper: bool = False
    gyro_z: float | None = None
    gyro_delta: float | None = None


@dataclass
class Detection:
    label: str
    position: np.ndarray
    confidence: float
    bearing: float


@dataclass
class Target:
    label: str
    position: np.ndarray
    first_seen: float
    last_seen: float
    observations: int = 1
    confirmed: bool = False
    visited: bool = False
    evidence_positions: list = field(default_factory=list)


class Detector(Protocol):
    def detect(self, frame: SensorFrame, pose: Pose) -> list[Detection]: ...


class FrontierPolicy(Protocol):
    def score(self, information: float, travel: float, return_cost: float,
              uncertainty: float, revisit: float) -> float: ...
