"""R1 모터 인덱스, 관절 범위, 변형(A5/A7) 구성.

값의 출처와 근거는 docs/r1_model.md 를 따른다.
인덱스 = unitree_hg LowCmd/LowState 의 motor_cmd[i] / motor_state[i] 인덱스.
"""

from dataclasses import dataclass

import numpy as np

# 시뮬(R1_C++.xml) 은 29칸, 실기는 35칸(머리 29, 30 포함). LowCmd 배열 자체는 35칸이다.
NUM_SIM_MOTORS = 29

JOINT_NAMES = [
    "left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee", "left_ankle_pitch", "left_ankle_roll",
    "right_hip_pitch", "right_hip_roll", "right_hip_yaw", "right_knee", "right_ankle_pitch", "right_ankle_roll",
    "waist_roll", "waist_yaw", "waist_pitch",
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw", "left_elbow",
    "left_wrist_roll", "left_wrist_pitch", "left_wrist_yaw",
    "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw", "right_elbow",
    "right_wrist_roll", "right_wrist_pitch", "right_wrist_yaw",
]
JOINT_INDEX = {name: i for i, name in enumerate(JOINT_NAMES)}

LEFT_LEG = list(range(0, 6))
RIGHT_LEG = list(range(6, 12))
WAIST = [12, 13]                 # roll, yaw (14 pitch 는 사용 안 함)
LEFT_ARM_A5 = [15, 16, 17, 18, 19]
RIGHT_ARM_A5 = [22, 23, 24, 25, 26]
WRIST_A7_EXTRA = [20, 21, 27, 28]  # 손목 pitch / yaw (A7 에서만 실제 관절)

# (하한, 상한) [rad]. 시뮬 모델 값. A7 손목은 xr_teleoperate r1_a7.urdf 값.
JOINT_LIMITS = np.array([
    [-2.93, 2.55], [-1.05, 1.75], [-2.74, 2.74], [-0.17, 2.43], [-0.87, 0.58], [-0.26, 0.26],
    [-2.93, 2.55], [-1.75, 1.05], [-2.74, 2.74], [-0.17, 2.43], [-0.87, 0.58], [-0.26, 0.26],
    [-0.52, 0.52], [-2.62, 2.62], [0.0, 0.0],
    [-3.14, 2.09], [-0.23, 2.48], [-1.92, 1.92], [-0.98, 2.19],
    [-1.92, 1.92], [-1.61, 1.61], [-1.61, 1.61],
    [-3.14, 2.09], [-2.48, 0.23], [-1.92, 1.92], [-0.98, 2.19],
    [-1.92, 1.92], [-1.61, 1.61], [-1.61, 1.61],
])

# 기본 PD 게인. G1 공식 예제(다리 60~100 / 40, 팔 40 / 1)를 출발점으로 한 값.
DEFAULT_KP = np.array(
    [60, 60, 60, 100, 40, 40] * 2
    + [60, 60, 0]
    + [40, 40, 40, 40, 20, 10, 10] * 2,
    dtype=float,
)
DEFAULT_KD = np.array(
    [1, 1, 1, 2, 1, 1] * 2
    + [1, 1, 0]
    + [1, 1, 1, 1, 0.5, 0.5, 0.5] * 2,
    dtype=float,
)


@dataclass(frozen=True)
class Variant:
    name: str
    active: tuple[int, ...]  # 명령을 보내는 모터 인덱스


def _variant(name: str, arms: list[int]) -> Variant:
    return Variant(name, tuple(sorted(LEFT_LEG + RIGHT_LEG + WAIST + arms)))


VARIANTS = {
    "a5": _variant("a5", LEFT_ARM_A5 + RIGHT_ARM_A5),
    "a7": _variant("a7", LEFT_ARM_A5 + RIGHT_ARM_A5 + WRIST_A7_EXTRA),
}
