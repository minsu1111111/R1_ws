"""R1 조작 장면 생성 (MjSpec).

원본 R1 모델(third_party/unitree_mujoco/unitree_robots/r1/scene.xml)을 읽어 코드로 테이블·상자·목표 영역·카메라를
붙인다. 원본 파일은 수정하지 않는다 (docs/decisions.md D8).

장면 치수는 팔 작업 영역(docs/kinematics.md)에 맞춘다:
  테이블 윗면 = 골반 높이 - 0.05 m, 물체는 몸 앞 x = 0.20 ~ 0.35 m.

    model = build_model(SceneConfig(base="fixed"))
"""

from dataclasses import dataclass, field
from pathlib import Path

import mujoco
import numpy as np

WS_ROOT = Path(__file__).resolve().parents[2]
R1_SCENE = WS_ROOT / "third_party/unitree_mujoco/unitree_robots/r1/scene.xml"

# 기본 자세(모든 관절 0)에서 발바닥이 바닥에 닿는 골반 높이 (발 최저점 측정값 0.74 - (-0.0726))
STAND_PELVIS_Z = 0.813


@dataclass
class SceneConfig:
    base: str = "fixed"            # fixed: 골반 고정 (상체 조작용) | band: 고무줄 | free: 자유
    table: bool = True
    table_top_rel_pelvis: float = -0.05
    table_x: tuple[float, float] = (0.20, 0.60)   # 앞 가장자리, 뒤 가장자리 (골반 기준)
    table_y: tuple[float, float] = (-0.40, 0.40)
    box_size: float = 0.05         # 한 변 [m]
    box_mass: float = 0.2          # [kg]
    box_xy: tuple[float, float] = (0.27, 0.22)
    target_xy: tuple[float, float] = (0.27, 0.0)
    target_radius: float = 0.04
    cameras: bool = True
    # 머리 카메라: R1 시뮬 모델에는 머리가 없어 torso_link 에 근사 위치로 단다
    # (URDF 기준 머리 = torso 위 약 0.37 m). 아래쪽 테이블을 본다.
    head_cam_pos: tuple[float, float, float] = (0.10, 0.0, 0.42)
    head_cam_target: tuple[float, float, float] = (0.32, 0.0, -0.05)
    front_cam_pos: tuple[float, float, float] = (1.3, 0.0, 1.3)
    extra: dict = field(default_factory=dict)


def table_top_z(cfg: SceneConfig) -> float:
    return STAND_PELVIS_Z + cfg.table_top_rel_pelvis


def look_at_quat(pos, target, up=(0.0, 0.0, 1.0)) -> np.ndarray:
    """카메라를 pos 에서 target 으로 향하게 하는 쿼터니언 (MuJoCo 카메라: -z 방향을 봄, +y 가 위)."""
    f = np.asarray(target, float) - np.asarray(pos, float)
    f /= np.linalg.norm(f)
    right = np.cross(f, up)
    right /= np.linalg.norm(right)
    cam_up = np.cross(right, f)
    mat = np.column_stack([right, cam_up, -f])  # 카메라 x, y, z 축
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, mat.flatten())
    return quat


def build_model(cfg: SceneConfig | None = None) -> mujoco.MjModel:
    cfg = cfg or SceneConfig()
    spec = mujoco.MjSpec.from_file(str(R1_SCENE))
    world = spec.worldbody

    pelvis = spec.body("pelvis")
    pelvis.pos = [0.0, 0.0, STAND_PELVIS_Z]
    if cfg.base == "fixed":
        spec.delete(spec.joint("floating_base_joint"))
    elif cfg.base not in ("band", "free"):
        raise ValueError(f"base must be fixed | band | free, got {cfg.base}")

    if cfg.table:
        top = table_top_z(cfg)
        thick = 0.04
        (x0, x1), (y0, y1) = cfg.table_x, cfg.table_y
        world.add_geom(
            name="table", type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[(x1 - x0) / 2, (y1 - y0) / 2, thick / 2],
            pos=[(x0 + x1) / 2, (y0 + y1) / 2, top - thick / 2],
            rgba=[0.55, 0.40, 0.28, 1.0], friction=[0.6, 0.005, 0.0001],
        )

        h = cfg.box_size / 2
        box = world.add_body(name="box", pos=[cfg.box_xy[0], cfg.box_xy[1], top + h])
        box.add_freejoint(name="box_free")
        box.add_geom(
            name="box", type=mujoco.mjtGeom.mjGEOM_BOX, size=[h, h, h], mass=cfg.box_mass,
            rgba=[0.85, 0.20, 0.15, 1.0], friction=[0.6, 0.005, 0.0001], condim=4,
        )

        # 목표 영역: 보이기만 하고 충돌하지 않는다
        world.add_geom(
            name="target", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
            size=[cfg.target_radius, 0.001, 0], pos=[cfg.target_xy[0], cfg.target_xy[1], top + 0.001],
            rgba=[0.2, 0.8, 0.3, 0.5], contype=0, conaffinity=0,
        )

    if cfg.cameras:
        torso = spec.body("torso_link")
        torso.add_camera(
            name="head_cam", pos=list(cfg.head_cam_pos),
            quat=look_at_quat(cfg.head_cam_pos, cfg.head_cam_target).tolist(), fovy=70,
        )
        front_target = (0.25, 0.0, STAND_PELVIS_Z - 0.05)
        world.add_camera(
            name="front_cam", pos=list(cfg.front_cam_pos),
            quat=look_at_quat(cfg.front_cam_pos, front_target).tolist(), fovy=50,
        )

    return spec.compile()
