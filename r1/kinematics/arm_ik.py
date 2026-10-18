"""R1 팔 정기구학(FK) / 역기구학(IK) (pinocchio).

- 모델: third_party/unitree_ros/robots/r1_description/R1.urdf (r1_description 과 같은 파일)
- 좌표계: 골반(pelvis_link) 기준. x 앞, y 왼쪽, z 위.
- 손 끝: wrist_roll_link 에서 x 방향 HAND_TIP_OFFSET (주먹 중심, 메시 길이 0.143 m).
- 입출력은 r1.interface 와 같은 29칸 배열 (인덱스 = 모터 번호, docs/r1_model.md).
- IK 는 위치만 맞춘다 (5자유도 팔은 손 끝 위치 3 + 자세 일부만 가능).
  지정한 팔 관절만 움직이고 나머지(허리 등)는 입력 값을 그대로 쓴다.

    ik = ArmIK()
    pos = ik.fk(q29, "left")                       # 손 끝 위치 [m]
    q29_new, ok, err = ik.solve(target, q29, "left")
"""

from pathlib import Path

import numpy as np
import pinocchio as pin

from r1.interface.joints import JOINT_INDEX, JOINT_LIMITS, NUM_SIM_MOTORS

WS_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_URDF = WS_ROOT / "third_party/unitree_ros/robots/r1_description/R1.urdf"

HAND_TIP_OFFSET = np.array([0.10, 0.0, 0.0])

ARM_JOINTS = {
    "a5": ["shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll"],
    # A7 전신 URDF 가 없어 현재 모델에서는 a5 만 동작한다 (docs/decisions.md D10)
}


class ArmIK:
    def __init__(self, urdf_path: str | Path = DEFAULT_URDF, variant: str = "a5"):
        self.model = pin.buildModelFromUrdf(str(urdf_path))  # 고정 베이스 = pelvis_link
        self.data = self.model.createData()
        self.arm_names = ARM_JOINTS[variant]

        # 29칸 모터 인덱스 ↔ pinocchio q 인덱스
        self._motor_to_q = {}
        for i in range(NUM_SIM_MOTORS):
            name = [n for n, k in JOINT_INDEX.items() if k == i][0] + "_joint"
            if self.model.existJointName(name):
                self._motor_to_q[i] = self.model.joints[self.model.getJointId(name)].idx_q

        # 손 끝 프레임 추가
        self.tip_frame = {}
        for side in ("left", "right"):
            parent = self.model.getFrameId(f"{side}_wrist_roll_link")
            f = self.model.frames[parent]
            frame = pin.Frame(
                f"{side}_hand_tip", f.parentJoint, parent,
                f.placement * pin.SE3(np.eye(3), HAND_TIP_OFFSET), pin.FrameType.OP_FRAME,
            )
            self.tip_frame[side] = self.model.addFrame(frame)
        self.data = self.model.createData()

    # ------------------------------------------------------------------ 변환
    def arm_motors(self, side: str) -> list[int]:
        return [JOINT_INDEX[f"{side}_{n}"] for n in self.arm_names]

    def to_pin(self, q29: np.ndarray) -> np.ndarray:
        q = pin.neutral(self.model)
        for i, k in self._motor_to_q.items():
            q[k] = q29[i]
        return q

    # ------------------------------------------------------------------ FK
    def fk(self, q29: np.ndarray, side: str) -> np.ndarray:
        """손 끝 위치 (pelvis 기준) [m]."""
        return self.fk_pose(q29, side).translation.copy()

    def fk_pose(self, q29: np.ndarray, side: str) -> pin.SE3:
        q = self.to_pin(q29)
        pin.framesForwardKinematics(self.model, self.data, q)
        return self.data.oMf[self.tip_frame[side]]

    # ------------------------------------------------------------------ 동역학
    def gravity_torque(self, q29: np.ndarray, motors: list[int] | None = None) -> np.ndarray:
        """중력을 버티는 데 필요한 관절 토크 (29칸, N·m). 골반이 똑바로 서 있다고 가정.

        motors 를 주면 그 인덱스만 채우고 나머지는 0 (예: 팔만 보상).
        """
        g = pin.computeGeneralizedGravity(self.model, self.data, self.to_pin(q29))
        tau = np.zeros(NUM_SIM_MOTORS)
        for i, k in self._motor_to_q.items():
            if motors is None or i in motors:
                tau[i] = g[k]
        return tau

    # ------------------------------------------------------------------ IK
    def solve(
        self,
        target: np.ndarray,
        q29_init: np.ndarray,
        side: str,
        restarts: int = 8,
        seed: int = 0,
        **kwargs,
    ) -> tuple[np.ndarray, bool, float]:
        """손 끝을 target(pelvis 기준, m)으로 보내는 29칸 관절각.

        q29_init 에서 먼저 풀고, 실패하면(관절 한계에 갇힘) 무작위 시작 자세로 restarts 번 다시 푼다.
        반환: (q29, 성공 여부, 남은 위치 오차[m]). 닿을 수 없는 점이면 가장 가까운 자세와 ok=False.
        """
        best = self._solve_once(target, q29_init, side, **kwargs)
        if best[1]:
            return best
        motors = self.arm_motors(side)
        rng = np.random.default_rng(seed)
        for _ in range(restarts):
            q0 = np.array(q29_init, dtype=float)
            q0[motors] = rng.uniform(JOINT_LIMITS[motors, 0], JOINT_LIMITS[motors, 1])
            cand = self._solve_once(target, q0, side, **kwargs)
            if cand[2] < best[2]:
                best = cand
            if best[1]:
                break
        return best

    def _solve_once(
        self,
        target: np.ndarray,
        q29_init: np.ndarray,
        side: str,
        iters: int = 200,
        tol: float = 1e-3,
        damping: float = 1e-2,
        step: float = 1.0,
        max_dq: float = 0.2,
        limit_margin: float = 0.05,
    ) -> tuple[np.ndarray, bool, float]:
        motors = self.arm_motors(side)
        q_idx = [self._motor_to_q[i] for i in motors]
        v_idx = q_idx  # 회전 관절만 있으므로 idx_q == idx_v
        lo = JOINT_LIMITS[motors, 0] + limit_margin
        hi = JOINT_LIMITS[motors, 1] - limit_margin
        fid = self.tip_frame[side]

        q = self.to_pin(np.asarray(q29_init, dtype=float))
        q[q_idx] = np.clip(q[q_idx], lo, hi)
        best_err, best_q = np.inf, q[q_idx].copy()
        for _ in range(iters):
            pin.framesForwardKinematics(self.model, self.data, q)
            e = np.asarray(target) - self.data.oMf[fid].translation
            err = float(np.linalg.norm(e))
            if err < best_err:  # 닿지 않는 목표에서 발산해도 가장 가까웠던 자세를 돌려준다
                best_err, best_q = err, q[q_idx].copy()
            if err < tol:
                break
            J = pin.computeFrameJacobian(self.model, self.data, q, fid, pin.LOCAL_WORLD_ALIGNED)[:3, v_idx]
            dq = J.T @ np.linalg.solve(J @ J.T + damping**2 * np.eye(3), e)
            dq *= min(1.0, max_dq / (np.abs(dq).max() + 1e-12))  # 특이 자세 근처 큰 점프 방지
            q[q_idx] = np.clip(q[q_idx] + step * dq, lo, hi)

        q29 = np.array(q29_init, dtype=float)
        q29[motors] = best_q
        return q29, best_err < tol, best_err
