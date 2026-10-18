"""손 끝 직선 이동 (IK + 중력 보상 + 속도 피드포워드).

    arm = CartesianArm(robot, ik, "left")
    arm.move_to([0.27, 0.30, 0.06], speed=0.10)    # 현재 손 끝 → 목표까지 직선, 0.10 m/s

중력 보상은 양팔 + 허리에 넣는다. 허리를 빼면 팔 무게로 허리 roll 이 처져 손 끝이 ~16 mm 밀린다 (docs/kinematics.md).
"""

import time

import numpy as np

from r1.interface.joints import WAIST
from r1.interface.robot import R1LowLevel
from r1.kinematics.arm_ik import ArmIK


class CartesianArm:
    def __init__(self, robot: R1LowLevel, ik: ArmIK, side: str = "left", dt: float = 0.01):
        self.robot, self.ik, self.side, self.dt = robot, ik, side, dt
        self.comp = ik.arm_motors("left") + ik.arm_motors("right") + WAIST
        self.q_cmd = robot.q().copy()

    def tip(self) -> np.ndarray:
        """측정 관절각으로 계산한 현재 손 끝 위치 (pelvis 기준)."""
        return self.ik.fk(self.robot.q(), self.side)

    def hold(self, seconds: float):
        end = time.time() + seconds
        while time.time() < end and not self.robot.damped:
            self.robot.set_target(self.q_cmd, tau_ff=self.ik.gravity_torque(self.robot.q(), self.comp))
            time.sleep(self.dt)

    def servo(self, target, dt: float | None = None) -> bool:
        """손 끝 목표로 한 주기만 IK 를 풀어 명령한다 (원격조작용, 매 주기 호출).

        IK 가 실패하면(닿지 않는 곳) 명령을 바꾸지 않고 False.
        """
        dt = dt or self.dt
        q_new, ok, _ = self.ik.solve(np.asarray(target, dtype=float), self.q_cmd, self.side, restarts=0)
        if not ok:
            q_new = self.q_cmd
        dq_ff = (q_new - self.q_cmd) / dt
        self.q_cmd = q_new
        self.robot.set_target(self.q_cmd, tau_ff=self.ik.gravity_torque(self.robot.q(), self.comp), dq_ff=dq_ff)
        return ok

    def reset(self, q29: np.ndarray):
        """로봇이 순간이동했을 때(시뮬 리셋) 명령 상태를 맞춘다."""
        self.q_cmd = np.asarray(q29, dtype=float).copy()
        self.robot.reset_command(self.q_cmd)

    def move_to(self, target, speed: float = 0.10) -> float:
        """손 끝을 지금 명령 위치에서 target 까지 직선으로 이동. 끝난 뒤 위치 오차[m] 반환."""
        target = np.asarray(target, dtype=float)
        start = self.ik.fk(self.q_cmd, self.side)
        duration = max(np.linalg.norm(target - start) / speed, self.dt)
        t0 = t_prev = time.time()
        q_prev = self.q_cmd.copy()
        while not self.robot.damped:
            now = time.time()
            s = min((now - t0) / duration, 1.0)
            s = 0.5 - 0.5 * np.cos(np.pi * s)  # 출발·도착에서 부드럽게
            p = start + s * (target - start)
            self.q_cmd, _, _ = self.ik.solve(p, self.q_cmd, self.side, restarts=0)
            dq_ff = (self.q_cmd - q_prev) / max(now - t_prev, 1e-3)
            q_prev, t_prev = self.q_cmd.copy(), now
            self.robot.set_target(
                self.q_cmd, tau_ff=self.ik.gravity_torque(self.robot.q(), self.comp), dq_ff=dq_ff
            )
            if now - t0 >= duration:
                break
            time.sleep(self.dt)
        self.hold(0.3)
        return float(np.linalg.norm(self.tip() - target))
