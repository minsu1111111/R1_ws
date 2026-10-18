"""왼손 끝을 몸 앞에서 직선으로 왕복시키는 IK 예제 (고무줄로 매단 상태에서 실행).

목표 경로 (pelvis 기준): (0.30, 0.30, 0.05) ↔ (0.30, 0.05, 0.05)  = 물체를 옆으로 미는 동작
매 주기 IK 로 관절각을 구해 R1LowLevel 로 보내고, 측정 관절각으로 FK 를 계산해 실제 손 끝 오차를 본다.
중력 보상(tau_ff)과 속도 피드포워드(dq_ff)를 함께 보낸다 (docs/kinematics.md 의 오차 분석 참고).

    bash sim/run_r1_sim.sh                        # 터미널 1
    source env/setup_sim.sh                       # 터미널 2
    python -m r1.kinematics.examples.reach_line
"""

import argparse
import time

import numpy as np

from r1.interface.joints import NUM_SIM_MOTORS, WAIST
from r1.interface.robot import R1LowLevel, finish, wait
from r1.kinematics.arm_ik import ArmIK

START = np.array([0.30, 0.30, 0.05])
END = np.array([0.30, 0.05, 0.05])


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--domain", type=int, default=1)
    p.add_argument("--iface", default="lo")
    p.add_argument("--damp-on-exit", action="store_true", help="정상 종료 때도 damp (실기 권장). 기본: 마지막 자세 유지")
    p.add_argument("--cycles", type=int, default=2)
    p.add_argument("--period", type=float, default=4.0, help="왕복 1회 시간 [s]")
    args = p.parse_args()

    ik = ArmIK()
    # 중력 보상: 양팔 + 허리. 허리를 빼면 팔 무게로 허리 roll 이 ~0.05 rad 처져 손 끝이 ~16 mm 밀린다
    comp = ik.arm_motors("left") + ik.arm_motors("right") + WAIST
    robot = R1LowLevel(domain_id=args.domain, interface=args.iface)
    robot.start()
    print("connected. Ctrl+C = damp")

    try:
        q_cmd = np.zeros(NUM_SIM_MOTORS)
        q_cmd, ok, err = ik.solve(START, q_cmd, "left")
        print(f"start pose IK ok={ok} err={err * 1000:.1f} mm")
        robot.set_target(q_cmd, tau_ff=ik.gravity_torque(q_cmd, comp))
        wait(4.0, robot)
        print(f"static tip error: {np.linalg.norm(ik.fk(robot.q(), 'left') - START) * 1000:.1f} mm")

        tip_err = []
        t0 = t_prev = time.time()
        q_prev = q_cmd.copy()
        while time.time() - t0 < args.cycles * args.period and not robot.damped:
            now = time.time()
            s = 0.5 - 0.5 * np.cos(2 * np.pi * (now - t0) / args.period)  # 0→1→0 부드럽게
            target = START + s * (END - START)
            q_cmd, ok, _ = ik.solve(target, q_cmd, "left", restarts=0)  # 이전 해에서 이어서 풀기
            dq_ff = (q_cmd - q_prev) / max(now - t_prev, 1e-3)
            q_prev, t_prev = q_cmd.copy(), now
            robot.set_target(q_cmd, tau_ff=ik.gravity_torque(robot.q(), comp), dq_ff=dq_ff)
            tip_err.append(np.linalg.norm(ik.fk(robot.q(), "left") - target))
            time.sleep(0.01)

        e = np.array(tip_err) * 1000
        print(f"moving tip error (measured FK vs target): mean {e.mean():.1f} mm, max {e.max():.1f} mm")

        robot.set_target(np.zeros(NUM_SIM_MOTORS), tau_ff=np.zeros(NUM_SIM_MOTORS))
        wait(3.0, robot)
    except KeyboardInterrupt:
        robot.damp()
        time.sleep(0.5)
        print("\nCtrl+C -> damped")
        return
    except Exception:
        robot.damp()
        time.sleep(0.5)
        raise
    finish(robot, damp=args.damp_on_exit)


if __name__ == "__main__":
    main()
