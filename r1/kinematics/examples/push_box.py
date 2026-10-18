"""왼손으로 상자를 목표 영역(초록)까지 미는 데모. r1.sim.run 의 기본 장면용.

장면 기본값 (r1/sim/scene.py, 골반 기준): 테이블 윗면 z = -0.05, 상자 (0.27, 0.22) 한 변 0.05, 목표 (0.27, 0.00)
상자 위치는 장면 기본값을 그대로 쓴다 (인식 없음). 나중에 원격조작·학습 정책이 이 부분을 대신한다.

    source env/setup_sim.sh
    python -m r1.sim.run --print-box                # 터미널 1 (상자 위치 출력)
    python -m r1.kinematics.examples.push_box       # 터미널 2
"""

import argparse
import time

import numpy as np

from r1.interface.robot import R1LowLevel, finish, wait
from r1.kinematics.arm_ik import ArmIK
from r1.kinematics.motion import CartesianArm

BOX = np.array([0.27, 0.22])
TARGET = np.array([0.27, 0.00])
TABLE_Z = -0.05      # 테이블 윗면 (골반 기준)
FIST_HALF = 0.04     # 주먹 반폭 (손 끝 중심 → 측면)
BOX_HALF = 0.025
# 손을 내리면 주먹이 아래로 기울어 앞쪽 아랫부분이 먼저 닿는다. +0.04 에서는 테이블과 접촉,
# +0.055 에서 경로 전체 최소 여유 17 mm (MuJoCo mj_geomDistance 로 확인). 주먹 아래 ~3 cm 가 상자에 걸린다.
PUSH_Z = TABLE_Z + 0.055
SAFE_Z = 0.08


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--domain", type=int, default=1)
    p.add_argument("--iface", default="lo")
    p.add_argument("--damp-on-exit", action="store_true", help="정상 종료 때도 damp (실기 권장). 기본: 마지막 자세 유지")
    p.add_argument("--speed", type=float, default=0.10, help="손 끝 속도 [m/s]")
    args = p.parse_args()

    ik = ArmIK()
    robot = R1LowLevel(domain_id=args.domain, interface=args.iface)
    robot.start()
    arm = CartesianArm(robot, ik, "left")
    print("connected. Ctrl+C = damp")

    contact_y = BOX[1] + BOX_HALF + FIST_HALF + 0.02   # 상자 왼쪽에서 2 cm 떨어진 곳
    end_y = TARGET[1] + BOX_HALF + FIST_HALF            # 상자 중심이 목표에 오도록
    waypoints = [
        ("above left of box", [BOX[0], contact_y, SAFE_Z]),
        ("lower", [BOX[0], contact_y, PUSH_Z]),
        ("push", [BOX[0], end_y, PUSH_Z]),
        ("lift", [BOX[0], end_y, SAFE_Z]),
    ]
    try:
        for name, wp in waypoints:
            err = arm.move_to(wp, speed=args.speed)
            print(f"{name:18s} -> {np.round(wp, 3)}  tip err {err * 1000:.1f} mm")
        robot.set_target(np.zeros(29), tau_ff=np.zeros(29))
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
