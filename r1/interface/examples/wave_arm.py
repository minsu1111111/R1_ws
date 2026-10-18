"""R1 팔 흔들기 예제 (고무줄로 매단 상태에서 실행).

1) 현재 자세 → 기본 자세(모든 관절 0)로 천천히 이동
2) 왼팔을 들고 팔꿈치를 흔든다
3) 기본 자세로 돌아온 뒤 damp

    bash sim/run_r1_sim.sh                        # 터미널 1
    source env/setup_sim.sh                       # 터미널 2
    python -m r1.interface.examples.wave_arm

Ctrl+C → 즉시 damp (비상정지).
"""

import argparse
import time

import numpy as np

from r1.interface.joints import JOINT_INDEX, NUM_SIM_MOTORS
from r1.interface.robot import R1LowLevel, wait

J = JOINT_INDEX


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--domain", type=int, default=1)
    p.add_argument("--iface", default="lo")
    p.add_argument("--variant", default="a5", choices=["a5", "a7"])
    p.add_argument("--wave-sec", type=float, default=8.0)
    args = p.parse_args()

    robot = R1LowLevel(domain_id=args.domain, interface=args.iface, variant=args.variant)
    robot.start()
    print("connected. Ctrl+C = damp")

    try:
        home = np.zeros(NUM_SIM_MOTORS)
        robot.set_target(home)
        wait(4.0, robot)  # 변화율 제한(1 rad/s) 때문에 큰 관절도 4초 안에 도착
        print(f"home      max|err| = {np.abs(robot.tracking_error()).max():.3f} rad")

        raised = home.copy()
        raised[J["left_shoulder_roll"]] = 1.2
        raised[J["left_shoulder_pitch"]] = -0.5
        raised[J["left_elbow"]] = 1.0
        robot.set_target(raised)
        wait(2.0, robot)

        t0 = time.time()
        while time.time() - t0 < args.wave_sec and not robot.damped:
            t = time.time() - t0
            q = raised.copy()
            q[J["left_elbow"]] = 1.0 + 0.5 * np.sin(2 * np.pi * 0.5 * t)
            q[J["left_wrist_roll"]] = 0.6 * np.sin(2 * np.pi * 0.5 * t)
            robot.set_target(q)
            time.sleep(0.01)
        print(f"wave      max|err| = {np.abs(robot.tracking_error()).max():.3f} rad")

        robot.set_target(home)
        wait(3.0, robot)
        print(f"home      max|err| = {np.abs(robot.tracking_error()).max():.3f} rad")
    except KeyboardInterrupt:
        print("\nCtrl+C")
    finally:
        robot.damp()
        time.sleep(0.5)
        print("damped")


if __name__ == "__main__":
    main()
