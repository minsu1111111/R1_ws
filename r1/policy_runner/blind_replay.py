"""기준선(baseline): 시연 1개의 관절 명령을 상자 위치와 무관하게 그대로 재생한다 (카메라를 보지 않음).

학습된 정책이 이 기준선보다 성공률이 높아야 '카메라로 상자를 보고 행동을 바꾼다'고 말할 수 있다.
정책 평가(run_act.py)와 같은 시뮬 시드·에피소드 수로 돌려 비교한다 (docs/training.md).

    python -m r1.sim.run --headless --ros --cam-hz 30 --cam-size 320x240 --seed 123     # 터미널 1
    python -m r1.policy_runner.blind_replay --dataset r1_push_box_auto --episodes 30    # 터미널 2
"""

import argparse
import json
import time
from pathlib import Path

import numpy as np
from lerobot.datasets.lerobot_dataset import LeRobotDataset

from r1.interface.joints import NUM_SIM_MOTORS
from r1.interface.robot import R1LowLevel, finish
from r1.kinematics.arm_ik import ArmIK
from r1.kinematics.motion import CartesianArm
from r1.teleop.record_push import READY_TIP, RosIO

WS_ROOT = Path(__file__).resolve().parents[2]


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--dataset", default="r1_push_box_auto")
    p.add_argument("--demo", type=int, default=0, help="재생할 시연 에피소드 번호")
    p.add_argument("--episodes", type=int, default=30)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--domain", type=int, default=1)
    p.add_argument("--iface", default="lo")
    p.add_argument("--log", help="결과 jsonl")
    args = p.parse_args()

    ds = LeRobotDataset(f"local/{args.dataset}", root=WS_ROOT / "data" / "datasets" / args.dataset)
    ep = ds.meta.episodes[args.demo]
    a0, a1 = int(ep["dataset_from_index"]), int(ep["dataset_to_index"])
    actions = np.stack([ds.hf_dataset[i]["action"].numpy() for i in range(a0, a1)])
    print(f"[blind] replaying demo {args.demo} ({len(actions)} frames) of {args.dataset}")

    ik = ArmIK()
    arm_idx = ik.arm_motors("left")
    period = 1.0 / args.fps
    ros = RosIO("/camera/head/image_raw")
    robot = R1LowLevel(domain_id=args.domain, interface=args.iface)
    robot.start()
    arm = CartesianArm(robot, ik, "left", dt=period)

    results = []
    try:
        for k in range(args.episodes):
            arm.reset(np.zeros(NUM_SIM_MOTORS))
            info = ros.reset_sim()
            time.sleep(0.2)
            arm.move_to(READY_TIP, speed=0.15)
            time.sleep(0.3)
            next_t = time.time()
            for a in list(actions) + [actions[-1]] * int(1.0 / period):
                q = arm.q_cmd.copy()
                q[arm_idx] = a
                arm.q_cmd = q
                robot.set_target(q, tau_ff=ik.gravity_torque(robot.q(), arm.comp))
                next_t += period
                time.sleep(max(0.0, next_t - time.time()))
            time.sleep(0.3)
            box = ros.latest_box()
            dist = float(np.linalg.norm(box[:2] - np.array(info["target_xy"])))
            ok = dist < info["target_radius"]
            results.append({"episode": k, "box_start": info["box_xy"], "box_end": box[:2].round(4).tolist(),
                            "dist_to_target": round(dist, 4), "success": ok})
            print(f"[blind] ep {k:2d}: box {np.round(info['box_xy'], 3)} -> dist {dist:.3f} "
                  f"{'SUCCESS' if ok else 'fail'}", flush=True)
    except KeyboardInterrupt:
        robot.damp()
        print("\nCtrl+C -> damped")
    finally:
        ros.close()
    if results:
        n_ok = sum(r["success"] for r in results)
        print(f"[blind] success {n_ok}/{len(results)} = {n_ok / len(results):.0%}")
        if args.log:
            with open(args.log, "w") as f:
                f.writelines(json.dumps(r) + "\n" for r in results)
    finish(robot, damp=False)


if __name__ == "__main__":
    main()
