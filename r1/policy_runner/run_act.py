"""학습된 ACT 정책을 시뮬(또는 실기)에서 실행하고 상자 밀기 성공률을 잰다.

    source env/setup_sim.sh
    python -m r1.sim.run --ros --cam-hz 30 --cam-size 320x240                         # 터미널 1
    python -m r1.policy_runner.run_act --ckpt data/checkpoints/<RUN>/checkpoints/last/pretrained_model   # 터미널 2

에피소드마다: 시뮬 리셋(상자 무작위) → 손을 준비 위치로 → 정책을 30 Hz 로 실행(--max-sec) → 상자-목표 거리로 성공 판정.
관측·행동 형식은 기록 때와 같다 (docs/teleop.md): 머리 카메라 + 왼팔 관절 5개 → 왼팔 관절 명령 5개.
정책 출력에도 중력 보상·속도 피드포워드·관절 범위·속도 제한(R1LowLevel)이 그대로 적용된다.
"""

import argparse
import json
import time

import numpy as np
import torch
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.factory import make_pre_post_processors
from lerobot.utils.control_utils import predict_action

from r1.interface.joints import NUM_SIM_MOTORS
from r1.interface.robot import R1LowLevel, finish
from r1.kinematics.arm_ik import ArmIK
from r1.kinematics.motion import CartesianArm
from r1.teleop.record_push import READY_TIP, TASK, RosIO


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--ckpt", required=True, help=".../checkpoints/<step|last>/pretrained_model")
    p.add_argument("--episodes", type=int, default=10)
    p.add_argument("--max-sec", type=float, default=8.0, help="에피소드당 정책 실행 시간")
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--domain", type=int, default=1)
    p.add_argument("--iface", default="lo")
    p.add_argument("--device", default="cuda")
    p.add_argument("--image-topic", default="/camera/head/image_raw")
    p.add_argument("--log", help="에피소드 결과를 jsonl 로 저장")
    p.add_argument("--n-action-steps", type=int, help="예측한 묶음 중 실행할 스텝 수 (학습 설정 덮어쓰기, 최대 chunk_size)")
    args = p.parse_args()

    device = torch.device(args.device)
    policy = ACTPolicy.from_pretrained(args.ckpt)
    if args.n_action_steps:
        policy.config.n_action_steps = min(args.n_action_steps, policy.config.chunk_size)
    policy.to(device).eval()
    pre, post = make_pre_post_processors(
        policy.config, pretrained_path=args.ckpt, preprocessor_overrides={"device_processor": {"device": args.device}}
    )
    print(f"[act] loaded {args.ckpt} (chunk {policy.config.chunk_size}, n_action_steps {policy.config.n_action_steps})")

    ik = ArmIK()
    arm_idx = ik.arm_motors("left")
    period = 1.0 / args.fps
    ros = RosIO(args.image_topic)
    robot = R1LowLevel(domain_id=args.domain, interface=args.iface)
    robot.start()
    arm = CartesianArm(robot, ik, "left", dt=period)
    while ros.latest_image() is None:
        time.sleep(0.1)

    results = []
    try:
        for ep in range(args.episodes):
            arm.reset(np.zeros(NUM_SIM_MOTORS))
            info = ros.reset_sim()
            time.sleep(0.2)
            arm.move_to(READY_TIP, speed=0.15)
            time.sleep(0.3)
            policy.reset()
            pre.reset()
            post.reset()

            min_dist, infer_ms = 1.0, []
            next_t = t0 = time.time()
            while time.time() - t0 < args.max_sec:
                img = ros.latest_image()
                q_meas = robot.q()
                obs = {"observation.state": q_meas[arm_idx].astype(np.float32), "observation.images.head": img}
                a = time.perf_counter()
                action = predict_action(obs, policy, device, pre, post, use_amp=False, task=TASK, robot_type="unitree_r1")
                infer_ms.append((time.perf_counter() - a) * 1000)

                q_cmd = arm.q_cmd.copy()
                q_cmd[arm_idx] = action.cpu().numpy()
                dq_ff = (q_cmd - arm.q_cmd) / period
                arm.q_cmd = q_cmd
                robot.set_target(q_cmd, tau_ff=ik.gravity_torque(q_meas, arm.comp), dq_ff=dq_ff)

                box = ros.latest_box()
                if box is not None:
                    min_dist = min(min_dist, float(np.linalg.norm(box[:2] - np.array(info["target_xy"]))))
                next_t += period
                time.sleep(max(0.0, next_t - time.time()))

            box = ros.latest_box()
            dist = float(np.linalg.norm(box[:2] - np.array(info["target_xy"])))
            ok = dist < info["target_radius"]
            r = {"episode": ep, "box_start": info["box_xy"], "box_end": box[:2].round(4).tolist(),
                 "dist_to_target": round(dist, 4), "min_dist": round(min_dist, 4), "success": ok,
                 "infer_ms_p50": round(float(np.median(infer_ms)), 1), "infer_ms_max": round(float(np.max(infer_ms)), 1)}
            results.append(r)
            print(f"[act] ep {ep:2d}: box {np.round(info['box_xy'], 3)} -> {np.round(box[:2], 3)}  "
                  f"dist {dist:.3f} (min {min_dist:.3f})  {'SUCCESS' if ok else 'fail'}  "
                  f"infer p50 {r['infer_ms_p50']} ms max {r['infer_ms_max']} ms", flush=True)
    except KeyboardInterrupt:
        robot.damp()
        print("\nCtrl+C -> damped")
    finally:
        ros.close()
    if results:
        n_ok = sum(r["success"] for r in results)
        print(f"[act] success {n_ok}/{len(results)} = {n_ok / len(results):.0%}")
        if args.log:
            with open(args.log, "w") as f:
                for r in results:
                    f.write(json.dumps(r) + "\n")
    finish(robot, damp=False)


if __name__ == "__main__":
    main()
