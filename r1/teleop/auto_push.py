"""자동 시연: 시뮬 상자 위치를 보고 스크립트가 상자를 밀며 LeRobot 데이터셋을 기록한다.

키보드 시연(record_push.py)과 같은 경로를 쓴다: DDS 제어(R1LowLevel) + ROS2 카메라 + EpisodeRecorder.
사람 수고 없이 파이프라인(데이터 → 학습 → 실행)을 검증하기 위한 것. 사람 시연보다 동작이 일정하다.

    source env/setup_sim.sh
    python -m r1.sim.run --headless --ros --cam-hz 30 --cam-size 320x240     # 터미널 1
    python -m r1.teleop.auto_push --episodes 50                              # 터미널 2

다양성: 정렬 위치(±0.8 cm), 접근 여유, 밀기 높이, 밀기 끝 위치, 속도(0.07~0.11 m/s)를 에피소드마다 무작위로.
시작 전 멈춤은 기본 0: 멈춤 구간이 있으면 정책이 '준비 자세 = 정지'를 배워 출발하지 못한다 (docs/training.md 5절).
기본은 성공한 에피소드만 저장 (--keep-failures 로 실패도 저장).
"""

import argparse
import time
from pathlib import Path

import numpy as np

from r1.interface.joints import JOINT_NAMES, NUM_SIM_MOTORS
from r1.interface.robot import R1LowLevel, finish
from r1.kinematics.arm_ik import ArmIK
from r1.kinematics.motion import CartesianArm
from r1.teleop.dataset import EpisodeRecorder
from r1.teleop.record_push import READY_TIP, TASK, WORKSPACE_HI, WORKSPACE_LO, RosIO

WS_ROOT = Path(__file__).resolve().parents[2]
BOX_HALF, FIST_HALF = 0.025, 0.04
PUSH_Z, SAFE_Z = 0.005, 0.08


def expert_waypoints(
    box_xy, target_xy, rng, max_pause: float = 0.0, correct_x: bool = False
) -> tuple[list[np.ndarray], float, float]:
    """상자 왼쪽 위 → 내리기 → 오른쪽으로 밀기 → 들기. (웨이포인트, 속도, 시작 전 멈춤 0 ~ max_pause)

    correct_x: 밀기 끝의 x 를 목표 x 쪽으로 옮겨 비스듬히 민다 (상자가 목표보다 몸 쪽/바깥에 있을 때 보정).
               False 면 y 방향으로만 민다 (v1·v2 데이터).
    """
    bx, by = box_xy
    x = bx + rng.uniform(-0.008, 0.008)
    contact_y = by + BOX_HALF + FIST_HALF + rng.uniform(0.01, 0.03)
    push_z = PUSH_Z + rng.uniform(0.0, 0.008)
    end_y = target_xy[1] + BOX_HALF + FIST_HALF + rng.uniform(-0.01, 0.01)
    end_x = x + (target_xy[0] - bx) if correct_x else x
    wps = [
        np.array([x, contact_y, SAFE_Z]),
        np.array([x, contact_y, push_z]),
        np.array([end_x, end_y, push_z]),
        np.array([end_x, end_y, SAFE_Z]),
    ]
    wps = [np.clip(w, WORKSPACE_LO, WORKSPACE_HI) for w in wps]
    return wps, rng.uniform(0.07, 0.11), rng.uniform(0.0, max_pause)


def targets_along(start, waypoints, speed, period):
    """웨이포인트를 잇는 손 끝 목표를 주기마다 하나씩 (구간마다 출발·도착 부드럽게)."""
    p0 = np.asarray(start, dtype=float)
    for wp in waypoints:
        n = max(int(np.linalg.norm(wp - p0) / speed / period), 1)
        for k in range(1, n + 1):
            s = 0.5 - 0.5 * np.cos(np.pi * k / n)
            yield p0 + s * (wp - p0)
        p0 = wp


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--domain", type=int, default=1)
    p.add_argument("--iface", default="lo")
    p.add_argument("--name", default="r1_push_box_auto", help="데이터셋 이름 (data/datasets/<name>)")
    p.add_argument("--episodes", type=int, default=50, help="저장할 에피소드 수")
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--keep-failures", action="store_true")
    p.add_argument("--max-start-pause", type=float, default=0.0, help="시작 전 멈춤 최대 [s] (v1 데이터는 0.2~0.6)")
    p.add_argument("--end-hold", type=float, default=0.3, help="끝에서 멈춰 있는 시간 [s]")
    p.add_argument("--correct-x", action="store_true", help="비스듬히 밀어 상자 x 를 목표 x 로 보정 (v3 데이터)")
    p.add_argument("--image-topic", default="/camera/head/image_raw")
    args = p.parse_args()

    rng = np.random.default_rng(args.seed)
    ik = ArmIK()
    arm_idx = ik.arm_motors("left")
    names = [JOINT_NAMES[i] for i in arm_idx]
    period = 1.0 / args.fps

    ros = RosIO(args.image_topic)
    robot = R1LowLevel(domain_id=args.domain, interface=args.iface)
    robot.start()
    arm = CartesianArm(robot, ik, "left", dt=period)

    t0 = time.time()
    while ros.latest_image() is None:
        if time.time() - t0 > 10:
            raise SystemExit(f"{args.image_topic} 가 들어오지 않음. 시뮬을 --ros 로 실행했는지 확인.")
        time.sleep(0.1)
    h, w, _ = ros.latest_image().shape
    rec = EpisodeRecorder(
        WS_ROOT / "data" / "datasets" / args.name, fps=args.fps,
        state_names=names, action_names=names, image_shapes={"head": (h, w, 3)}, task=TASK,
    )

    saved = tried = 0
    try:
        while saved < args.episodes:
            tried += 1
            arm.reset(np.zeros(NUM_SIM_MOTORS))
            info = ros.reset_sim()
            if info is None:
                raise SystemExit("/r1_sim/reset 실패")
            time.sleep(0.2)
            arm.move_to(READY_TIP, speed=0.15)
            time.sleep(0.3)  # 카메라·상자 위치가 리셋 후 값으로 갱신되도록

            wps, speed, pause = expert_waypoints(info["box_xy"], info["target_xy"], rng, args.max_start_pause, args.correct_x)
            plan = [READY_TIP] * int(pause / period) + list(targets_along(READY_TIP, wps, speed, period))
            plan += [wps[-1]] * int(args.end_hold / period)  # 끝에서 잠시 멈춤

            next_t = time.time()
            for target in plan:
                arm.servo(target, dt=period)
                img = ros.latest_image()
                if img is not None:
                    rec.add(robot.q()[arm_idx], arm.q_cmd[arm_idx], {"head": img})
                next_t += period
                time.sleep(max(0.0, next_t - time.time()))

            time.sleep(0.2)
            box = ros.latest_box()
            dist = float(np.linalg.norm(box[:2] - np.array(info["target_xy"])))
            success = dist < info["target_radius"]
            log = {"box_start": info["box_xy"], "target_xy": info["target_xy"], "target_radius": info["target_radius"],
                   "box_end": box[:2].round(4).tolist(), "dist_to_target": round(dist, 4), "success": success,
                   "source": "auto_push", "speed": round(speed, 3), "start_pause": round(pause, 3), "correct_x": args.correct_x}
            if success or args.keep_failures:
                n = rec.frames
                rec.save(log)
                saved += 1
                print(f"[auto] try {tried:3d}: saved ep {rec.num_episodes - 1:3d} ({n} frames) dist {dist:.3f} {'OK' if success else 'FAIL'}", flush=True)
            else:
                rec.discard()
                print(f"[auto] try {tried:3d}: discarded (dist {dist:.3f})", flush=True)
    except KeyboardInterrupt:
        robot.damp()
        print("\nCtrl+C -> damped")
    finally:
        rec.close()
        ros.close()
        print(f"[auto] {saved} saved / {tried} tried -> {rec.root}")
    finish(robot, damp=False)


if __name__ == "__main__":
    main()
