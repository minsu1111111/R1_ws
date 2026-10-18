"""키보드 원격조작으로 '상자 밀기' 시연을 LeRobot 데이터셋으로 기록한다.

    source env/setup_sim.sh
    python -m r1.sim.run --ros --cam-hz 30 --cam-size 320x240     # 터미널 1: 시뮬 (카메라 30 Hz, 320x240)
    python -m r1.teleop.record_push                              # 터미널 2: 이 프로그램 (이 터미널에서 키 입력)

키 (이 터미널에 포커스가 있어야 한다)
    w / s   손 앞 / 뒤 (x)          a / d   손 왼쪽 / 오른쪽 (y)          r / f   손 위 / 아래 (z)
    n       새 에피소드: 시뮬 리셋(상자 무작위) + 손을 준비 위치로
    space   기록 시작
    Enter   기록 끝 + 저장             x       기록 끝 + 버리기
    q       종료 (데이터셋 마무리)       Ctrl+C  비상정지(damp) 후 종료

기록 내용 (30 Hz)
    observation.images.head : 머리 카메라 (/camera/head/image_raw)
    observation.state       : 왼팔 관절 5개 측정값 [rad]
    action                  : 왼팔 관절 5개 명령값 [rad]  (IK 결과 = 정책이 배울 출력)
    task                    : "push the red box to the green circle"
상자 시작·끝 위치와 성공 여부는 <데이터셋>.episodes.jsonl 에 남긴다 (시뮬 전용 정답값).
"""

import argparse
import json
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

from r1.interface.joints import JOINT_NAMES, NUM_SIM_MOTORS
from r1.interface.robot import R1LowLevel
from r1.kinematics.arm_ik import ArmIK
from r1.kinematics.motion import CartesianArm
from r1.teleop.dataset import EpisodeRecorder
from r1.teleop.keyboard import KeyReader

WS_ROOT = Path(__file__).resolve().parents[2]
TASK = "push the red box to the green circle"

# 손 끝 작업 영역 (골반 기준). z 하한은 주먹이 테이블에 닿지 않는 높이 (docs/kinematics.md)
WORKSPACE_LO = np.array([0.15, -0.08, 0.005])
WORKSPACE_HI = np.array([0.34, 0.36, 0.12])
READY_TIP = np.array([0.24, 0.33, 0.08])   # 새 에피소드 시작 때 손 위치 (상자 범위 왼쪽 위)
SPEED = 0.08                               # 손 끝 이동 속도 [m/s]
MOVE_KEYS = {"w": (0, +1), "s": (0, -1), "a": (1, +1), "d": (1, -1), "r": (2, +1), "f": (2, -1)}


class RosIO:
    """카메라·상자 위치 구독, 리셋 서비스 호출. 별도 스레드에서 spin."""

    def __init__(self, image_topic: str):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("r1_teleop_record")
        self._lock = threading.Lock()
        self.image, self.image_t = None, 0.0
        self.box, self.box_t = None, 0.0
        self.node.create_subscription(Image, image_topic, self._on_image, 10)
        self.node.create_subscription(PoseStamped, "/r1_sim/box_pose", self._on_box, 10)
        self.reset_cli = self.node.create_client(Trigger, "/r1_sim/reset")
        self._ex = SingleThreadedExecutor()
        self._ex.add_node(self.node)
        threading.Thread(target=self._ex.spin, daemon=True).start()

    def _on_image(self, msg: Image):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        with self._lock:
            self.image, self.image_t = img, time.time()

    def _on_box(self, msg: PoseStamped):
        p = msg.pose.position
        with self._lock:
            self.box, self.box_t = np.array([p.x, p.y, p.z]), time.time()

    def latest_image(self, max_age: float = 0.2):
        with self._lock:
            if self.image is None or time.time() - self.image_t > max_age:
                return None
            return self.image.copy()

    def latest_box(self):
        with self._lock:
            return None if self.box is None else self.box.copy()

    def reset_sim(self, timeout: float = 5.0) -> dict | None:
        if not self.reset_cli.wait_for_service(timeout_sec=1.0):
            return None
        fut = self.reset_cli.call_async(Trigger.Request())
        end = time.time() + timeout
        while not fut.done() and time.time() < end:
            time.sleep(0.01)
        if not fut.done() or not fut.result().success:
            return None
        return json.loads(fut.result().message)

    def close(self):
        self._ex.shutdown()
        self.node.destroy_node()
        rclpy.try_shutdown()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--domain", type=int, default=1)
    p.add_argument("--iface", default="lo")
    p.add_argument("--name", default="r1_push_box", help="데이터셋 이름 (data/datasets/<name>)")
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--image-topic", default="/camera/head/image_raw")
    args = p.parse_args()

    ik = ArmIK()
    arm_idx = ik.arm_motors("left")
    names = [JOINT_NAMES[i] for i in arm_idx]

    ros = RosIO(args.image_topic)
    robot = R1LowLevel(domain_id=args.domain, interface=args.iface)
    robot.start()
    arm = CartesianArm(robot, ik, "left", dt=1.0 / args.fps)

    print("waiting for camera image ...")
    t0 = time.time()
    while ros.latest_image() is None:
        if time.time() - t0 > 10:
            sys.exit(f"{args.image_topic} 가 들어오지 않음. 시뮬을 --ros 로 실행했는지, ROS_DOMAIN_ID 가 같은지 확인.")
        time.sleep(0.1)
    h, w, _ = ros.latest_image().shape
    rec = EpisodeRecorder(
        WS_ROOT / "data" / "datasets" / args.name, fps=args.fps,
        state_names=names, action_names=names, image_shapes={"head": (h, w, 3)}, task=TASK,
    )
    print(__doc__.split("키 (")[1].split("기록 내용")[0])

    keys = KeyReader()
    recording, episode_info, tip_cmd = False, {}, ik.fk(arm.q_cmd, "left")
    last_status = 0.0

    def new_episode():
        nonlocal tip_cmd, episode_info
        home = np.zeros(NUM_SIM_MOTORS)
        arm.reset(home)                       # 명령을 먼저 기본 자세로 (리셋 직후 튀지 않게)
        info = ros.reset_sim()
        if info is None:
            print("\n[!] /r1_sim/reset 실패 (시뮬을 --ros 로 실행했나요?)")
            return
        time.sleep(0.2)
        arm.move_to(READY_TIP, speed=0.15)
        tip_cmd = READY_TIP.copy()
        episode_info = {"box_start": info.get("box_xy"), "target_xy": info["target_xy"],
                        "target_radius": info["target_radius"]}
        print(f"\n[ep] new episode: box {np.round(info.get('box_xy'), 3)} -> target {info['target_xy']}. space 로 기록 시작")

    def box_result():
        box = ros.latest_box()
        if box is None or "target_xy" not in episode_info:
            return {}
        dist = float(np.linalg.norm(box[:2] - np.array(episode_info["target_xy"])))
        return {"box_end": box[:2].round(4).tolist(), "dist_to_target": round(dist, 4),
                "success": dist < episode_info["target_radius"]}

    try:
        new_episode()
        period = 1.0 / args.fps
        next_t = time.time()
        while True:
            # ---------------- 명령 키
            for ch in keys.pop_events():
                if ch == "q":
                    raise StopIteration
                elif ch == "n" and not recording:
                    new_episode()
                elif ch == " " and not recording:
                    recording = True
                    print(f"\n[ep] recording episode {rec.num_episodes} ...")
                elif ch in ("\n", "\r") and recording:
                    recording = False
                    res = box_result()
                    print(f"\n[ep] saving {rec.frames} frames ... {res}")
                    rec.save({**episode_info, **res})
                    print(f"[ep] saved. total {rec.num_episodes} episodes. n 으로 다음 에피소드")
                elif ch == "x" and recording:
                    recording = False
                    rec.discard()
                    print("\n[ep] discarded. n 으로 다시")

            # ---------------- 손 이동
            v = np.zeros(3)
            for k, (axis, sign) in MOVE_KEYS.items():
                if keys.held(k):
                    v[axis] += sign
            tip_cmd = np.clip(tip_cmd + SPEED * period * v, WORKSPACE_LO, WORKSPACE_HI)
            if not arm.servo(tip_cmd, dt=period):
                tip_cmd = ik.fk(arm.q_cmd, "left")   # 닿지 않으면 마지막으로 닿은 곳에 머문다

            # ---------------- 기록
            if recording:
                img = ros.latest_image()
                if img is not None:
                    rec.add(robot.q()[arm_idx], arm.q_cmd[arm_idx], {"head": img})

            now = time.time()
            if now - last_status > 0.5:
                box = ros.latest_box()
                d = box_result().get("dist_to_target")
                state = f"REC {rec.frames:4d}" if recording else "idle    "
                print(f"\r{state} | tip {np.round(tip_cmd, 3)} | box {None if box is None else np.round(box[:2], 3)}"
                      f" | to target {d if d is None else f'{d:.3f}'}   ", end="", flush=True)
                last_status = now

            next_t += period
            time.sleep(max(0.0, next_t - time.time()))
            if time.time() - next_t > 0.5:  # 저장 등으로 크게 밀리면 기준을 다시 잡는다
                next_t = time.time()
    except StopIteration:
        print("\n[q] quit")
    except KeyboardInterrupt:
        robot.damp()
        print("\nCtrl+C -> damped")
    finally:
        keys.close()
        if recording:
            print("[ep] 기록 중이던 에피소드는 버립니다")
        rec.close()
        ros.close()
        print(f"dataset: {rec.root} ({rec.num_episodes} episodes), log: {rec.log_path}")


if __name__ == "__main__":
    main()
