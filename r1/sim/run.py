"""R1 Python 시뮬 (조작 장면 + DDS). 실기와 같은 토픽을 쓰므로 r1.interface 코드가 그대로 동작한다.

    source env/setup_sim.sh
    python -m r1.sim.run                       # 골반 고정, 테이블·상자·카메라, 뷰어
    python -m r1.sim.run --base band           # 고무줄로 매단 상태 (C++ 시뮬과 같은 조건)
    python -m r1.sim.run --headless            # 뷰어 없이 (속도 측정, 서버)
    python -m r1.sim.run --ros-cameras         # 카메라 영상을 ROS2 로 (/camera/head|front/image_raw)

실시간 루프: 시뮬 시간이 실제 시간을 따라가도록 물리 스텝을 돌리고, 2초마다 실시간 비율(RTF)을 출력한다.
LowState.tick 은 시뮬 시간[ms] 이다.

첫 LowCmd 를 받기 전까지는 기본 자세(모든 관절 0)를 PD 로 유지한다 (팔이 테이블로 떨어지지 않도록).

뷰어 키:  Tab / Shift+Tab 좌우 패널 | 마우스 드래그 회전, 우클릭 드래그 이동, 휠 확대 | Backspace 리셋 | 9 고무줄 켜기/끄기 (--base band) | 7/8 고무줄 짧게/길게
"""

import argparse
import os
import sys
import threading
import time

# 화면 없이(--headless) 카메라를 렌더링할 때는 EGL (mujoco import 전에 정해야 한다)
if "--headless" in sys.argv:
    os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import mujoco.viewer
import numpy as np

from r1.sim.bridge import SimBridge
from r1.sim.scene import SceneConfig, build_model

KEY_BACKSPACE = 259


class ElasticBand:
    """C++ 시뮬(main.cc)과 같은 스프링: torso_link 를 (0, 0, 3) 에 매단다."""

    def __init__(self, model):
        self.body = model.body("torso_link").id
        self.point = np.array([0.0, 0.0, 3.0])
        self.stiffness, self.damping = 200.0, 100.0
        self.length = 0.0
        self.enable = True

    def apply(self, d):
        d.xfrc_applied[self.body, :3] = 0.0
        if not self.enable:
            return
        x = d.xpos[self.body]
        v = d.cvel[self.body, 3:]  # 근사: 선속도
        delta = self.point - x
        dist = np.linalg.norm(delta)
        direction = delta / dist
        f = (self.stiffness * (dist - self.length) - self.damping * float(v @ direction)) * direction
        d.xfrc_applied[self.body, :3] = f


def arm_contacts(m, d) -> set[str]:
    """팔 링크가 닿은 상대 (다리끼리·발-바닥 접촉은 제외)."""
    out = set()
    for c in d.contact[: d.ncon]:
        names = [m.body(m.geom_bodyid[g]).name or m.geom(g).name for g in (c.geom1, c.geom2)]
        names = [m.geom(g).name or nm for g, nm in zip((c.geom1, c.geom2), names)]
        if any(k in "".join(names) for k in ("shoulder", "elbow", "wrist")):
            out.add("<->".join(sorted(names)))
    return out


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--base", default="fixed", choices=["fixed", "band", "free"])
    p.add_argument("--domain", type=int, default=1)
    p.add_argument("--iface", default="lo")
    p.add_argument("--headless", action="store_true")
    p.add_argument("--ui", action="store_true", help="뷰어 좌우 설정 패널 표시 (기본: 숨김, Tab / Shift+Tab 으로 토글)")
    p.add_argument("--no-table", action="store_true")
    p.add_argument("--print-box", action="store_true", help="2초마다 상자 위치 출력")
    p.add_argument("--ros-cameras", action="store_true", help="head_cam, front_cam 영상을 ROS2 토픽으로 발행")
    p.add_argument("--cam-hz", type=float, default=15.0, help="ROS2 카메라 발행 주기 [Hz]")
    p.add_argument("--cam-size", default="640x480", help="카메라 해상도 WxH")
    p.add_argument("--record", metavar="OUT.mp4", help="카메라 영상을 mp4 로 저장")
    p.add_argument("--record-cam", default="front_cam", help="front_cam | head_cam")
    p.add_argument("--record-fps", type=float, default=20.0)
    p.add_argument("--print-contacts", action="store_true", help="로봇 팔과 테이블·상자·몸통의 접촉을 바뀔 때마다 출력")
    args = p.parse_args()

    m = build_model(SceneConfig(base=args.base, table=not args.no_table))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    bridge = SimBridge(m, args.domain, args.iface)
    band = ElasticBand(m) if args.base == "band" else None
    lock = threading.Lock()

    def on_key(key):
        with lock:
            if key == KEY_BACKSPACE:
                mujoco.mj_resetData(m, d)
                mujoco.mj_forward(m, d)
            elif band and key == ord("9"):
                band.enable = not band.enable
            elif band and key == ord("7"):
                band.length -= 0.1
            elif band and key == ord("8"):
                band.length += 0.1

    viewer = None
    if not args.headless:
        viewer = mujoco.viewer.launch_passive(
            m, d, key_callback=on_key, show_left_ui=args.ui, show_right_ui=args.ui
        )
        with viewer.lock():  # 로봇 전신 + 테이블이 한 화면에 들어오는 시작 시점
            viewer.cam.lookat[:] = [0.15, 0.0, 0.70]
            viewer.cam.distance = 2.1
            viewer.cam.azimuth = 200.0
            viewer.cam.elevation = -18.0
    # 오프스크린 렌더러: 녹화와 ROS2 카메라가 함께 쓴다. 뷰어와 같은 프로세스에서 동작 확인 (종료 시 먼저 닫는다).
    cam_w, cam_h = (int(x) for x in args.cam_size.split("x"))
    renderer = recorder = ros_cams = None
    if args.record or args.ros_cameras:
        renderer = mujoco.Renderer(m, cam_h, cam_w)
    if args.record:
        import cv2

        recorder = cv2.VideoWriter(args.record, cv2.VideoWriter_fourcc(*"mp4v"), args.record_fps, (cam_w, cam_h))
    if args.ros_cameras:
        from r1.sim.ros_camera import RosCameraPublisher

        cams = {name: float(m.cam_fovy[m.camera(name).id]) for name in ("head_cam", "front_cam")}
        ros_cams = RosCameraPublisher(cams, cam_w, cam_h)
        print(f"[r1 sim] ROS2 cameras {list(cams)} {cam_w}x{cam_h} @ {args.cam_hz} Hz -> /camera/<head|front>/image_raw")
    last_frame = last_cam = -1.0
    dt = m.opt.timestep
    print(f"[r1 sim] base={args.base} dt={dt * 1000:.1f} ms domain={args.domain} if={args.iface}")

    wall0, sim0 = time.perf_counter(), d.time
    rtf_wall, rtf_sim = wall0, d.time
    last_sync = 0.0
    box_id = m.body("box").id if not args.no_table else None
    last_contacts, window_contacts, window_t = set(), set(), 0.0
    try:
        while viewer is None or viewer.is_running():
            now = time.perf_counter()
            with lock:
                # 시뮬 시간이 실제 시간을 따라잡을 때까지 스텝 (최대 20 스텝 = 40 ms 씩)
                n = 0
                while d.time - sim0 < now - wall0 and n < 20:
                    d.ctrl[:] = bridge.compute_ctrl(d)
                    if band:
                        band.apply(d)
                    mujoco.mj_step(m, d)
                    bridge.publish_state(d)
                    n += 1
                if n == 20:  # 따라잡지 못하면 기준을 다시 잡는다 (느려진 만큼 RTF 로 보인다)
                    wall0, sim0 = now, d.time
                if args.print_contacts:  # 0.5 s 동안 생긴 접촉을 모아, 이전 구간과 다를 때만 출력
                    window_contacts |= arm_contacts(m, d)
                    if d.time - window_t >= 0.5:
                        if window_contacts != last_contacts:
                            print(f"[r1 sim] t={d.time:5.1f}s contacts: {sorted(window_contacts) or 'none'}", flush=True)
                        last_contacts, window_contacts, window_t = window_contacts, set(), d.time
                if recorder is not None and d.time - last_frame >= 1 / args.record_fps:
                    renderer.update_scene(d, camera=args.record_cam)
                    recorder.write(renderer.render()[:, :, ::-1])
                    last_frame = d.time
                if ros_cams is not None and d.time - last_cam >= 1 / args.cam_hz:
                    for cam in ("head_cam", "front_cam"):
                        renderer.update_scene(d, camera=cam)
                        ros_cams.publish(cam, renderer.render())
                    last_cam = d.time
                if viewer is not None and now - last_sync > 1 / 60:
                    viewer.sync()
                    last_sync = now

            if now - rtf_wall > 2.0:
                msg = f"[r1 sim] RTF {(d.time - rtf_sim) / (now - rtf_wall):.2f}  cmds {bridge.cmd_count}"
                if args.print_box and box_id is not None:
                    msg += f"  box {np.round(d.xpos[box_id], 3)}"
                print(msg, flush=True)
                rtf_wall, rtf_sim = now, d.time
            time.sleep(0.0005)
    except KeyboardInterrupt:
        pass
    finally:
        if recorder is not None:
            recorder.release()
            print(f"[r1 sim] saved {args.record}")
        if ros_cams is not None:
            ros_cams.close()
        if renderer is not None:  # 뷰어보다 먼저 닫아야 종료 시 core dump 가 나지 않는다
            renderer.close()
        if viewer is not None:
            viewer.close()


if __name__ == "__main__":
    main()
