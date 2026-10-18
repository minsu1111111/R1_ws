"""시뮬 ROS2 인터페이스 (python -m r1.sim.run --ros).

발행
  /camera/head|front/image_raw, camera_info   카메라 (ros_camera.py)
  /r1_sim/box_pose   (geometry_msgs/PoseStamped, frame pelvis_link)   상자 위치 — 시뮬 전용 정답값(성공 판정·기록용)
서비스
  /r1_sim/reset      (std_srvs/Trigger)   로봇 기본 자세 + 상자 무작위 위치. message 에 JSON {box_xy, target_xy, target_radius}

리셋은 시뮬 루프 스레드에서 실행해야 하므로, 서비스 콜백은 요청만 넣고 완료를 기다린다.
rclpy 는 별도 스레드에서 spin 한다.
"""

import json
import threading

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger

from r1.sim.ros_camera import RosCameraPublisher


class SimRosNode:
    def __init__(self, cameras: dict[str, float], width: int, height: int):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("r1_sim")
        self.cams = RosCameraPublisher(cameras, width, height, node=self.node)
        self.box_pub = self.node.create_publisher(PoseStamped, "/r1_sim/box_pose", 10)
        self.node.create_service(Trigger, "/r1_sim/reset", self._on_reset)

        self._reset_requested = threading.Event()
        self._reset_done = threading.Event()
        self._reset_info: dict = {}

        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self.node)
        self._spin = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin.start()

    # ------------------------------------------------------------------ 리셋
    def _on_reset(self, request, response):
        self._reset_done.clear()
        self._reset_requested.set()
        ok = self._reset_done.wait(timeout=3.0)
        response.success = ok
        response.message = json.dumps(self._reset_info) if ok else "timeout"
        return response

    def take_reset_request(self) -> bool:
        """시뮬 루프가 호출: 리셋 요청이 있으면 True (요청 플래그는 지운다)."""
        if self._reset_requested.is_set():
            self._reset_requested.clear()
            return True
        return False

    def reset_done(self, info: dict):
        self._reset_info = info
        self._reset_done.set()

    # ------------------------------------------------------------------ 발행
    def publish_box(self, pos, quat_wxyz):
        msg = PoseStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "pelvis_link"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (float(v) for v in pos)
        w, x, y, z = (float(v) for v in quat_wxyz)
        msg.pose.orientation.w, msg.pose.orientation.x = w, x
        msg.pose.orientation.y, msg.pose.orientation.z = y, z
        self.box_pub.publish(msg)

    def close(self):
        self._executor.shutdown()
        self.node.destroy_node()
        rclpy.try_shutdown()
