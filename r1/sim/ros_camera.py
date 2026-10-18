"""시뮬 카메라 영상 → ROS2 (sensor_msgs/Image + CameraInfo).

    /camera/head/image_raw, /camera/head/camera_info     (head_cam: torso_link 에 붙은 근사 머리 카메라)
    /camera/front/image_raw, /camera/front/camera_info   (front_cam: 월드 고정 관찰 카메라)

- 시뮬 프로세스 안에서 rclpy 로 발행한다 (렌더링에 시뮬 상태가 필요하므로).
  Unitree DDS(cyclonedds Python)와 rclpy(rmw_cyclonedds_cpp)가 한 프로세스에서 충돌 없이 동작함을 확인했다.
- msg.data 에 bytes 를 넣으면 rclpy 가 원소마다 검사해 640x480 한 장에 ~117 ms 걸린다.
  array.array('B') 로 넣으면 ~0.3 ms (docs/troubleshooting.md).
- QoS 는 기본(reliable, depth 10) → RViz Image 표시와 호환.
"""

import array

import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image

TOPIC_NAME = {"head_cam": "head", "front_cam": "front"}


def camera_info(width: int, height: int, fovy_deg: float, frame_id: str) -> CameraInfo:
    """MuJoCo 카메라(fovy = 세로 화각, 정사각 픽셀)의 핀홀 내부 파라미터."""
    fy = (height / 2) / np.tan(np.radians(fovy_deg) / 2)
    fx = fy
    cx, cy = width / 2, height / 2
    info = CameraInfo()
    info.header.frame_id = frame_id
    info.width, info.height = width, height
    info.distortion_model = "plumb_bob"
    info.d = [0.0] * 5
    info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return info


class RosCameraPublisher:
    def __init__(self, cameras: dict[str, float], width: int, height: int):
        """cameras: {MuJoCo 카메라 이름: fovy[deg]}"""
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("r1_sim_camera")
        self.width, self.height = width, height
        self._img, self._info, self._pub_img, self._pub_info = {}, {}, {}, {}
        for cam, fovy in cameras.items():
            name = TOPIC_NAME.get(cam, cam)
            frame = f"{name}_camera"
            img = Image()
            img.header.frame_id = frame
            img.height, img.width, img.encoding, img.step = height, width, "rgb8", width * 3
            self._img[cam] = img
            self._info[cam] = camera_info(width, height, fovy, frame)
            self._pub_img[cam] = self.node.create_publisher(Image, f"/camera/{name}/image_raw", 10)
            self._pub_info[cam] = self.node.create_publisher(CameraInfo, f"/camera/{name}/camera_info", 10)

    def publish(self, cam: str, rgb: np.ndarray):
        stamp = self.node.get_clock().now().to_msg()
        img, info = self._img[cam], self._info[cam]
        img.header.stamp = info.header.stamp = stamp
        img.data = array.array("B", rgb.tobytes())
        self._pub_img[cam].publish(img)
        self._pub_info[cam].publish(info)

    def close(self):
        self.node.destroy_node()
        rclpy.try_shutdown()
