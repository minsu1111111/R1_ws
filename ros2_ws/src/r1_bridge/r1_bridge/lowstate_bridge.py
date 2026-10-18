"""/lowstate (unitree_hg/LowState, ~900 Hz) → /joint_states, /imu, TF(world → pelvis_link).

- 모터 인덱스 → URDF 관절 이름 매핑은 docs/r1_model.md 관절표를 따른다.
  시뮬의 더미 관절(14, 20, 21, 27, 28)은 URDF 에 없으므로 내보내지 않는다.
- 머리(head_pitch/yaw)는 실기에서 모터 29, 30 이다. LowState 에 값이 있으면 쓰고, 시뮬처럼 없으면 0 을 낸다
  (robot_state_publisher 가 모든 관절 값을 받아야 TF 가 완성된다).
- 몸통 자세는 IMU 쿼터니언(pelvis 원점 기준)을 쓴다. 위치는 알 수 없으므로 고정 높이로 둔다.
- 입력이 ~900 Hz 이므로 publish_rate(Hz) 로 줄여서 내보낸다.
"""

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from tf2_ros import TransformBroadcaster
from unitree_hg.msg import LowState

# 모터 인덱스 → URDF 관절 이름 (R1.urdf, 시뮬 A5 기준)
MOTOR_TO_JOINT = {
    0: "left_hip_pitch_joint", 1: "left_hip_roll_joint", 2: "left_hip_yaw_joint",
    3: "left_knee_joint", 4: "left_ankle_pitch_joint", 5: "left_ankle_roll_joint",
    6: "right_hip_pitch_joint", 7: "right_hip_roll_joint", 8: "right_hip_yaw_joint",
    9: "right_knee_joint", 10: "right_ankle_pitch_joint", 11: "right_ankle_roll_joint",
    12: "waist_roll_joint", 13: "waist_yaw_joint",
    15: "left_shoulder_pitch_joint", 16: "left_shoulder_roll_joint", 17: "left_shoulder_yaw_joint",
    18: "left_elbow_joint", 19: "left_wrist_roll_joint",
    22: "right_shoulder_pitch_joint", 23: "right_shoulder_roll_joint", 24: "right_shoulder_yaw_joint",
    25: "right_elbow_joint", 26: "right_wrist_roll_joint",
    29: "head_pitch_joint", 30: "head_yaw_joint",
}
HEAD_MOTORS = (29, 30)


class LowStateBridge(Node):
    def __init__(self):
        super().__init__("r1_lowstate_bridge")
        self.declare_parameter("publish_rate", 100.0)
        self.declare_parameter("base_height", 0.75)
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("base_frame", "pelvis_link")
        self.declare_parameter("has_head", False)  # 실기: True (모터 29, 30)

        self.base_height = self.get_parameter("base_height").value
        self.world_frame = self.get_parameter("world_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.has_head = self.get_parameter("has_head").value

        self.latest = None
        self.create_subscription(LowState, "lowstate", self._on_state, qos_profile_sensor_data)
        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self.imu_pub = self.create_publisher(Imu, "imu", 10)
        self.tf = TransformBroadcaster(self)
        self.create_timer(1.0 / self.get_parameter("publish_rate").value, self._publish)
        self.get_logger().info("waiting for /lowstate ...")

    def _on_state(self, msg: LowState):
        if self.latest is None:
            self.get_logger().info("receiving /lowstate")
        self.latest = msg

    def _publish(self):
        msg = self.latest
        if msg is None:
            return
        now = self.get_clock().now().to_msg()

        js = JointState()
        js.header.stamp = now
        for i, name in MOTOR_TO_JOINT.items():
            js.name.append(name)
            if i in HEAD_MOTORS and not self.has_head:
                js.position.append(0.0)
                js.velocity.append(0.0)
                js.effort.append(0.0)
            else:
                m = msg.motor_state[i]
                js.position.append(float(m.q))
                js.velocity.append(float(m.dq))
                js.effort.append(float(m.tau_est))
        self.joint_pub.publish(js)

        w, x, y, z = (float(v) for v in msg.imu_state.quaternion)
        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = self.base_frame
        imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z = w, x, y, z
        gx, gy, gz = (float(v) for v in msg.imu_state.gyroscope)
        ax, ay, az = (float(v) for v in msg.imu_state.accelerometer)
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = gx, gy, gz
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = ax, ay, az
        self.imu_pub.publish(imu)

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.world_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.z = self.base_height
        t.transform.rotation.w, t.transform.rotation.x = w, x
        t.transform.rotation.y, t.transform.rotation.z = y, z
        self.tf.sendTransform(t)


def main():
    rclpy.init()
    node = LowStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
