# ROS2 패키지

`ros2_ws/src/` 의 패키지와 사용법입니다. ROS2 는 관찰·기록·센서용이고 제어는 하지 않습니다 ([decisions.md](decisions.md) D1).

| 패키지 | 종류 | 내용 |
|---|---|---|
| `unitree_hg`, `unitree_go`, `unitree_api` | ament_cmake | Unitree 메시지 (third_party/unitree_ros2 심볼릭 링크) |
| `r1_description` | ament_cmake | R1 전신 URDF + 메시 |
| `r1_bridge` | ament_python | `/lowstate` → `/joint_states`, `/imu`, TF + RViz launch |

## 실행

```bash
bash sim/run_r1_sim.sh                              # 터미널 1: 시뮬
source env/setup_sim.sh                             # 터미널 2
ros2 launch r1_bridge view.launch.py                # RViz
python -m r1.interface.examples.wave_arm            # 터미널 3: 움직여 보기 (MuJoCo 와 RViz 에서 같이 움직임)
```

| launch 인자 | 기본값 | 의미 |
|---|---|---|
| `has_head` | `false` | `true` 면 머리 각도를 모터 29, 30 에서 읽음 (실기). 시뮬은 머리가 없어 0 |
| `rviz` | `true` | `false` 면 RViz 없이 브리지 + robot_state_publisher 만 |

빌드 (venv 끈 셸):

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws && colcon build && cd ..
```

시뮬 카메라 영상은 ROS2 패키지가 아니라 Python 시뮬이 직접 발행한다 (`python -m r1.sim.run --ros-cameras`, [sim.md](sim.md)).
RViz 설정(`r1_bridge/rviz/r1.rviz`)에 `HeadCamera`, `FrontCamera` 영상 패널이 있다. 토픽이 없으면 빈 패널로 남는다.

## r1_description

- 원본: `third_party/unitree_ros/robots/r1_description/R1.urdf` (sparse checkout, 커밋 고정)
- 빌드 때 CMake 가 메시 경로 `meshes/...` 를 `package://r1_description/meshes/...` 로 바꾼 `r1.urdf` 를 만들어 설치한다. 원본은 수정하지 않는다.
- 구성: 다리 12, 허리 roll/yaw, 팔 5×2, 머리 pitch/yaw = 26 관절. 루트 링크 `pelvis_link`.
- 24개 공통 관절의 이름·범위가 시뮬 모델(`R1_C++.xml`)과 일치함을 확인 (차이 < 0.02 rad).
- A7 전신 URDF 는 공식으로 없다 (`r1_a7_description` 은 상체만). A7 확정 후 다시 다룬다.

## r1_bridge (`lowstate_bridge` 노드)

| 입력 | 출력 |
|---|---|
| `/lowstate` (`unitree_hg/msg/LowState`, ~900 Hz, sensor QoS) | `/joint_states` (100 Hz, 26 관절) |
| | `/imu` (`sensor_msgs/Imu`, 프레임 `pelvis_link`) |
| | TF `world → pelvis_link` (IMU 자세, 높이 0.75 m 고정) |

- 모터 인덱스 → 관절 이름 매핑은 [r1_model.md](r1_model.md) 관절표. 더미 관절(14, 20, 21, 27, 28)은 내보내지 않는다.
- 몸통 **위치**는 LowState 에 없으므로 고정 높이로 둔다. 자세(기울기)만 실제 값이다.
- 파라미터: `publish_rate`(100), `base_height`(0.75), `world_frame`(world), `base_frame`(pelvis_link), `has_head`(false).

## 시험 결과 (2026-10-18, 시뮬 domain 2)

- `/joint_states` 100.2 Hz, 관절 26개
- 팔 흔들기 중 `left_elbow_joint` 값과 TF(`world → left_elbow_link`)가 갱신됨
- RViz 에서 R1 전신이 MuJoCo 와 같은 자세로 표시됨, `Global Status: Ok`
- 시뮬 + RViz + 예제를 함께 돌릴 때 팔 흔들기 최대 추종 오차 0.122 rad (단독 실행 0.090 rad). CPU 부하 영향으로 보임
