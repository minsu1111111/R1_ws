# aisl_robot_ws

Unitree **R1 EDU** 휴머노이드로 **모방학습 / VLA / RL**을 하기 위한 워크스페이스입니다.
실기가 오기 전에는 MuJoCo 시뮬에서 개발하고, 실기가 오면 네트워크 설정만 바꿔 같은 코드를 그대로 씁니다.

- **로컬 PC**: MuJoCo 시뮬, DDS 제어, ROS2 시각화·기록, 원격조작·데이터 수집, 학습된 정책 평가
- **외부 서버**: LeRobot(ACT / VLA) 학습, `unitree_rl_mjlab` RL 학습, VLA 추론 서버

> 📚 자세한 문서는 [docs/](docs/README.md): [설치](docs/setup.md) · [구성](docs/architecture.md) · [R1 모델](docs/r1_model.md) · [제어 래퍼](docs/r1_interface.md) · [ROS2](docs/ros2.md) · [설계 결정](docs/decisions.md) · [문제 해결](docs/troubleshooting.md)

## 구조 개요

```
unitree_mujoco (R1 sim) 또는 실제 R1
        │  rt/lowstate, rt/lowcmd  (CycloneDDS, unitree_hg)
        ├── r1.interface (unitree_sdk2_python)   ← 제어 주경로: PD, 정책 실행, 안전 제한
        └── ROS2 Humble (rmw_cyclonedds_cpp)     ← 관찰·기록·센서 통합
              ├ r1_bridge      → /joint_states, /imu → RViz
              ├ r1_sim_camera  → /camera/*/image_raw
              └ rosbag2
```

- 시뮬(`unitree_mujoco`)은 실기와 **같은 DDS 토픽**을 주고받기 때문에, 제어 코드는 시뮬과 실기에서 동일합니다.
- 제어 루프는 ROS2를 거치지 않고 DDS로 직접 보냅니다(지연 최소화). ROS2는 시각화, 기록, 센서, 이후 Nav2/MoveIt2 확장용입니다.

## 디렉토리

```
aisl_robot_ws/
├── third_party/      외부 저장소 원본 (git 제외, 직접 수정하지 않음)
├── r1/               [로컬] Python 패키지 (sim/실기 공통)
│   ├── interface/      DDS 래퍼: 상태 읽기, LowCmd, 관절·속도 제한, E-stop
│   ├── kinematics/     팔 IK (pinocchio)
│   ├── teleop/         원격조작 + LeRobot 형식 데이터 기록
│   └── policy_runner/  정책 실행 (로컬 추론 / 서버 VLA 클라이언트)
├── ros2_ws/src/      [로컬] ROS2 패키지 (colcon)
│   ├── unitree_hg|go|api  → third_party/unitree_ros2 심볼릭 링크
│   ├── r1_description/    URDF + meshes
│   ├── r1_bridge/         LowState → JointState / Imu, launch, RViz 설정
│   └── r1_sim_camera/     MuJoCo 카메라 → sensor_msgs/Image
├── sim/              [로컬] 시뮬 실행 스크립트·설정, MJCF 장면 (R1 + 테이블·물체·카메라)
├── training/         [서버] 학습 코드
│   ├── il/             LeRobot 학습 설정, R1 데이터 변환
│   ├── rl/             unitree_rl_mjlab R1 작업 설정
│   └── server/         VLA 추론 서버
├── data/             git 제외, 서버와 rsync
│   ├── datasets/       로컬 수집 → 서버
│   └── checkpoints/    서버 학습 결과 → 로컬
├── env/              환경 스크립트, 버전 고정(constraints.txt)
├── docs/             확정된 문서: 설치, 구성, 모델, 설계 결정, 문제 해결
└── notes/            공부 정리, 실험 로그
```

### 데이터 흐름

```
r1/teleop ─▶ data/datasets ─(push)─▶ 서버 training/ ─▶ data/checkpoints ─(pull)─▶ r1/policy_runner (MuJoCo 평가)
```

## 환경

| 항목 | 버전 |
|---|---|
| OS | Ubuntu 22.04 |
| ROS2 | Humble |
| DDS | CycloneDDS 0.10.x (Unitree 실기와 동일 계열) |
| Python | 3.10 |

## 시작하기 (로컬)

전체 절차와 이유는 [docs/setup.md](docs/setup.md)에 있습니다. 요약:

```bash
# 1. 시스템 패키지
sudo apt install ros-humble-rmw-cyclonedds-cpp ros-humble-rosidl-generator-dds-idl libglfw3-dev

# 2. 외부 저장소 + ROS2 메시지 링크
bash env/clone_third_party.sh

# 3. Python venv (conda 사용 안 함, 버전은 env/constraints.txt)
python3 -m venv --system-site-packages .venv && source .venv/bin/activate
pip install -c env/constraints.txt cyclonedds==0.10.2 numpy mujoco pin lerobot==0.4.1
pip install -c env/constraints.txt -e third_party/unitree_sdk2_python -e .

# 4. C++ unitree_sdk2, MuJoCo 3.3.6, C++ 시뮬 빌드 → docs/setup.md 4~6

# 5. ROS2 워크스페이스 (venv 끈 터미널에서)
source /opt/ros/humble/setup.bash && cd ros2_ws && colcon build && cd ..
```

### 시뮬 실행과 확인

시뮬레이터는 C++ 프로그램(`unitree_mujoco/simulate`)이고, 제어 코드는 Python 입니다.

```bash
# 터미널 1: R1 시뮬 (라이브러리 경로 문제 때문에 반드시 스크립트로 실행)
bash sim/run_r1_sim.sh

# 터미널 2: ROS2 에서 DDS 토픽 확인
source env/setup_sim.sh
ros2 topic list            # /lowstate 가 보여야 함
ros2 topic echo /lowstate --once
```

### 실기로 전환

```bash
source env/setup_real.sh enp3s0   # 로봇과 연결된 NIC (ip a 로 확인)
```

시뮬과 실기는 NIC과 domain ID로 구분합니다(시뮬 `lo` / domain 1, 실기 유선 NIC / domain 0).

## 서버 학습

서버에는 이 저장소와 `third_party/lerobot`, `third_party/unitree_rl_mjlab`만 있으면 됩니다.

```bash
# 로컬 → 서버 데이터 업로드, 서버 → 로컬 체크포인트 받기
R1_SERVER=user@host R1_SERVER_WS=~/aisl_robot_ws bash env/sync_to_server.sh both
```

- 데이터셋 형식이 맞도록 **서버와 로컬의 lerobot 버전을 같게 고정**합니다.
- RL: `Unitree-R1-Flat` 작업으로 학습한 뒤, `unitree_rl_mjlab/deploy/robots/r1`을 로컬에서 빌드해 시뮬에 DDS로 붙여 검증합니다(sim2sim).

## 로드맵

- [x] 1. 기반 설치, R1 시뮬 실행
- [x] 2. 환경 스크립트 (`env/`)
- [x] 3. ROS2 메시지 빌드, `/lowstate` 확인
- [x] 4. `r1_description` (공식 전신 URDF, 관절명·범위 시뮬과 일치) → [docs/ros2.md](docs/ros2.md)
- [x] 5. `r1.interface`: LowState 읽기, LowCmd PD, 제한·E-stop → [docs/r1_interface.md](docs/r1_interface.md)
- [x] 6. `r1_bridge`: RViz에서 시뮬 자세 확인 → [docs/ros2.md](docs/ros2.md)
- [ ] 7. 팔 IK, 조작 장면(테이블·물체·카메라), 카메라 토픽
- [ ] 8. 원격조작 + LeRobot 데이터 수집
- [ ] 9. ACT 학습(서버) → 시뮬 폐루프 평가
- [ ] 10. VLA 추론 서버-클라이언트 (더미 모델 → pi0 / GR00T)
- [ ] 11. RL 보행 학습(서버) → sim2sim 검증

## 주의

- 시뮬 모델은 모터 슬롯 29개 중 **실제 관절 24개(팔 5자유도)**, 나머지 5개는 더미입니다. 머리 관절, 카메라, 손은 없습니다.
- 구매 구성: R1 EDU, **손 없음**. 팔(A5/A7)은 미확정 → [docs/r1_model.md](docs/r1_model.md). 첫 IL 과제는 잡지 않는 조작(밀기·누르기) → [D9](docs/decisions.md)
- 로컬에서는 conda 를 켜지 않습니다 (ROS2 Humble 과 충돌) → [docs/decisions.md](docs/decisions.md) D2
- 시뮬과 실기를 같은 네트워크·도메인에서 동시에 켜면 토픽이 섞입니다.
- 실기에서 저수준 제어를 처음 할 때는 **반드시 거치대에 매단 상태**에서 하고, 비상정지를 먼저 확인합니다.

## 참고

- [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco)
- [unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python)
- [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2)
- [xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate)
- [unitree_IL_lerobot](https://github.com/unitreerobotics/unitree_IL_lerobot)
- [unitree_rl_mjlab](https://github.com/unitreerobotics/unitree_rl_mjlab)
- [LeRobot](https://github.com/huggingface/lerobot)
