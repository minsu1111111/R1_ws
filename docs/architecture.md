# 구성과 데이터 흐름

## 계층

```
┌─────────────────────────────────────────────┐
│ LeRobot (정책: 무엇을 할지)                   │  학습은 서버, 추론은 로컬/서버
├─────────────────────────────────────────────┤
│ r1 패키지 (Python): interface, kinematics,   │  내가 작성하는 코드
│ teleop, policy_runner                        │
├──────────────────────┬──────────────────────┤
│ unitree_sdk2_python  │ ROS2 Humble (rclpy)  │
│ = 제어 주경로          │ = 관찰·기록·센서      │
├──────────────────────┴──────────────────────┤
│ CycloneDDS 0.10.x                            │  같은 DDS 버스
├─────────────────────────────────────────────┤
│ unitree_mujoco (C++ 시뮬) 또는 실제 R1          │
└─────────────────────────────────────────────┘
```

- **DDS**: 통신 방식. Unitree 로봇과 시뮬은 `rt/lowcmd`, `rt/lowstate` 토픽을 CycloneDDS 로 주고받습니다.
- **ROS2**: DDS 위에 만들어진 프레임워크. `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`로 맞추면 같은 토픽을 `/lowstate`로 볼 수 있습니다.
- **LeRobot**: 학습 라이브러리 (데이터셋, ACT/Diffusion/VLA 학습·추론). 통신은 하지 않습니다.

## DDS 와 ROS2 사용 구분

| 작업 | 사용 |
|---|---|
| 관절 PD 제어, 정책 실행, 원격조작 명령 | DDS (`r1.interface`) |
| RViz 시각화, tf, rosbag 기록 | ROS2 |
| 카메라·LiDAR 드라이버, SLAM, Nav2, MoveIt2 | ROS2 |
| 학습 데이터 저장 | LeRobot 데이터셋 (rosbag 아님) |

제어 루프는 ROS2를 거치지 않습니다. 지연과 의존성을 줄이기 위해서입니다.

## 시뮬과 실기

시뮬레이터는 실기와 같은 DDS 토픽을 내보내는 "가짜 로봇"입니다. 내 코드는 바뀌지 않고 환경만 바뀝니다.

| | 시뮬 | 실기 |
|---|---|---|
| 환경 스크립트 | `source env/setup_sim.sh` | `source env/setup_real.sh <NIC>` |
| 네트워크 인터페이스 | `lo` | 로봇과 연결된 유선 NIC (192.168.123.x) |
| domain ID | 1 | 0 |

시뮬레이터 자체는 C++ 프로그램이지만, 제어 코드는 전부 Python 입니다 ([decisions.md](decisions.md) D4).

## 로컬과 서버

| 위치 | 하는 일 | 환경 |
|---|---|---|
| 로컬 PC | 시뮬, DDS 제어, ROS2, 원격조작, 데이터 수집, 정책 평가, 실기 제어 | `.venv` 하나 (시스템 Python 3.10) |
| 학습 서버 | IL(ACT/VLA) 학습, RL 학습, 큰 VLA 추론 | conda `il` (3.10), `rl` (3.11) |

```
r1/teleop ─▶ data/datasets ─(push)─▶ 서버 training/ ─▶ data/checkpoints ─(pull)─▶ r1/policy_runner
```

`env/sync_to_server.sh`로 주고받습니다. 큰 VLA(pi0, GR00T)는 1060에서 추론이 어려우므로
서버의 추론 서버(`training/server`)가 행동을 계산하고 로컬이 받아 DDS로 보냅니다.
