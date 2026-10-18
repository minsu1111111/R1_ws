# 로컬 환경 설치

기준 PC: Ubuntu 22.04, ROS2 Humble, GTX 1060 6GB, NVIDIA driver 580.
모든 명령은 워크스페이스 루트(`~/aisl_robot_ws`)에서 실행합니다. `WS=~/aisl_robot_ws`.

진행 상태: ✅ 완료 / ⏳ 진행 중 / ⬜ 남음

## 1. 시스템 패키지 (sudo) ✅

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp ros-humble-rosidl-generator-dds-idl libglfw3-dev
```

| 패키지 | 용도 |
|---|---|
| `ros-humble-rmw-cyclonedds-cpp` | ROS2 가 CycloneDDS 로 통신 (Unitree 토픽을 보기 위해 필수) |
| `ros-humble-rosidl-generator-dds-idl` | `unitree_hg/go/api` 메시지 빌드 |
| `libglfw3-dev` | C++ 시뮬 viewer |

이미 있던 것: cmake, g++, `libboost-all-dev`, `libspdlog-dev`, `libyaml-cpp-dev`, ROS2 desktop-full.

## 2. 외부 저장소 ✅

```bash
bash env/clone_third_party.sh
```

`third_party/` 에 공식 저장소 9개를 받아 **고정 커밋으로 체크아웃**하고, `ros2_ws/src/unitree_{hg,go,api}` 심볼릭 링크를 만듭니다.
이미 받은 저장소에 다시 실행하면 고정 커밋과 맞는지 확인합니다 (`[ok]` / `[pin]` / `[warn]`).
목록과 용도는 [third_party/README.md](../third_party/README.md), 관리 원칙은 [decisions.md](decisions.md) D8.

| 저장소 | 고정 커밋 | 커밋 날짜 |
|---|---|---|
| unitree_mujoco | `1eb6642e` | 2026-09-07 |
| unitree_sdk2 | `d1abf001` | 2026-10-15 |
| unitree_sdk2_python | `814556d1` | 2026-09-21 |
| unitree_ros2 | `668d1ec5` | 2026-07-02 |
| xr_teleoperate | `817fb00c` | 2026-09-07 |
| unitree_IL_lerobot | `41c28057` | 2026-05-25 |
| unitree_rl_mjlab | `1425b15f` | 2026-04-13 |
| lerobot | `a5b29d43` (v0.4.1) | 2025-11-10 |
| unitree_ros (R1 폴더만) | `5994d4fa` | 확인일 2026-10-18 |

## 3. Python venv ✅

```bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -c env/constraints.txt cyclonedds==0.10.2 numpy mujoco pin lerobot==0.4.1
pip install -c env/constraints.txt -e third_party/unitree_sdk2_python -e .
```

- conda 를 쓰지 않습니다. 이 워크스페이스에서는 conda 환경이 꺼져 있어야 합니다 ([decisions.md](decisions.md) D2).
- 버전 고정은 `env/constraints.txt` 한 곳에서 관리합니다 (lerobot 0.4.1: D3, numpy 2: D5).
- `--system-site-packages` 는 ROS2 의 `rclpy` 를 venv 에서 쓰기 위함입니다.

## 4. C++ unitree_sdk2 (sudo 불필요) ✅

```bash
cmake -S third_party/unitree_sdk2 -B third_party/unitree_sdk2/build \
      -DCMAKE_INSTALL_PREFIX=$WS/third_party/unitree_sdk2/install -DBUILD_EXAMPLES=OFF
cmake --build third_party/unitree_sdk2/build -j4
cmake --install third_party/unitree_sdk2/build
```

공식 문서는 `/opt/unitree_robotics` 설치를 권하지만 sudo 없이 워크스페이스 안에 둡니다 (D6).

## 5. MuJoCo C 라이브러리 3.3.6 ✅

```bash
mkdir -p third_party/mujoco_bin
curl -L https://github.com/google-deepmind/mujoco/releases/download/3.3.6/mujoco-3.3.6-linux-x86_64.tar.gz \
  | tar xz -C third_party/mujoco_bin
ln -sfn $WS/third_party/mujoco_bin/mujoco-3.3.6 third_party/unitree_mujoco/simulate/mujoco
```

C++ 시뮬 빌드용입니다. Python `mujoco` 패키지(venv)와는 별개입니다.

## 6. C++ 시뮬레이터 빌드 ✅

```bash
cmake -S third_party/unitree_mujoco/simulate -B third_party/unitree_mujoco/simulate/build \
      -DCMAKE_PREFIX_PATH=$WS/third_party/unitree_sdk2/install
cmake --build third_party/unitree_mujoco/simulate/build -j4
```

실행은 **반드시 스크립트로** 합니다 (ROS2 의 libddsc 와 섞이는 문제, [troubleshooting.md](troubleshooting.md)):

```bash
bash sim/run_r1_sim.sh            # 기본 scene.xml, 메시지 타입은 모터 수로 unitree_hg 자동 선택
```

시뮬 설정은 [sim/r1_sim_config.yaml](../sim/r1_sim_config.yaml) 에서 고칩니다 (실행 때 시뮬 위치로 복사됨).
기본으로 **가상 고무줄이 켜져** 있어 R1 이 `torso_link` 로 매달린 채 시작합니다 (실기 거치대 역할).

| 키 (시뮬 창) | 동작 |
|---|---|
| `9` | 고무줄 켜기/끄기 |
| `7` / `↑` | 줄 0.1 m 짧게 |
| `8` / `↓` | 줄 0.1 m 길게 |
| `Backspace` | 시뮬 리셋 |

확인 (2026-10-18): 고무줄 켬 → 6초 동안 roll 0°, pitch ±2° 이내로 직립 유지. 끄면 제어 명령이 없을 때 쓰러짐.

## 7. ROS2 워크스페이스 빌드 ✅

venv 를 켜지 않은 터미널에서 빌드합니다 (colcon 이 시스템 Python 을 쓰도록).

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws && colcon build && cd ..
```

`unitree_api`, `unitree_go`, `unitree_hg` 3개 패키지가 빌드됩니다 (약 25초).

## 8. 동작 확인

```bash
# 터미널 1
bash sim/run_r1_sim.sh

# 터미널 2
source env/setup_sim.sh
ros2 topic list               # /lowstate, /lowcmd 등이 보여야 함
ros2 topic info /lowstate     # Type: unitree_hg/msg/LowState
ros2 topic hz /lowstate       # 약 900 Hz
python -c "import rclpy, unitree_sdk2py, mujoco, lerobot, pinocchio; print('ok')"
python -c "import torch; print(torch.cuda.is_available())"
```

Python(sdk2_python)에서 상태 읽기:

```python
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
ChannelFactoryInitialize(1, "lo")          # 시뮬: domain 1, lo
sub = ChannelSubscriber("rt/lowstate", LowState_)
sub.Init(lambda m: print(m.motor_state[0].q), 10)
```

2026-10-18 확인 결과:
- ✅ ROS2 에서 `/lowstate` (`unitree_hg/msg/LowState`) 약 910 Hz 수신
- ✅ sdk2_python 에서 `rt/lowstate` 약 890 Hz 수신 (3초에 2662개)
- ✅ 모든 패키지 import, `torch.cuda.is_available() == True`, GPU 연산 정상
시뮬 토픽: `/lowstate`, `/lowcmd`, `/secondary_imu`, `/lf/bmsstate`, `/sportmodestate`, `/wirelesscontroller`.
제어 명령이 없으면 로봇은 넘어집니다 (정상).

안 될 때는 [troubleshooting.md](troubleshooting.md).

## 설치된 주요 버전

| 패키지 | 버전 |
|---|---|
| Python | 3.10.12 (시스템) |
| ROS2 | Humble |
| cyclonedds (Python) | 0.10.2 |
| unitree_sdk2py | 1.0.1 |
| mujoco (Python) | 3.16.0 |
| MuJoCo C (C++ 시뮬용) | 3.3.6 |
| lerobot | 0.4.1 |
| numpy | 2.2.6 |
| torch | 2.7.1+cu126 (sm_60 포함 → GTX 1060 사용 가능) |
| pinocchio (`pin`) | 4.1.0 |
| setuptools | 79.0.1 (<80, colcon 호환) |
| unitree_sdk2 (C++) | `third_party/unitree_sdk2/install` |
