# 문제 해결 기록

형식: 증상 / 원인 / 해결.

---

## C++ 시뮬이 시작 직후 abort (`dds_write.c:318 ... iox_pub ... Assertion failed`)

- **증상**: `./unitree_mujoco -r r1` 실행 → 모델 정보 출력 후 위 assertion 과 함께 core dump.
- **원인**: `.bashrc` 에서 `source /opt/ros/humble/setup.bash` 를 하므로 `LD_LIBRARY_PATH` 에 ROS2 경로가 앞에 있다.
  `ldd` 로 보면 `libddsc.so.0` 는 ROS2 것(iceoryx 공유메모리 포함 빌드), `libddscxx.so.0` 는 unitree_sdk2 것이 잡혀 서로 섞인다.
  ROS2 환경이 없는 셸(`env -i ... ldd`)에서는 둘 다 unitree_sdk2 것이 잡힌다.
- **해결**: `bash sim/run_r1_sim.sh` 로 실행한다. `third_party/unitree_sdk2/install/lib` 를 `LD_LIBRARY_PATH` 맨 앞에 둔다.
- **주의**: unitree_sdk2 에 링크된 다른 C++ 프로그램(RL deploy 등)도 같은 방식으로 실행해야 한다.
- 확인: `ldd <바이너리> | grep ddsc` 에서 두 라이브러리가 같은 경로인지 본다.

## 시뮬 설정을 바꿨는데 반영이 안 됨 / config.yaml 이 원래대로 돌아감

- **원인**: 시뮬은 `third_party/unitree_mujoco/simulate/config.yaml` 만 읽고, 그 파일은 `sim/run_r1_sim.sh` 실행 때마다 `sim/r1_sim_config.yaml` 로 덮어써진다.
  또 그 파일을 에디터에 열어 둔 채 저장하면 예전 내용으로 되돌아간다 (2026-10-18 실제로 발생).
- **해결**: 설정은 `sim/r1_sim_config.yaml` 에서만 고치고, third_party 쪽 파일은 열지 않는다.

## `Package 'r1_bridge' not found` (ros2 launch / ros2 run)

- **증상**: `ros2 launch r1_bridge view.launch.py` → `Package 'r1_bridge' not found`, 검색 경로에 `unitree_hg/go/api` 만 보임.
- **원인**: 그 터미널에서 `source env/setup_sim.sh` 를 한 뒤에 새 패키지가 빌드됐다. `install/setup.bash` 는 source 하는 순간의 패키지 목록만 경로에 넣는다.
- **해결**: 같은 터미널에서 `source env/setup_sim.sh` 를 다시 하거나 새 터미널을 연다.
- 새 패키지를 `colcon build` 한 뒤에는 열려 있는 터미널 모두 다시 source 한다. (기존 패키지 코드만 고친 경우는 필요 없음)

## robot_state_publisher: `The root link pelvis_link has an inertia specified in the URDF, but KDL does not support...`

- 공식 R1.urdf 의 루트 링크에 관성값이 있어서 나오는 경고. RViz 표시와 TF 에는 영향 없음 (확인함). 무시.

## `selected interface "lo" is not multicast-capable: disabling multicast`

- 경고일 뿐이다. 루프백에서는 multicast 없이 동작하며 `/lowstate` 수신에 문제 없음을 확인했다.

## Python 시뮬에서 R1 이 맞지 않는 메시지로 통신함

- **증상**: `simulate_python/config.py` 에서 `ROBOT = "r1"` 로 두면 `unitree_go` LowCmd/LowState 를 쓴다.
- **원인**: `unitree_sdk2py_bridge.py` 가 `config.ROBOT == "g1"` 일 때만 `unitree_hg` 를 import 한다.
- **해결**: C++ 시뮬 사용 ([decisions.md](decisions.md) D4).

## pip: `ResolutionImpossible` (lerobot 0.4.1 + numpy<2)

- **증상**: `pip install -c env/constraints.txt lerobot==0.4.1` 이 실패.
- **원인**: `rerun-sdk 0.24~0.26` 이 `numpy>=2` 를 요구.
- **해결**: numpy 고정을 `>=2,<2.3` 으로 변경 (D5).

## `cv_bridge` import 실패 (`_ARRAY_API not found`)

- **증상**: venv(numpy 2) 에서 `from cv_bridge import CvBridge` 후 사용 시 실패.
- **원인**: apt 로 설치된 `cv_bridge`, 시스템 `cv2` 가 numpy 1 로 빌드됨.
- **해결**: cv_bridge 를 쓰지 않고 `tobytes()` / `np.frombuffer()` 로 직접 변환 (D5).

## `pip` 중 `Can't uninstall 'Pygments'. No files were found to uninstall.`

- **증상**: 설치 로그에 위 메시지.
- **원인**: `--system-site-packages` venv 에서 시스템 패키지(apt)를 덮어쓰려 할 때 나오는 메시지.
- **해결**: 무시해도 됨. venv 쪽에 새 버전이 설치되고 그것이 우선 사용된다.

## pip: `colcon-core 0.21.3 has requirement setuptools<80`

- **증상**: lerobot 설치 후 pip 이 위 충돌 경고를 출력.
- **원인**: lerobot 의존성 설치 중 venv 의 setuptools 가 80.x 로 올라감. colcon(시스템)은 <80 요구.
- **해결**: `env/constraints.txt` 에 `setuptools<80` 추가 후 재설치 (79.0.1). lerobot 0.4.1 은 `<81` 이라 문제없음.
- 참고: colcon 빌드는 venv 를 끈 셸에서 한다.

## pip: `pynacl 1.5.0 requires cffi, which is not installed`

- 시스템(apt) 패키지 pynacl 에 대한 경고로, 우리 코드는 pynacl 을 쓰지 않는다. 무시.

## PyTorch 가 GTX 1060 을 지원하는지

- torch 2.7.1+cu126 의 arch 목록에 `sm_60` 이 있어 Pascal(1060, sm_61)에서 동작한다. 확인함.
- CUDA 12.8 이상 빌드의 PyTorch 는 Pascal 지원이 빠졌으므로 torch 를 올릴 때 `cu126` 빌드인지 확인한다.

## ROS2 에서 `/lowstate` 가 안 보임 (예상되는 문제)

체크리스트:
1. `echo $RMW_IMPLEMENTATION` → `rmw_cyclonedds_cpp` 인가 (`ros-humble-rmw-cyclonedds-cpp` 설치 필요)
2. `echo $ROS_DOMAIN_ID` → 시뮬 `config.yaml` 의 `domain_id` 와 같은가
3. `CYCLONEDDS_URI` 의 인터페이스가 시뮬 `interface` 와 같은가 (`lo`)
4. `ros2_ws` 를 빌드하고 `install/setup.bash` 를 source 했는가 (`unitree_hg` 메시지 타입)
5. ROS2 데몬이 예전 설정으로 떠 있으면 `ros2 daemon stop` 후 재시도
