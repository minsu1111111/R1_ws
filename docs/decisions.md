# 설계 결정 기록

형식: 결정 / 근거 / 영향. 날짜는 결정한 날입니다.

---

## D1. 제어는 DDS(sdk2_python), ROS2 는 관찰·기록·센서용 (2026-10-18)

- **결정**: 로봇 제어 경로는 `unitree_sdk2_python`으로 DDS 에 직접 보낸다. ROS2 는 같은 DDS 버스를 구독해 시각화, rosbag, 센서 통합에 쓴다.
- **근거**
  - Unitree 공식 IL/원격조작 코드(`xr_teleoperate`, `unitree_IL_lerobot`)가 ROS2 를 쓰지 않고 DDS 로 직접 제어한다.
  - RL 배포 코드(`unitree_rl_mjlab/deploy`)도 `unitree_sdk2`(C++) 를 쓴다.
  - 제어 루프(수백 Hz)에 ROS2 계층을 추가할 이유가 없다.
- **영향**: ROS2 는 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` 와 같은 domain ID 로 맞춰야 토픽이 보인다.

## D2. 로컬은 conda 없이 시스템 Python 3.10 + venv 하나 (2026-10-18)

- **결정**: `python3 -m venv --system-site-packages .venv` 하나에 ROS2, SDK, MuJoCo, LeRobot 을 모두 둔다.
- **근거** (각 저장소에서 직접 확인)

  | 구성 요소 | Python |
  |---|---|
  | ROS2 Humble `rclpy` | 시스템 3.10 으로 빌드됨 |
  | `cyclonedds==0.10.2` (sdk2_python 요구) | PyPI wheel 이 cp310 까지만 있음 |
  | `unitree_IL_lerobot` | `>=3.10,<3.11` |

  conda 환경을 켜면 conda 의 Python / `libstdc++` 가 먼저 잡혀 `rclpy`, colcon 빌드가 깨질 수 있다.
- **영향**: 이 워크스페이스에서는 conda 를 켜지 않는다 (`auto_activate_base: False` 확인됨). `--system-site-packages` 로 rclpy 를 venv 에서 import 한다.

## D3. LeRobot 은 v0.4.1 고정 (2026-10-18)

- **결정**: 로컬과 서버 `il` 환경 모두 `lerobot==0.4.1`.
- **근거**
  - `unitree_IL_lerobot` 의 lerobot 서브모듈이 커밋 `a5b29d4` = **v0.4.1** 에 고정되어 있다.
  - 최신 lerobot 0.6.x 는 `requires-python >=3.12` → ROS2 Humble 과 같은 환경에 설치 불가.
  - 데이터셋 형식(`CODEBASE_VERSION`)은 v0.4.1 과 최신 모두 `v3.0` → 나중에 서버에서만 최신 버전을 써도 데이터는 호환된다.
- **영향**: 체크포인트 로딩은 버전마다 다를 수 있으므로, 서버에서 다른 버전으로 학습한 모델은 서버에서 추론한다.

## D4. 시뮬레이터는 C++ 버전 (`unitree_mujoco/simulate`) (2026-10-18)

- **결정**: R1 시뮬은 C++ 시뮬레이터로 실행한다. 제어 코드는 Python 이다.
- **근거**
  - Python 시뮬(`simulate_python`)은 `ROBOT == "g1"` 일 때만 `unitree_hg` 메시지를 쓴다. `r1` 로 설정하면 `unitree_go`(20 모터) 메시지가 되어 R1(29 슬롯)과 맞지 않는다.
  - Python 시뮬의 elastic band 는 g1/h1 이 아니면 `base_link` 에 붙는데 R1 에는 `base_link` 가 없다.
  - C++ 시뮬은 모터 수 > 20 이면 `unitree_hg` 를 자동 선택하고(`param.h`), band 는 `torso_link`(R1 에 있음)에 붙는다. README 도 C++ 을 권장한다.
- **대안 (보류)**: Python 시뮬을 래퍼에서 `ROBOT="g1"` + R1 장면으로 실행하면 수정 없이 동작할 것으로 보인다. 카메라 렌더링·장면 리셋을 Python 으로 확장하기 쉬워 IL 데이터 수집 단계에서 다시 검토한다.
- **영향**: `libglfw3-dev`, C++ `unitree_sdk2`, MuJoCo 3.3.6 C 라이브러리가 필요하다.
  ROS2 를 source 한 셸에서는 라이브러리가 섞이므로 `sim/run_r1_sim.sh` 로만 실행한다.

## D5. venv 에서 numpy 2 사용, cv_bridge 미사용 (2026-10-18)

- **결정**: `numpy>=2,<2.3`. ROS 이미지 변환은 `cv_bridge` 대신 직접 한다.
- **근거**
  - lerobot 0.4.1 → `rerun-sdk>=0.24` → `numpy>=2`. `numpy<2` 고정 시 pip 해석 불가.
  - ROS2 Humble 에서 numpy 1.21 / 2.2 비교 테스트 결과:

    | 항목 | numpy 1.21 | numpy 2.2 |
    |---|---|---|
    | JointState / Image 생성·직렬화 | 정상 | 정상 |
    | rclpy 노드 publish | 정상 | 정상 |
    | `cv_bridge` | 정상 | **실패** (`_ARRAY_API not found`) |
- **영향**: 이미지 메시지는 `msg.data = array.array("B", arr.tobytes())` (bytes 는 ~350배 느림), 수신은 `np.frombuffer(msg.data, np.uint8).reshape(h, w, c)` 로 처리한다.

## D6. C++ unitree_sdk2 는 워크스페이스 안에 설치 (2026-10-18)

- **결정**: `/opt/unitree_robotics` 대신 `third_party/unitree_sdk2/install` 에 설치.
- **근거**: sudo 없이 빌드·재설치 가능, 워크스페이스를 지우면 같이 정리된다.
- **영향**: C++ 빌드 시 `-DCMAKE_PREFIX_PATH=$WS/third_party/unitree_sdk2/install` 을 넘긴다.

## D7. 학습은 외부 서버, 나머지는 로컬 (2026-10-18)

- **결정**: 학습(IL, RL)과 큰 VLA 추론만 서버. 시뮬, 데이터 수집, 평가, 실기 제어는 로컬.
- **근거**: 로컬 GPU 는 GTX 1060 6GB. MuJoCo 시뮬과 ACT 추론은 가능하지만 RL 병렬 학습, VLA 파인튜닝은 불가.
- **영향**: 서버에는 이 저장소 + `third_party/lerobot`, `third_party/unitree_rl_mjlab` 만 필요. RL 은 서버 conda `rl` (Python 3.11, `mjlab==1.2.0`, `mujoco-warp==3.5.0`).

## D8. 외부 저장소(third_party) 관리 방식 (2026-10-18)

- **결정**
  1. `third_party/` 코드는 직접 수정하지 않는다. 설정은 우리 레포(`sim/r1_sim_config.yaml` 등)에 두고 실행 스크립트가 복사한다.
  2. 모든 외부 저장소를 **커밋 고정**한다 (`env/clone_third_party.sh`).
  3. IL 데이터 수집 단계에서 시뮬을 크게 바꿔야 하면 C++ 시뮬을 fork 하지 않고 `sim/` 에 우리 Python 시뮬을 작성한다. C++ 시뮬은 원본 그대로 기준용으로 둔다.
  4. fork 는 외부 코드의 버그를 고쳐야 할 때만 한다.
- **근거**
  - `third_party/` 는 git 에서 제외되므로 안에서 고친 내용은 레포에 남지 않고, 다른 PC 에서 재현되지 않는다.
  - Unitree 저장소는 자주 바뀐다 (R1 모델은 2026-05 추가). 최신을 받으면 오늘 확인한 동작·관절 구성이 깨질 수 있다.
  - 원본을 유지하면 문제가 생겼을 때 외부 코드 문제인지 우리 코드 문제인지 나눌 수 있다.
  - C++ 시뮬은 설정 파일 경로를 지정하는 옵션이 없고 `simulate/config.yaml` 만 읽는다. 고무줄(`enable_elastic_band`) 은 CLI 옵션도 없다.
  - IL 에 필요한 카메라 렌더링, 물체 랜덤화, 리셋은 C++ 수정보다 Python 이 훨씬 쉽다. 공식 Python 브리지(`unitree_sdk2py_bridge.py`)가 작아 같은 방식으로 직접 작성할 수 있다.
- **영향**
  - 외부 저장소를 올릴 때: 스크립트의 커밋 변경 → 동작 확인 → `docs/setup.md` 버전 표 갱신.
  - `third_party/unitree_mujoco/simulate/config.yaml` 은 실행 때마다 덮어써진다. 그 파일을 에디터에서 열어 두고 저장하면 원래 내용으로 돌아가므로 열지 않는다.
  - lerobot clone 도 설치 버전과 같은 v0.4.1 로 맞췄다 (D3).

## D9. 손 없이 시작: 첫 IL 과제는 잡지 않는 조작 (2026-10-18)

- **결정**: 첫 모방학습 과제는 물체 밀기, 버튼 누르기, 정렬처럼 손이 필요 없는 조작으로 한다. 팔 구성(A5/A7)은 미확정이므로 관절 목록을 설정으로 분리한다.
- **근거**
  - 구매 구성은 R1 EDU, 손/그리퍼 없음 (확정).
  - 팔은 판매처 표기상 5자유도 가능성이 높고, 시뮬 모델도 5자유도라 시뮬과 실기 구성이 거의 같다.
  - 잡지 않는 조작(non-prehensile)은 VLA/IL 연구에서도 흔한 과제이고, 시뮬에서 접촉만으로 재현할 수 있다.
- **영향**
  - 7단계 조작 장면은 테이블 + 밀 수 있는 물체 + 버튼/목표 영역으로 구성한다.
  - LeRobot 행동(action) 공간은 팔 관절(+ 허리)만. 그리퍼 차원 없음.
  - 이후 선택지: 3D 프린팅 말단 장치(고리, 패들, 수동 그리퍼), 상체/전신 동작 모방(RL).

## D10. 시뮬은 당분간 A5 모델, 코드는 A5/A7 공용 (2026-10-18)

- **결정**: 4~6단계는 공식 A5 시뮬 모델로 진행한다. 제어 코드는 `variant="a5" | "a7"` 로 활성 관절을 바꾼다. A7 시뮬 모델은 손목을 실제로 쓰는 7단계 또는 A7 확정 시점에 `sim/` 에 만든다.
- **근거**
  - 구매 R1 은 A7 로 가정하지만 견적서로 확인 전이다.
  - 공식 A7 모델은 상체 URDF(`xr_teleoperate/assets/r1/r1_a7.urdf`)뿐이다. 전신 시뮬 모델과 좌표를 맞추고 모터 파라미터(armature, damping)를 추정하는 검증 작업이 필요하다.
  - A5 와 A7 의 차이는 손목 4개 슬롯(20, 21, 27, 28)뿐이고, 4~6단계(자세 유지, 팔 PD, RViz)에는 영향이 작다.
- **영향**: A7 으로 실행해도 현재 시뮬에서는 손목 4개가 움직이지 않는다.

## D11. RViz 모델은 unitree_ros 의 전신 R1.urdf 를 빌드 때 가공해서 사용 (2026-10-18)

- **결정**: `r1_description` 은 `third_party/unitree_ros/robots/r1_description/R1.urdf` 를 CMake 에서 읽어 메시 경로만 `package://` 로 바꿔 설치한다. 파일을 레포로 복사하지 않는다.
- **근거**
  - xr_teleoperate 의 URDF 는 상체만 있어 다리가 안 보인다. unitree_ros 의 `R1.urdf` 는 전신 + 머리이고, 24개 공통 관절 이름·범위가 시뮬 모델과 일치한다.
  - 복사하면 원본 업데이트를 따라가기 어렵고 레포에 메시(수십 MB)가 들어간다. D8(외부 코드 미수정·커밋 고정)과도 맞다.
  - unitree_ros 는 전체가 크므로 R1 폴더만 sparse checkout 한다 (45 MB).
- **영향**: `r1_description` 빌드 전에 `env/clone_third_party.sh` 가 필요하다 (없으면 CMake 가 안내 메시지와 함께 실패).

## D12. 개발용 시뮬은 Python 시뮬(`r1.sim`), C++ 시뮬은 기준용 (2026-10-18)

- **결정**: 조작 장면과 이후 데이터 수집은 `python -m r1.sim.run` 으로 한다. C++ 시뮬은 원본 그대로 두고 비교용으로 남긴다.
- **근거**
  - C++ 시뮬은 이 PC 에서 실시간의 약 0.6배로 돌아 손 끝 추종에 ~160 ms 지연이 생겼다. Python 시뮬은 RTF 1.00, 팔꿈치 계단 응답이 오프라인 계산과 일치 (54 vs 48 ms).
  - 테이블·상자·카메라·리셋·녹화를 Python 코드로 바로 추가할 수 있다 (D8 의 계획).
  - 같은 DDS 토픽을 쓰므로 제어 코드는 두 시뮬과 실기에서 동일하다.
- **설계 선택**
  - PD 는 최신 명령 + 현재 상태로 **매 물리 스텝** 계산 (공식 Python 시뮬은 명령 도착 시 1회 계산 후 유지).
  - 첫 명령 전에는 기본 자세 유지 (토크 0 이면 팔이 테이블로 떨어져 상자를 친다).
  - 기본 베이스는 골반 고정. 실기에서도 상체 조작 중 하체는 Unitree 기본 균형 제어가 맡을 것으로 보고, 시뮬에서는 고정으로 근사한다.
- **영향**: 화면 녹화는 `--headless`(EGL) 에서만 가능 (뷰어와 GL 컨텍스트 충돌).

## D13. 중력 보상은 양팔 + 허리 (2026-10-18)

- **결정**: 팔을 움직일 때 `tau_ff` 로 양팔과 허리(12, 13) 관절의 중력 토크를 넣는다 (`CartesianArm.comp`).
- **근거**: 팔만 보상하면 팔 무게로 허리 roll 이 0.052 rad 처져 손 끝이 16 mm 밀린다. 허리까지 넣으면 0.4 mm. pinocchio 중력 토크와 MuJoCo `qfrc_bias` 는 소수 셋째 자리까지 일치.
- **영향**: 다리는 보상하지 않는다 (골반 고정 상태에서 발이 바닥에 닿아 있어 URDF 고정 베이스 가정과 맞지 않음).

## D14. 시뮬 카메라는 시뮬 프로세스가 직접 ROS2 로 발행 (2026-10-18)

- **결정**: 별도 ROS2 패키지(`r1_sim_camera`, 계획만 있었음)를 만들지 않고 `r1/sim/ros_camera.py` 에서 rclpy 로 발행한다.
- **근거**
  - 렌더링에는 시뮬의 현재 상태(로봇 자세, 상자 위치)가 필요하다. 다른 프로세스로 옮기려면 상태 전체를 따로 보내야 한다.
  - 한 프로세스에서 Unitree DDS(cyclonedds Python)와 rclpy(rmw_cyclonedds_cpp)가 충돌 없이 동작함을 시험으로 확인했다.
  - 뷰어와 오프스크린 렌더러도 한 프로세스에서 동작한다 (종료 시 렌더러를 먼저 닫음).
- **영향**: 실기에서는 카메라 드라이버(ROS2)가 같은 토픽 이름으로 발행하도록 맞추면 이후 코드(기록, 정책)를 그대로 쓸 수 있다.
