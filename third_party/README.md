# third_party

외부 저장소 원본을 두는 곳입니다. git에는 올라가지 않으며 `env/clone_third_party.sh`로 받습니다.
빌드 절차는 [docs/setup.md](../docs/setup.md).
각 저장소는 스크립트에 적힌 **커밋으로 고정**됩니다. 여기 있는 코드는 직접 수정하지 않습니다.
설정은 레포 쪽(`sim/` 등)에 두고 복사하며, 버그 수정이 필요할 때만 fork 합니다. ([docs/decisions.md](../docs/decisions.md) D8)

| 저장소 | 용도 | 위치 |
|---|---|---|
| unitree_mujoco | R1 MuJoCo 시뮬 (실기와 같은 DDS 토픽) | 로컬 |
| unitree_sdk2 | C++ SDK. C++ 시뮬, RL deploy 빌드용 (`unitree_sdk2/install` 에 설치) | 로컬 |
| unitree_sdk2_python | DDS 제어 SDK | 로컬 |
| mujoco_bin | MuJoCo 3.3.6 C 라이브러리 (C++ 시뮬용, git 저장소 아님) | 로컬 |
| unitree_ros2 | ROS2 메시지 (`unitree_hg/go/api`) | 로컬 |
| unitree_ros | R1 URDF (`robots/r1_description`, `r1_a7_description` 만 sparse checkout) | 로컬 |
| xr_teleoperate | 원격조작 참고 (R1 A5/A7 지원) | 로컬 (참고) |
| unitree_IL_lerobot | LeRobot 연동 참고 (G1 기준) | 로컬·서버 |
| unitree_rl_mjlab | RL 학습 + `deploy/robots/r1` | 서버 (학습), 로컬 (deploy 빌드) |
| lerobot | 모방학습 / VLA 학습 | 서버 |
