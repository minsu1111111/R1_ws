# 문서

| 문서 | 내용 |
|---|---|
| [setup.md](setup.md) | 로컬 환경 설치 절차 (실제 진행한 순서 그대로) |
| [architecture.md](architecture.md) | DDS / ROS2 / 시뮬 / 서버 구성과 데이터 흐름 |
| [r1_model.md](r1_model.md) | R1 MuJoCo 모델 분석: 관절표, 더미 관절, 센서, 실기와의 차이 |
| [ros2.md](ros2.md) | ROS2 패키지: r1_description, r1_bridge, RViz 실행 |
| [r1_interface.md](r1_interface.md) | `r1.interface` 저수준 제어 래퍼: 사용법, 안전 장치, 시험 결과 |
| [decisions.md](decisions.md) | 설계 결정 기록 (무엇을, 왜, 근거) |
| [troubleshooting.md](troubleshooting.md) | 겪은 문제와 해결 방법 |

`docs/`는 확정된 내용(설치법, 구조, 결정)을 두는 곳이고,
`notes/`는 공부 정리와 실험 로그처럼 계속 바뀌는 내용을 두는 곳입니다.

## 문서 작성 규칙

- 설치 절차나 버전이 바뀌면 `setup.md`와 `env/constraints.txt`를 같이 고칩니다.
- 무언가를 선택했거나 버렸다면 `decisions.md`에 근거와 함께 한 항목 추가합니다.
- 30분 이상 막혔던 문제는 `troubleshooting.md`에 증상 / 원인 / 해결 순서로 남깁니다.
- 확인한 날짜와 버전(커밋)을 적습니다. 외부 저장소는 자주 바뀝니다.
