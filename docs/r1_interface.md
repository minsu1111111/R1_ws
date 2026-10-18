# r1.interface: 저수준 제어 래퍼

`r1/interface/` 는 `unitree_sdk2_python` 위에서 R1 관절을 PD 제어하는 래퍼입니다.
시뮬과 실기에서 같은 코드가 동작하고, 바뀌는 것은 `domain_id` / `interface` 뿐입니다.

| 파일 | 내용 |
|---|---|
| `joints.py` | 모터 인덱스, 관절 이름, 관절 범위, 기본 PD 게인, 변형(A5 / A7) |
| `robot.py` | `R1LowLevel`: 상태 읽기, 목표 자세 전송(2 ms 주기), 안전 장치, `damp()` |
| `examples/wave_arm.py` | 기본 자세 → 왼팔 흔들기 → 복귀 → damp |

## 사용법

```python
from r1.interface.robot import R1LowLevel
import numpy as np

robot = R1LowLevel(domain_id=1, interface="lo", variant="a5")   # 실기: domain_id=0, interface="enp3s0"
robot.start()                     # 첫 LowState 수신 대기, 현재 자세를 첫 목표로 잡고 제어 스레드 시작
robot.set_target(np.zeros(29))    # 29칸 목표 자세 [rad]
q, dq = robot.q(), robot.dq()     # 현재 상태
robot.damp()                      # 비상정지
```

예제 실행:

```bash
bash sim/run_r1_sim.sh                                   # 터미널 1 (고무줄 켜짐)
source env/setup_sim.sh                                  # 터미널 2
python -m r1.interface.examples.wave_arm                 # --variant a7 / --domain / --iface
```

## 동작 구조

```
set_target(q_des) ─▶ 관절 범위 클램프 ─▶ q_goal
                                          │  제어 스레드 (2 ms, RecurrentThread)
                                          ▼
                          변화율 제한 (max 1 rad/s) ─▶ q_cmd ─▶ LowCmd (kp, kd, q, tau, CRC) ─▶ rt/lowcmd
```

- `start()` 는 **현재 자세를 첫 목표로** 잡는다. 시작 순간 로봇이 튀지 않는다.
- `mode_machine` 은 첫 LowState 에서 복사한다 (G1 공식 예제와 같은 방식).

## 안전 장치

| 장치 | 기본값 | 의미 |
|---|---|---|
| 관절 범위 클램프 | 범위에서 0.05 rad 안쪽 | 범위를 벗어난 목표는 잘린다 |
| 변화율 제한 | 1.0 rad/s | 목표가 갑자기 바뀌어도 천천히 따라간다 |
| 비활성 관절 | kp = kd = tau = 0 | 변형에 없는 관절(A5 의 손목 pitch/yaw, 허리 pitch)에는 힘을 주지 않는다 |
| `damp()` | kp = 0, kd = 2 | 위치 제어를 끄고 감쇠만 남긴다. **래치**: 다시 제어하려면 재시작 |
| 예제의 Ctrl+C | `damp()` 호출 | |

## 변형 (A5 / A7)

`variant="a5"` 는 다리 12 + 허리 2 + 팔 5×2 = 24 관절, `"a7"` 은 손목 pitch/yaw 4개를 더해 28 관절에 명령을 보낸다.
현재 시뮬 모델은 A5 이므로 A7 로 실행해도 손목 4개는 시뮬에서 움직이지 않는다 ([decisions.md](decisions.md) D10).

## 게인

`joints.py` 의 `DEFAULT_KP / KD` 는 G1 공식 예제(다리 60~100 / 1~2, 팔 40 / 1)를 출발점으로 한 값이며 **R1 용으로 튜닝하지 않았다.**
손목은 토크가 작아(A7 손목 10 N·m) 게인을 낮게 두었다.

## 시험 결과 (2026-10-18, 시뮬, 고무줄 켬)

| 구간 | 최대 추종 오차 (활성 관절) |
|---|---|
| 기본 자세 유지 | 0.028 rad |
| 팔 흔들기 (팔꿈치 ±0.5 rad, 0.5 Hz) | 0.090 rad |
| 기본 자세 복귀 | 0.028 rad |

- 정지 상태 오차는 중력 보상 없이 PD 만 쓰기 때문이다. 필요하면 `set_target(q, tau_ff=...)` 로 중력 보상 토크를 더한다.
- 시뮬은 LowCmd 의 `mode` 필드를 보지 않는다 (`tau + kp(q-q') + kd(dq-dq')` 만 적용). 실기에서는 `mode=1` 이어야 모터가 켜진다.

## 실기에서 추가로 할 일

- 실기는 기본 운동 모드(sport mode)가 켜져 있을 수 있다. G1 예제는 `MotionSwitcherClient` 로 모드를 해제한 뒤 저수준 제어를 한다. R1 에서도 같은지 확인 후 `start()` 에 옵션으로 추가한다.
- 실기 모터 배열은 35칸이고 머리가 29, 30 에 있다 ([r1_model.md](r1_model.md)). 머리 제어가 필요해지면 `NUM_SIM_MOTORS` 를 넘는 인덱스를 지원하도록 확장한다.
- 처음에는 반드시 거치대에 매달고, 게인을 절반으로 낮춰 시작한다.
