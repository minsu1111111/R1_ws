# R1 MuJoCo 모델 분석

- 파일: `third_party/unitree_mujoco/unitree_robots/r1/R1_C++.xml` (장면: `scene.xml`)
- 확인 기준: unitree_mujoco 커밋 `c598f10` ("add r1 robot", 2026-05-18), 확인일 2026-10-18
- 확인 방법: `mujoco.MjModel`로 로드해 actuator / joint / sensor 를 출력

## 요약

| 항목 | 값 |
|---|---|
| actuator(모터 슬롯) | 29 (`unitree_hg` LowCmd 배열 크기와 동일) |
| **실제로 움직이는 관절** | **24** (다리 12, 허리 2, 팔 5×2) |
| 더미 관절 | 5 (`waist_pitch`, 좌우 `wrist_pitch`, `wrist_yaw`) |
| 팔 | 5자유도 (shoulder pitch/roll/yaw, elbow, wrist roll) → A5 형태 |
| 머리 관절 | 없음 |
| 카메라 | 없음 (조작 장면에서 직접 추가해야 함) |
| 손 / 그리퍼 | 없음 |
| 기본 timestep | 0.002 s |
| 총 질량 | 약 28.9 kg |
| 루트 바디 | `pelvis` (floating base), 상체 `torso_link` |
| 센서 | 관절별 pos / vel / torque, `imu_quat`, `imu_gyro`, `imu_acc`, `frame_pos`, `frame_vel` |

## 구매 하드웨어와의 관계 (2026-10-18 기준)

| 항목 | 알고 있는 것 | 상태 |
|---|---|---|
| 모델 | R1 **EDU** | 확정 |
| 손 / 그리퍼 | **없음** (구매 안 함) | 확정 |
| 팔 | **A7(7자유도)로 가정**. 구매 시 판매처 실물을 본 기억 기준 (2026-10-18) | 견적서·계약서 모델명으로 최종 확인 필요 |
| 머리 | 2자유도 (실기 모터 29, 30). 시뮬 모델에는 없음 | 원격조작 코드로 확인 |

- 공식 시뮬 모델(`unitree_mujoco`, `unitree_rl_mjlab`)은 둘 다 **A5** 이고 머리가 없다. A7 실기와 맞추려면 `sim/` 에 A7 모델을 만들어야 한다.
- 코드는 사용할 관절 목록을 설정으로 받게 만들어 A5 / A7 둘 다 지원한다.
- 손이 없으므로 첫 IL 과제는 잡지 않는 조작(밀기, 누르기, 정렬 등)으로 한다 → [decisions.md](decisions.md) D9.

## 실기 R1 모터 배열 (xr_teleoperate 기준) vs 시뮬

출처: `third_party/xr_teleoperate/teleop/robot_control/robot_arm.py` (`R1_A5_JointIndex`, `R1_A7_JointIndex`), 커밋 `817fb00c`.

| 항목 | 시뮬 (`R1_C++.xml`) | 실기 (원격조작 코드) |
|---|---|---|
| 모터 배열 크기 | 29 | **35** (A5, A7 동일) |
| 0~11 다리, 13 허리 yaw, 15~19 / 22~26 팔 | 같은 인덱스 | 같은 인덱스 |
| 12 허리 roll | 동작하는 관절 | `kWaistRollNotUsed` (원격조작에서 안 씀. 실기에 없는지는 **미확인**) |
| 14 허리 pitch | 더미 | `NotUsed` |
| 20, 21, 27, 28 손목 pitch/yaw | 더미 | A5: `NotUsed` / **A7: 사용** |
| 29, 30 머리 pitch/yaw | 없음 | **있음** (A5, A7 모두) |
| 31~ | 없음 | `NotUsed` |

- A5 와 A7 은 같은 R1 본체이고 차이는 손목 4개 슬롯뿐이다. 휴머노이드라는 사실만으로는 구분되지 않는다.
- 공식 URDF: `third_party/xr_teleoperate/assets/r1/r1_a5.urdf`, `r1_a7.urdf` (+ `meshes/`) → 4단계 `r1_description` 에 사용.
  **상체만** 있다 (루트 `pelvis_link` → 허리 yaw, 팔, 머리). 다리·허리 roll 없음.

### 공식 R1 A7 URDF 관절 (상체)

| 관절 | 범위 (rad) | effort (N·m) | 시뮬(`R1_C++.xml`)과 비교 |
|---|---|---|---|
| waist_yaw | [-2.62, 2.62] | 60 | 범위 같음, 시뮬 88 |
| shoulder_pitch | [-3.14, 2.09] | 60 | 범위 같음, 시뮬 25 |
| shoulder_roll (L) | [-0.23, 2.48] | 60 | 범위 같음, 시뮬 25 |
| shoulder_yaw | [-1.92, 1.92] | 33 | 범위 같음, 시뮬 25 |
| elbow | [-0.98, 2.19] | 33 | 범위 같음, 시뮬 25 |
| wrist_roll | [-1.92, 1.92] | 33 | 범위 같음, 시뮬 25 |
| **wrist_pitch** | [-1.61, 1.61] | **10** | 시뮬에서는 더미 |
| **wrist_yaw** | [-1.61, 1.61] | **10** | 시뮬에서는 더미 |
| head_pitch | [-0.63, 0.63] | 33 | 시뮬에 없음 |
| head_yaw | [-2.01, 2.01] | 0.86 | 시뮬에 없음 |

- 팔 토크 한계가 시뮬(25)과 URDF(60/33)에서 다르다. 실기 도착 후 확인한다.
- 손목 pitch/yaw 는 10 N·m 로 약하다. 미는 작업에서 손목에 큰 힘을 싣지 않도록 자세를 설계한다.
- 실기 도착 후 확인할 것: `LowState.motor_state` 길이, 허리 roll(12) 동작 여부, 손목 슬롯(20, 21, 27, 28) 응답, 머리(29, 30).

## 더미 관절

`waist_pitch`, `left/right_wrist_pitch`, `left/right_wrist_yaw` 는 로봇에 붙어 있지 않습니다.
질량 1 g 짜리 빈 바디가 월드 기준 높이 20 m 에 따로 있고, 관절 범위·제어 범위가 모두 0 입니다.
G1(29자유도)과 같은 29칸 모터 배열을 쓰기 위해 자리만 채운 것으로 보입니다.

- 이 인덱스(14, 20, 21, 27, 28)에는 명령을 보내도 의미가 없습니다. 제어 코드와 학습 데이터에서 제외합니다.
- 실기 R1이 A7(7자유도 팔)이라면 이 자리가 실제 관절이 될 수 있으니, 실기 도착 후 `LowState`로 다시 확인합니다.

## 관절표 (actuator 인덱스 = LowCmd `motor_cmd[i]` 인덱스)

범위 단위는 rad, 토크 한계 단위는 N·m 입니다.

| idx | 관절 | 범위 | 토크 | 비고 |
|---|---|---|---|---|
| 0 | left_hip_pitch | [-2.93, 2.55] | 88 | |
| 1 | left_hip_roll | [-1.05, 1.75] | 88 | |
| 2 | left_hip_yaw | [-2.74, 2.74] | 88 | |
| 3 | left_knee | [-0.17, 2.43] | 139 | |
| 4 | left_ankle_pitch | [-0.87, 0.58] | 50 | |
| 5 | left_ankle_roll | [-0.26, 0.26] | 50 | |
| 6 | right_hip_pitch | [-2.93, 2.55] | 88 | |
| 7 | right_hip_roll | [-1.75, 1.05] | 88 | |
| 8 | right_hip_yaw | [-2.74, 2.74] | 88 | |
| 9 | right_knee | [-0.17, 2.43] | 139 | |
| 10 | right_ankle_pitch | [-0.87, 0.58] | 50 | |
| 11 | right_ankle_roll | [-0.26, 0.26] | 50 | |
| 12 | waist_roll | [-0.52, 0.52] | 50 | |
| 13 | waist_yaw | [-2.62, 2.62] | 88 | |
| 14 | waist_pitch | - | - | **더미** |
| 15 | left_shoulder_pitch | [-3.14, 2.09] | 25 | |
| 16 | left_shoulder_roll | [-0.23, 2.48] | 25 | |
| 17 | left_shoulder_yaw | [-1.92, 1.92] | 25 | |
| 18 | left_elbow | [-0.98, 2.19] | 25 | |
| 19 | left_wrist_roll | [-1.92, 1.92] | 25 | |
| 20 | left_wrist_pitch | - | - | **더미** |
| 21 | left_wrist_yaw | - | - | **더미** |
| 22 | right_shoulder_pitch | [-3.14, 2.09] | 25 | |
| 23 | right_shoulder_roll | [-2.48, 0.23] | 25 | |
| 24 | right_shoulder_yaw | [-1.92, 1.92] | 25 | |
| 25 | right_elbow | [-0.98, 2.19] | 25 | |
| 26 | right_wrist_roll | [-1.92, 1.92] | 25 | |
| 27 | right_wrist_pitch | - | - | **더미** |
| 28 | right_wrist_yaw | - | - | **더미** |

## G1 과 다른 점 (G1 코드를 참고할 때 주의)

- 허리 순서: R1 은 12 = roll, 13 = yaw, 14 = pitch(더미). G1 29자유도는 12 = yaw, 13 = roll, 14 = pitch.
  `unitree_IL_lerobot` 등 G1 코드를 가져올 때 인덱스를 그대로 쓰면 안 됩니다.
- 팔 토크 한계가 25 N·m 로 작습니다. 무거운 물체 조작이나 빠른 동작에 제약이 있습니다.

## 다시 뽑는 방법

```bash
.venv/bin/python - <<'EOF'
import mujoco
m = mujoco.MjModel.from_xml_path("third_party/unitree_mujoco/unitree_robots/r1/scene.xml")
for i in range(m.nu):
    j = m.joint(m.actuator_trnid[i, 0])
    print(i, j.name, m.jnt_limited[j.id], j.range, m.actuator_ctrlrange[i])
EOF
```
