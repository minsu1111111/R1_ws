"""시뮬 ↔ DDS 연결 (실기와 같은 rt/lowcmd, rt/lowstate, unitree_hg).

공식 Python 시뮬(unitree_sdk2py_bridge.py)과 다른 점:
- 명령이 올 때 토크를 한 번 계산해 유지하지 않고, **최신 명령 + 현재 관절 상태로 매 물리 스텝 PD 를 다시 계산**한다.
  (C++ 시뮬, 실제 모터 드라이버와 같은 방식)
- 첫 명령을 받기 전에는 기본 자세(모든 관절 0)를 기본 게인으로 유지한다. 토크 0 으로 두면 팔이 테이블로 떨어져
  제어 시작 시 상자를 쳐 버린다 (docs/troubleshooting.md).
"""

import threading

import mujoco
import numpy as np
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_

from r1.interface.joints import DEFAULT_KD, DEFAULT_KP


class SimBridge:
    def __init__(self, model: mujoco.MjModel, domain_id: int = 1, interface: str = "lo"):
        self.m = model
        self.nu = model.nu
        n = self.nu
        self._kp, self._kd = np.zeros(n), np.zeros(n)
        self._q, self._dq, self._tau = np.zeros(n), np.zeros(n), np.zeros(n)
        self._lock = threading.Lock()
        self.cmd_count = 0

        # 센서 주소 (R1_C++.xml: 관절 q, dq, tau 순서 뒤에 IMU)
        self._adr = {name: model.sensor(name).adr[0] for name in ("imu_quat", "imu_gyro", "imu_acc")}

        ChannelFactoryInitialize(domain_id, interface)
        self._state = unitree_hg_msg_dds__LowState_()
        self._pub = ChannelPublisher("rt/lowstate", LowState_)
        self._pub.Init()
        self._sub = ChannelSubscriber("rt/lowcmd", LowCmd_)
        self._sub.Init(self._on_cmd, 0)  # queueLen=0: DDS 수신 스레드에서 바로 호출

    def _on_cmd(self, msg: LowCmd_):
        mc = msg.motor_cmd
        n = self.nu
        kp = np.array([mc[i].kp for i in range(n)])
        kd = np.array([mc[i].kd for i in range(n)])
        q = np.array([mc[i].q for i in range(n)])
        dq = np.array([mc[i].dq for i in range(n)])
        tau = np.array([mc[i].tau for i in range(n)])
        with self._lock:
            self._kp, self._kd, self._q, self._dq, self._tau = kp, kd, q, dq, tau
            self.cmd_count += 1

    def compute_ctrl(self, d: mujoco.MjData) -> np.ndarray:
        n = self.nu
        q_now = d.sensordata[:n]
        dq_now = d.sensordata[n:2 * n]
        with self._lock:
            if self.cmd_count == 0:
                return DEFAULT_KP[:n] * (0.0 - q_now) - DEFAULT_KD[:n] * dq_now
            return self._tau + self._kp * (self._q - q_now) + self._kd * (self._dq - dq_now)

    def publish_state(self, d: mujoco.MjData):
        s, n, sd = self._state, self.nu, d.sensordata
        for i in range(n):
            ms = s.motor_state[i]
            ms.q = float(sd[i])
            ms.dq = float(sd[n + i])
            ms.tau_est = float(sd[2 * n + i])
        a = self._adr
        s.imu_state.quaternion = [float(x) for x in sd[a["imu_quat"]:a["imu_quat"] + 4]]
        s.imu_state.gyroscope = [float(x) for x in sd[a["imu_gyro"]:a["imu_gyro"] + 3]]
        s.imu_state.accelerometer = [float(x) for x in sd[a["imu_acc"]:a["imu_acc"] + 3]]
        s.tick = int(d.time * 1000)  # 시뮬 시간 [ms]
        self._pub.Write(s)
