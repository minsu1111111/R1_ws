"""R1 저수준 제어 래퍼 (unitree_sdk2_python, unitree_hg).

시뮬과 실기에서 같은 코드로 동작한다. 차이는 domain_id / interface 뿐이다.

    robot = R1LowLevel(domain_id=1, interface="lo", variant="a5")
    robot.start()
    robot.set_target(q_des)          # 제어 스레드가 2 ms 마다 LowCmd 로 보냄
    robot.damp()                     # 비상정지: 위치 제어를 끄고 감쇠만 남김

안전 장치
- 관절 범위 클램프 (JOINT_LIMITS, margin 만큼 안쪽)
- 목표 변화율 제한 (max_joint_speed rad/s): 목표가 갑자기 튀어도 천천히 따라감
- 사용하지 않는 관절(변형에 없는 인덱스)은 kp = kd = tau = 0
- damp() 는 래치된다. 다시 제어하려면 프로그램을 재시작한다.
"""

import threading
import time

import numpy as np
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread

from r1.interface.joints import DEFAULT_KD, DEFAULT_KP, JOINT_LIMITS, NUM_SIM_MOTORS, VARIANTS

MODE_PR = 0  # 발목 pitch/roll 직렬 제어 (G1 예제와 동일)


class R1LowLevel:
    def __init__(
        self,
        domain_id: int = 1,
        interface: str = "lo",
        variant: str = "a5",
        control_dt: float = 0.002,
        max_joint_speed: float = 1.0,
        limit_margin: float = 0.05,
        damping_kd: float = 2.0,
    ):
        self.domain_id = domain_id
        self.interface = interface
        self.variant = VARIANTS[variant]
        self.control_dt = control_dt
        self.max_step = max_joint_speed * control_dt
        self.damping_kd = damping_kd

        n = NUM_SIM_MOTORS
        self.active = np.zeros(n, dtype=bool)
        self.active[list(self.variant.active)] = True
        self.q_lo = JOINT_LIMITS[:, 0] + limit_margin
        self.q_hi = JOINT_LIMITS[:, 1] - limit_margin

        self.kp = DEFAULT_KP.copy()
        self.kd = DEFAULT_KD.copy()
        self._q_goal = None   # 사용자가 준 목표
        self._q_cmd = None    # 변화율 제한을 거쳐 실제로 보내는 목표
        self._tau_ff = np.zeros(n)
        self._damped = False

        self._state = None
        self._mode_machine = 0
        self._state_event = threading.Event()
        self._lock = threading.Lock()
        self._cmd = unitree_hg_msg_dds__LowCmd_()
        self._crc = CRC()

    # ---------------------------------------------------------------- 시작
    def start(self, timeout: float = 5.0):
        ChannelFactoryInitialize(self.domain_id, self.interface)
        self._pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self._pub.Init()
        self._sub = ChannelSubscriber("rt/lowstate", LowState_)
        self._sub.Init(self._on_state, 10)

        if not self._state_event.wait(timeout):
            raise TimeoutError(
                f"rt/lowstate 를 {timeout}s 동안 받지 못함 (domain={self.domain_id}, if={self.interface})"
            )
        # 현재 자세를 첫 목표로 → 시작 순간 로봇이 튀지 않는다
        q0 = self.q()
        with self._lock:
            self._q_goal = q0.copy()
            self._q_cmd = q0.copy()

        self._thread = RecurrentThread(interval=self.control_dt, target=self._control_step, name="r1_control")
        self._thread.Start()

    # ---------------------------------------------------------------- 상태
    def _on_state(self, msg: LowState_):
        self._state = msg
        if not self._state_event.is_set():
            self._mode_machine = msg.mode_machine
            self._state_event.set()

    def q(self) -> np.ndarray:
        return np.array([self._state.motor_state[i].q for i in range(NUM_SIM_MOTORS)])

    def dq(self) -> np.ndarray:
        return np.array([self._state.motor_state[i].dq for i in range(NUM_SIM_MOTORS)])

    def tau(self) -> np.ndarray:
        return np.array([self._state.motor_state[i].tau_est for i in range(NUM_SIM_MOTORS)])

    def imu_quat(self) -> np.ndarray:
        return np.array(self._state.imu_state.quaternion)  # w, x, y, z

    # ---------------------------------------------------------------- 명령
    def set_target(self, q_des: np.ndarray, tau_ff: np.ndarray | None = None):
        """29칸 목표 자세. 범위 밖 값은 잘리고, 변화율 제한을 거쳐 전달된다."""
        q_des = np.clip(np.asarray(q_des, dtype=float), self.q_lo, self.q_hi)
        with self._lock:
            self._q_goal = q_des
            if tau_ff is not None:
                self._tau_ff = np.asarray(tau_ff, dtype=float)

    def set_gains(self, kp: np.ndarray | None = None, kd: np.ndarray | None = None):
        with self._lock:
            if kp is not None:
                self.kp = np.asarray(kp, dtype=float)
            if kd is not None:
                self.kd = np.asarray(kd, dtype=float)

    def damp(self):
        """비상정지. 위치 제어를 끄고 속도 감쇠만 남긴다 (래치)."""
        with self._lock:
            self._damped = True

    @property
    def damped(self) -> bool:
        return self._damped

    def tracking_error(self) -> np.ndarray:
        """활성 관절의 (보낸 목표 - 현재 위치)."""
        with self._lock:
            q_cmd = self._q_cmd.copy()
        return (q_cmd - self.q())[self.active]

    # ---------------------------------------------------------------- 제어 스레드
    def _control_step(self):
        with self._lock:
            step = np.clip(self._q_goal - self._q_cmd, -self.max_step, self.max_step)
            self._q_cmd = self._q_cmd + step
            q_cmd, kp, kd, tau_ff, damped = self._q_cmd, self.kp, self.kd, self._tau_ff, self._damped

        cmd = self._cmd
        cmd.mode_pr = MODE_PR
        cmd.mode_machine = self._mode_machine
        for i in range(NUM_SIM_MOTORS):
            m = cmd.motor_cmd[i]
            m.dq = 0.0
            if not self.active[i]:
                m.mode, m.q, m.kp, m.kd, m.tau = 0, 0.0, 0.0, 0.0, 0.0
            elif damped:
                m.mode, m.q, m.kp, m.kd, m.tau = 1, 0.0, 0.0, self.damping_kd, 0.0
            else:
                m.mode, m.q, m.kp, m.kd, m.tau = 1, float(q_cmd[i]), float(kp[i]), float(kd[i]), float(tau_ff[i])
        cmd.crc = self._crc.Crc(cmd)
        self._pub.Write(cmd)


def wait(seconds: float, robot: R1LowLevel):
    """damp() 가 걸리면 바로 빠져나오는 sleep."""
    end = time.time() + seconds
    while time.time() < end and not robot.damped:
        time.sleep(0.01)
