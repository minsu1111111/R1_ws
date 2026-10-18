"""터미널 키보드 입력 (cbreak 모드, 백그라운드 스레드).

이 터미널에 포커스가 있을 때만 입력을 받는다. 다른 창에 타이핑해도 로봇이 움직이지 않는다.
Ctrl+C 는 그대로 KeyboardInterrupt 로 동작한다.

키를 누르고 있으면 터미널 자동 반복으로 같은 문자가 계속 들어온다. 처음 반복이 시작되기까지
(보통 0.3 ~ 0.5 s) 공백이 있으므로, 이동 키는 마지막 입력 후 hold_s 동안 눌린 것으로 본다.
"""

import os
import select
import sys
import termios
import threading
import time
import tty


class KeyReader:
    def __init__(self, hold_s: float = 0.25):
        self.hold_s = hold_s
        self._last = {}            # 키 → 마지막 입력 시각 (이동 키)
        self._events = []          # 한 번만 처리할 명령 키
        self._lock = threading.Lock()
        self._fd = sys.stdin.fileno()
        self._old = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while self._running:
            r, _, _ = select.select([self._fd], [], [], 0.05)
            if not r:
                continue
            ch = os.read(self._fd, 1).decode(errors="ignore")
            now = time.time()
            with self._lock:
                self._last[ch] = now
                self._events.append(ch)

    def held(self, key: str) -> bool:
        with self._lock:
            return time.time() - self._last.get(key, 0.0) < self.hold_s

    def pop_events(self) -> list[str]:
        with self._lock:
            ev, self._events = self._events, []
        return ev

    def close(self):
        self._running = False
        self._thread.join(timeout=0.2)
        termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)
