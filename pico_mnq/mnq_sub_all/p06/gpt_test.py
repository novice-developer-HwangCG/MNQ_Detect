#!/usr/bin/env python3
import sys
import serial
from PyQt5 import QtCore, QtWidgets

# ----- Pico RX 프로토콜 (Jetson -> Pico) -----
START = 0xFF
ENABLE_IDLE = 0
ENABLE_ON = 1

MODE_AUTO = 0
MODE_MANUAL = 1

CMD_NONE = 0
CMD_DOWN = 1
CMD_UP = 2


def hit_text(v: int) -> str:
    return "Get" if v == 1 else "None"


class MainWindow(QtWidgets.QWidget):
    def __init__(self, port="/dev/ttyUSB0", baud=115200):
        super().__init__()
        self.setWindowTitle("MNQ UART Controller")

        # 상태
        self.mode = MODE_AUTO          # 시작: 자동
        self.manual_state = CMD_UP   # 수동일 때 표시용(Up/Down)
        self.enable = ENABLE_ON

        # 시리얼
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.0,  # non-blocking
            )
            self.ser.reset_input_buffer()
            self.ser_ok = True
        except Exception as e:
            self.ser = None
            self.ser_ok = False
            print(f"[ERROR] Serial open failed: {e}")

        # UI
        self.btn_mode = QtWidgets.QPushButton("자동")          # 토글 버튼
        self.btn_ud = QtWidgets.QPushButton("Down")           # 토글 버튼 (수동에서만)
        self.lbl_hit = QtWidgets.QLabel("Get hit\nhit_s1 = None\nhit_s2 = None\nhit_s3 = None")
        self.lbl_hit.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignTop)

        self.btn_mode.clicked.connect(self.on_toggle_mode)
        self.btn_ud.clicked.connect(self.on_toggle_ud)

        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.btn_mode)
        layout.addWidget(self.btn_ud)
        layout.addWidget(self.lbl_hit)
        self.setLayout(layout)

        self.apply_ui_state()

        # 타이머: 주기적으로 (1) Pico로 현재 상태 프레임 송신 (2) 자동 모드면 hit 읽어서 라벨 갱신
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(50)  # 20 Hz

    # ---------- UI 업데이트 ----------
    def apply_ui_state(self):
        # 모드 버튼 표시
        self.btn_mode.setText("자동" if self.mode == MODE_AUTO else "수동")

        # Up/Down 버튼은 수동에서만 활성화
        is_manual = (self.mode == MODE_MANUAL)
        self.btn_ud.setEnabled(is_manual)
        self.btn_ud.setText("Up" if self.manual_state == CMD_UP else "Down")

        # 수동으로 바뀌면 hit 라벨은 요구사항대로 "None"으로 초기화(자동일 때만 표시)
        if not is_manual:
            return
        self.set_hit_label(0, 0, 0)

    def set_hit_label(self, s1: int, s2: int, s3: int):
        self.lbl_hit.setText(
            "Get hit\n"
            f"hit_s1 = {hit_text(s1)}\n"
            f"hit_s2 = {hit_text(s2)}\n"
            f"hit_s3 = {hit_text(s3)}"
        )

    # ---------- 버튼 콜백 ----------
    def on_toggle_mode(self):
        self.mode = MODE_MANUAL if self.mode == MODE_AUTO else MODE_AUTO
        self.apply_ui_state()

        # 모드 변경 즉시 Pico에 반영 (cmd는 none)
        self.send_frame(self.enable, self.mode, CMD_NONE)

    def on_toggle_ud(self):
        # 수동에서만 의미 있음
        if self.mode != MODE_MANUAL:
            return

        # Up/Down 토글
        self.manual_state = CMD_UP if self.manual_state == CMD_DOWN else CMD_DOWN
        self.apply_ui_state()

        # 버튼 클릭 시 1회 명령 전송 -> 바로 cmd를 0으로 내려서 같은 명령 재실행도 가능하게 함
        cmd = self.manual_state
        self.send_frame(self.enable, MODE_MANUAL, cmd)
        QtCore.QTimer.singleShot(80, lambda: self.send_frame(self.enable, MODE_MANUAL, CMD_NONE))

    # ---------- UART 송신/수신 ----------
    def send_frame(self, enable: int, mode: int, cmd: int):
        """
        Pico 수신 규격: 0xFF + 3바이트(payload) = 총 4바이트
        [0xFF, enable, mode, cmd]
        """
        if not self.ser_ok:
            return
        try:
            self.ser.write(bytes([START, enable & 0xFF, mode & 0xFF, cmd & 0xFF]))
        except Exception as e:
            print(f"[ERROR] serial write: {e}")

    def read_hit_line(self):
        """
        Pico가 보내는 ASCII 라인: "0,1,0\n" 형태를 읽어 (s1,s2,s3) 반환
        없으면 None
        """
        if not self.ser_ok:
            return None
        try:
            if self.ser.in_waiting <= 0:
                return None
            line = self.ser.readline()
            if not line:
                return None
            txt = line.decode("utf-8", errors="ignore").strip()
            parts = txt.split(",")
            if len(parts) != 3:
                return None
            s1, s2, s3 = (int(parts[0]), int(parts[1]), int(parts[2]))
            return s1, s2, s3
        except Exception:
            return None

    # ---------- 주기 처리 ----------
    def tick(self):
        # 1) Pico에 현재 상태를 계속 송신 (enable=1, mode, cmd=0 또는 수동상태 유지 원하면 cmd=0)
        cmd = CMD_NONE
        self.send_frame(self.enable, self.mode, cmd)

        # 2) 자동 모드일 때만 hit 수신/표시
        if self.mode == MODE_AUTO:
            hit = self.read_hit_line()
            if hit is not None:
                s1, s2, s3 = hit
                self.set_hit_label(s1, s2, s3)

    def closeEvent(self, event):
        # 종료 시 Pico에 idle 보내고 닫기
        try:
            if self.ser_ok:
                self.send_frame(ENABLE_IDLE, MODE_AUTO, CMD_NONE)
                self.ser.close()
        except Exception:
            pass
        event.accept()


def main():
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow(port="/dev/ttyUSB0", baud=115200)
    w.resize(260, 180)
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
