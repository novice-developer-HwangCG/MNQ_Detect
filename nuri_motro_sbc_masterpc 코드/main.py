#!/usr/bin/python3
import os
import sys

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, Int32MultiArray, Float32

from PyQt5 import uic
from PyQt5.QtCore import QTimer, pyqtSignal
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtGui import QIntValidator


class NuriPublisherNode(Node):
    def __init__(self):
        super().__init__('nuri_ui_publisher')

        # 0518_sbc.py 기준 구독 토픽
        # mnq_manual: 0=auto, 1=manual UP, 2=manual DOWN
        # ban_channel: [ch3, ch1, ch2], 각 값 0=ON, 1=OFF
        # hit_scenario: [critical_set, non_cri_set]
        # mnq_rst: 0=None, 1=zero move
        self.mnq_manual_pub = self.create_publisher(Int32, 'mnq_manual', 10)
        self.ban_channel_pub = self.create_publisher(Int32MultiArray, 'ban_channel', 10)
        self.hit_scenario_pub = self.create_publisher(Int32MultiArray, 'hit_scenario', 10)
        self.mnq_rst_pub = self.create_publisher(Int32, 'mnq_rst', 10)
        self.gain_pub = self.create_publisher(Int32, 'gain_pwm', 10)

        self.angle_callback = None
        self.shot_callback = None

        self.angle_sub = self.create_subscription(Float32, 'angle', self.angle_topic_callback, 10)
        self.shot_sub = self.create_subscription(Int32, 'shot_signal', self.shot_topic_callback, 10)

    def pub_int(self, publisher, value: int):
        msg = Int32()
        msg.data = int(value)
        publisher.publish(msg)

    def pub_array(self, publisher, values):
        msg = Int32MultiArray()
        msg.data = [int(v) for v in values]
        publisher.publish(msg)

    def angle_topic_callback(self, msg: Float32):
        if self.angle_callback is not None:
            self.angle_callback(float(msg.data))

    def shot_topic_callback(self, msg: Int32):
        if self.shot_callback is not None:
            self.shot_callback(int(msg.data))

class MainWindow(QMainWindow):
    angle_received = pyqtSignal(float)
    shot_received = pyqtSignal(int)
    def __init__(self, ui_path: str, ros_node: NuriPublisherNode):
        super().__init__()

        if not os.path.isfile(ui_path):
            raise FileNotFoundError(f"UI file not found: {ui_path}")

        uic.loadUi(ui_path, self)
        self.setFixedSize(self.size())

        self.ros_node = ros_node
        self.angle_received.connect(self.update_angle_data)
        self.ros_node.angle_callback = self.angle_received.emit

        self.shot_received.connect(self.update_shot_data)
        self.ros_node.shot_callback = self.shot_received.emit

        # UI 내부 동작 허용 플래그
        # 현재 SBC는 motor_enable 토픽을 구독하지 않으므로 enable/disable은 UI 버튼 잠금 용도
        self.system_enabled = False

        # 채널 상태: True=ON, False=OFF
        # SBC 전송값: ON=0, OFF=1
        self.channel_state = {
            1: True,
            2: True,
            3: True,
        }

        self.cri_set = 1
        self.non_cri_set = 1
        self.gain_pwm = 65

        self._connect_signals()
        self._setup_validators()
        self.update_ui()

    # ------------------------------------------------------------
    # Init helpers
    # ------------------------------------------------------------
    def _connect_signals(self):
        self.enable_btn.clicked.connect(self.on_enable)
        self.disable_btn.clicked.connect(self.on_disable)

        self.ctrl_auto_btn.clicked.connect(self.on_ctrl_auto)

        self.manual_up_btn.clicked.connect(self.on_manual_up)
        self.manual_down_btn.clicked.connect(self.on_manual_down)

        self.ch1_on_btn.clicked.connect(lambda: self.set_channel(1, True))
        self.ch1_off_btn.clicked.connect(lambda: self.set_channel(1, False))
        self.ch2_on_btn.clicked.connect(lambda: self.set_channel(2, True))
        self.ch2_off_btn.clicked.connect(lambda: self.set_channel(2, False))
        self.ch3_on_btn.clicked.connect(lambda: self.set_channel(3, True))
        self.ch3_off_btn.clicked.connect(lambda: self.set_channel(3, False))

        self.cri_edit.editingFinished.connect(self.publish_hit_scenario_from_ui)
        self.noncri_edit.editingFinished.connect(self.publish_hit_scenario_from_ui)

        self.mnq_rst_btn.clicked.connect(self.on_mnq_reset)

        self.gain_edit.editingFinished.connect(self.publish_gain_pwm_from_ui)
        if hasattr(self, 'gain_apply_btn'):
            self.gain_apply_btn.clicked.connect(self.publish_gain_pwm_from_ui)

    def _setup_validators(self):
        scenario_validator = QIntValidator(0, 9, self)
        self.cri_edit.setValidator(scenario_validator)
        self.noncri_edit.setValidator(scenario_validator)

        gain_validator = QIntValidator(0, 254, self)
        self.gain_edit.setValidator(gain_validator)

    # ------------------------------------------------------------
    # UI state
    # ------------------------------------------------------------
    def update_ui(self):
        enabled = self.system_enabled

        self.ctrl_auto_btn.setEnabled(enabled)
        self.manual_up_btn.setEnabled(enabled)
        self.manual_down_btn.setEnabled(enabled)

        self.ch1_on_btn.setEnabled(enabled)
        self.ch1_off_btn.setEnabled(enabled)
        self.ch2_on_btn.setEnabled(enabled)
        self.ch2_off_btn.setEnabled(enabled)
        self.ch3_on_btn.setEnabled(enabled)
        self.ch3_off_btn.setEnabled(enabled)

        self.cri_edit.setEnabled(enabled)
        self.noncri_edit.setEnabled(enabled)
        self.mnq_rst_btn.setEnabled(enabled)

        self.gain_edit.setEnabled(enabled)
        if hasattr(self, 'gain_apply_btn'):
            self.gain_apply_btn.setEnabled(enabled)

    def _status(self, text: str):
        if hasattr(self, 'statusbar') and self.statusbar is not None:
            self.statusbar.showMessage(text, 2000)

    # ------------------------------------------------------------
    # Enable / disable
    # ------------------------------------------------------------
    def on_enable(self):
        self.system_enabled = True
        self.update_ui()

        # enable 시 현재 UI 설정값을 SBC로 1회 동기화
        self.publish_ban_channel()
        self.publish_hit_scenario_from_ui()
        self.publish_gain_pwm_from_ui()
        self.on_ctrl_auto()

        self._status('Enabled')

    def on_disable(self):
        self.system_enabled = False

        # 자동모드 값만 전송
        self.ros_node.pub_int(self.ros_node.mnq_manual_pub, 0)

        self.update_ui()
        self._status('Disabled')

    # ------------------------------------------------------------
    # Control command
    # ------------------------------------------------------------
    def on_ctrl_auto(self):
        if not self.system_enabled:
            return

        # SBC mnq_manual_callback 기준: 0 = auto
        self.ros_node.pub_int(self.ros_node.mnq_manual_pub, 0)
        self._status('Auto command sent')

    def on_manual_up(self):
        if not self.system_enabled:
            return

        # SBC mnq_manual_callback 기준: 1 = manual UP
        self.ros_node.pub_int(self.ros_node.mnq_manual_pub, 1)
        self._status('Manual UP command sent')

    def on_manual_down(self):
        if not self.system_enabled:
            return

        # SBC mnq_manual_callback 기준: 2 = manual DOWN
        self.ros_node.pub_int(self.ros_node.mnq_manual_pub, 2)
        self._status('Manual DOWN command sent')

    # ------------------------------------------------------------
    # Channel control
    # ------------------------------------------------------------
    def set_channel(self, ch: int, state: bool):
        if not self.system_enabled:
            return

        self.channel_state[ch] = state
        self.publish_ban_channel()

        state_text = 'ON' if state else 'OFF'
        self._status(f'CH{ch} {state_text}')

    def publish_ban_channel(self):
        # SBC ban_channel_callback 기준:
        # msg.data[0] -> ch3_set
        # msg.data[1] -> ch1_set
        # msg.data[2] -> ch2_set
        ch1_value = 0 if self.channel_state[1] else 1
        ch2_value = 0 if self.channel_state[2] else 1
        ch3_value = 0 if self.channel_state[3] else 1

        self.ros_node.pub_array(
            self.ros_node.ban_channel_pub,
            [ch3_value, ch1_value, ch2_value]
        )

    # ------------------------------------------------------------
    # Hit scenario
    # ------------------------------------------------------------
    def _read_int_edit(self, edit, default_value: int, min_value: int, max_value: int):
        text = edit.text().strip()
        if not text:
            value = default_value
        else:
            try:
                value = int(text)
            except ValueError:
                value = default_value

        if value < min_value:
            value = min_value
        if value > max_value:
            value = max_value

        edit.setText(str(value))
        return value

    def update_angle_data(self, angle: float):
        if hasattr(self, 'angle_data'):
            self.angle_data.setText(f'{angle:.2f}')

    def update_shot_data(self, shot: int):
        if shot == 0:
            return

        if hasattr(self, 'hit_ch'):
            self.hit_ch.setText(str(shot))

    def publish_hit_scenario_from_ui(self):
        self.cri_set = self._read_int_edit(self.cri_edit, 1, 0, 9)
        self.non_cri_set = self._read_int_edit(self.noncri_edit, 1, 0, 9)

        # SBC hit_scenario_callback 기준:
        # msg.data[0] -> cri_set
        # msg.data[1] -> non_cri_set
        self.ros_node.pub_array(
            self.ros_node.hit_scenario_pub,
            [self.cri_set, self.non_cri_set]
        )

        self._status(f'Hit scenario: cri={self.cri_set}, non_cri={self.non_cri_set}')

    def publish_gain_pwm_from_ui(self):
        if not self.system_enabled:
            return

        self.gain_pwm = self._read_int_edit(
            self.gain_edit,
            65,
            0,
            254
        )

        self.ros_node.pub_int(
            self.ros_node.gain_pub,
            self.gain_pwm
        )

        self._status(f'Gain PWM: {self.gain_pwm}')

    # ------------------------------------------------------------
    # MNQ reset
    # ------------------------------------------------------------
    def on_mnq_reset(self):
        if not self.system_enabled:
            return

        # SBC/Pico 프레임 기준 mnq_rst는 순간 명령이므로 1 전송 후 0 복귀
        self.ros_node.pub_int(self.ros_node.mnq_rst_pub, 1)
        QTimer.singleShot(200, lambda: self.ros_node.pub_int(self.ros_node.mnq_rst_pub, 0))

        self._status('MNQ reset command sent')


def main():
    rclpy.init(args=sys.argv)

    app = QApplication(sys.argv)
    ros_node = NuriPublisherNode()

    # 실행 위치에 mainwindow_for_nuri.ui가 있다고 가정
    window = MainWindow('mainwindow_for_nuri.ui', ros_node)
    window.show()

    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0.0))
    ros_timer.start(10)

    exit_code = app.exec_()

    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
