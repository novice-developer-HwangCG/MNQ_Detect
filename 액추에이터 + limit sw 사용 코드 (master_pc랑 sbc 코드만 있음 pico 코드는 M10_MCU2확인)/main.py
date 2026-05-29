import os
import sys

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, Int32MultiArray

from PyQt5 import uic
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtGui import QIntValidator

"""
# Receive_Data[0] motor_enable, 0 = 제어 off, 1 = on    // 코드 실행 시 자동으로 1 보내기
# Receive_Data[1] motor_ctrl, 0 = 자동(자동 모터 제어), 1 = 수동(사용자 직접 제어, up),  2 = 수동 (DOWN)
# Receive_Data[2] ch1_set → 0 = 1 채널 on, 1 = 1 채널 off
# Receive_Data[3] ch2_set → 0 = 2 채널 on, 1 = 2 채널 off
# Receive_Data[4] ch3_set → 0 = 3 채널 on, 1 = 3 채널 off
# Receive_Data[5] gain_pwm → 0 ~ 254 = 신호 민감도      // 해당 버전 사용 X, 넣어 놓기만 할 것
# Receive_Data[6] critical_set → 0 ~ 9
# Receive_Data[7] non_cri_set → 0 ~ 9
"""

pre_shot = 0

class M10PublisherNode(Node):
    def __init__(self):
        super().__init__('nuri_ui_publisher')

        self.motor_enable_pub = self.create_publisher(Int32, 'stop', 10)
        self.mnq_manual_pub = self.create_publisher(Int32, 'mnq_manual', 10)
        self.gain_pwm_pub = self.create_publisher(Int32, 'gain_pwm', 10)

        self.ban_channel_pub = self.create_publisher(Int32MultiArray, 'ban_channel', 10)
        self.hit_scenario_pub = self.create_publisher(Int32MultiArray, 'hit_scenario', 10)

        self.pre_shot = 0
        self.shot_signal_sub = self.create_subscription(Int32, 'shot_signal', self.shot_signal_callback, 10)

    def pub_int(self, publisher, value: int, topic_name: str):
        msg = Int32()
        msg.data = value
        print(f"[PUB] {topic_name} = {value}", flush=True)
        publisher.publish(msg)

    def pub_array(self, publisher, values: list[int]):
        msg = Int32MultiArray()
        msg.data = values
        publisher.publish(msg)

    def shot_signal_callback(self, msg: Int32):
        if msg.data != self.pre_shot:
            if msg.data == 0:
                pass
            print(f"shot_signal: {msg.data}", flush=True)
            self.pre_shot = msg.data

class MainWindow(QMainWindow):
    def __init__(self, ui_path: str, ros_node: M10PublisherNode):
        super().__init__()

        if not os.path.isfile(ui_path):
            raise FileNotFoundError(f"UI file not found : {ui_path}")

        uic.loadUi(ui_path, self)
        self.setFixedSize(self.size())

        self.ros_node = ros_node

        self.system_enabled = False
        self.control_mode = "manual"
        self.channel_state = {
            1: True,
            2: True,
            3: True,
        }

        self.enable_btn.clicked.connect(self.on_enable)
        self.disable_btn.clicked.connect(self.on_disable)

        self.ctrl_auto_btn.clicked.connect(self.on_ctrl_auto)

        self.manual_up_btn.clicked.connect(self.on_manual_up)
        self.manual_down_btn.clicked.connect(self.on_manual_down)

        #self.angle_rst_btn.clicked.connect(self.on_angle_reset)

        self.ch1_on_btn.clicked.connect(lambda: self.set_channel(1, True))
        self.ch1_off_btn.clicked.connect(lambda: self.set_channel(1, False))
        self.ch2_on_btn.clicked.connect(lambda: self.set_channel(2, True))
        self.ch2_off_btn.clicked.connect(lambda: self.set_channel(2, False))
        self.ch3_on_btn.clicked.connect(lambda: self.set_channel(3, True))
        self.ch3_off_btn.clicked.connect(lambda: self.set_channel(3, False))

        # 사용자 입력칸
        #v = QIntValidator(0, 254, self)
        #self.gain_line.setValidator(v)

        # 버튼 클릭 시 값 적용
        #self.gain_apply_btn.clicked.connect(self.gain_pwm_set)

        self.update_ui()

    def update_ui(self):
        manual_enable = self.system_enabled
        self.manual_up_btn.setEnabled(manual_enable)
        self.manual_down_btn.setEnabled(manual_enable)

    # enable / disable
    def on_enable(self):
        self.system_enabled = True
        self.ros_node.pub_int(self.ros_node.motor_enable_pub, 1, 'motor_enable')
        self.update_ui()

    def on_disable(self):
        self.system_enabled = False
        self.ros_node.pub_int(self.ros_node.motor_enable_pub, 0, 'motor_enable')
        self.update_ui()

    # auto / manual
    def on_ctrl_auto(self):
        self.control_mode = "auto"
        self.ros_node.pub_int(self.ros_node.mnq_manual_pub, 0, 'mnq_manual')
        self.update_ui()

    def on_manual_up(self):
        if not self.system_enabled:
            return

        self.control_mode = "manual"
        self.update_ui()
        self.ros_node.pub_int(self.ros_node.mnq_manual_pub, 1, 'mnq_manual')

    def on_manual_down(self):
        if not self.system_enabled:
            return

        self.control_mode = "manual"
        self.update_ui()
        self.ros_node.pub_int(self.ros_node.mnq_manual_pub, 2, 'mnq_manual')

    # # manual control
    # def on_manual_up(self):
    #     if not self.system_enabled:
    #         return

    #     self.control_mode = "manual"
    #     self.update_ui()
    #     self.ros_node.pub_int(self.ros_node.mnq_manual_pub, 1, 'mnq_manual')

    # def on_manual_down(self):
    #     if not self.system_enabled:
    #         return

    #     self.control_mode = "manual"
    #     self.update_ui()
    #     self.ros_node.pub_int(self.ros_node.mnq_manual_pub, 2, 'mnq_manual')

    # angle reset
    # def on_angle_reset(self):
    #     self.ros_node.pub_int(self.ros_node.angle_rst_pub, 1, 'angle_rst')

    #     # 100ms 뒤 다시 0으로 복귀
    #     QTimer.singleShot(100,lambda: self.ros_node.pub_int(self.ros_node.angle_rst_pub, 0, 'angle_rst'))

    # channel control
    def set_channel(self, ch: int, state: bool):
        self.channel_state[ch] = state

        values = [
            0 if self.channel_state[1] else 1,
            0 if self.channel_state[2] else 1,
            0 if self.channel_state[3] else 1,
        ]
        self.ros_node.pub_array(self.ros_node.ban_channel_pub, values)

        self.update_ui()

    # def gain_pwm_set(self):
    #     text = self.gain_line.text().strip()
    #     if not text:
    #         print("gain_line is empty")
    #         return

    #     gain_value = int(text)

    #     self.ros_node.pub_int(self.ros_node.gain_pwm_pub, gain_value, 'gain_pwm')
        # print(f"gain applied: {gain_value}")


def main():
    rclpy.init(args=sys.argv)

    app = QApplication(sys.argv)
    ros_node = M10PublisherNode()
    window = MainWindow("mainwindow_for_act.ui", ros_node)
    window.show()

    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0.0))
    ros_timer.start(10)

    exit_code = app.exec_()

    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
