#!/usr/bin/python3
import time
import serial
import math
import rclpy

from rclpy.node import Node
from std_msgs.msg import Int32, Int32MultiArray, String, Float32
from serial.serialutil import SerialException


class MNQController(Node):
    def __init__(self):
        super().__init__('mnq_control')

        # ------------------ Pico UART1 연결용 ------------------
        self.mnq_ser = serial.Serial(
            port="/dev/ttyMNQ",
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.01
        )
        self.mnq_ser.reset_input_buffer()

        self.current_time = self.get_clock().now()

        # ------------------ GUI → ROS2 pub 수신용 상태 10바이트 ------------------
        """
        # Receive_Data[0] motor_enable, 0 = 제어 off, 1 = on    // 코드 실행 시 자동으로 1 보내기
        # Receive_Data[1] motor_ctrl, 0 = 자동(자동 모터 제어), 1 = 수동(사용자 직접 제어 / UP), 2 = 수동 (DOWN) / 3 = target_mode
        # Receive_Data[2] ch1_set → 0 = 1 채널 on, 1 = 1 채널 off
        # Receive_Data[3] ch2_set → 0 = 2 채널 on, 1 = 2 채널 off
        # Receive_Data[4] ch3_set → 0 = 3 채널 on, 1 = 3 채널 off
        # Receive_Data[5] gain_pwm → 0 ~ 254 = 신호 민감도
        # Receive_Data[6] critical_set → 0 ~ 9
        # Receive_Data[7] non_cri_set → 0 ~ 9
        # Receive_Data[8] mnq_rst → 0 = None, 1 = zero move     // 현재 각도 읽고 UP상태 각도로 이동 (up상태에서 MNQ가 기울어 질 시)
        """

        # 기본값:
        # 제어 on, 자동, ch1 on, ch2 on, ch3 on, gain, cri, non_cri
        self.data = bytearray([1, 0, 0, 0, 0, 65, 1, 1, 0])

        # ------------------ subscription 추가 ------------------
        # self.create_subscription(Int32, 'stop', self.stopper_callback, 10)

        self.create_subscription(Int32, 'mnq_manual', self.mnq_manual_callback, 10)
        self.create_subscription(Int32MultiArray, 'ban_channel', self.ban_channel_callback, 10)
        self.create_subscription(Int32MultiArray, 'hit_scenario', self.hit_scenario_callback, 10)
        self.create_subscription(Int32, 'gain_pwm', self.gain_pwm_set_callback, 10)
        self.create_subscription(Int32, 'mnq_rst', self.mnq_zero_rst_callback, 10)

        self.shot_signal_pub = self.create_publisher(Int32, 'shot_signal', 10)
        self.angle_pub = self.create_publisher(Float32, 'angle', 10)

        self.motor_ctrl = 0
        self.ban_channel = [0,0,0]
        self.hit_scenario = [0,0]

        self.body_shot_time = 0
        self.head_shot_time = 0
        self.body_topic = 0

        # self.stop = False
        self.ch1_set = 0
        self.ch2_set = 0
        self.ch3_set = 0
        self.gain = 65

        self.mnq_rst = 0

        self.cri_set = 1
        self.non_cri_set = 1
        self.pre_angle = 0.0
        self.pre_shot_sig = None

    """ ----------------- PUB&SUB ---------------- """
    # def stopper_callback(self, msg:Int32):
    #     if msg.data == 0:
    #         self.stop = True
    #     elif msg.data == 1:
    #         self.stop = False

    def mnq_manual_callback(self, msg: Int32):
        if msg.data == 0:
            self.motor_ctrl = 0
        elif msg.data == 1:
            self.motor_ctrl = 1
        elif msg.data == 2:
            self.motor_ctrl = 2
        elif msg.data == 3:
            self.motor_ctrl = 3

    def ban_channel_callback(self, msg):
        # ui 기준 좌 = ch3, 중앙 = ch1, 우 = ch2
        self.ch1_set = msg.data[1]
        self.ch2_set = msg.data[2]
        self.ch3_set = msg.data[0]

    def hit_scenario_callback(self, msg):
        self.cri_set = msg.data[0]
        self.non_cri_set = msg.data[1]
        #print(f"cri_set: {self.cri_set}, non_cri_set: {self.non_cri_set}")

    def mnq_zero_rst_callback(self, msg: Int32):
        self.mnq_rst = int(msg.data)

    def gain_pwm_set_callback(self, msg: Int32):
        self.gain = int(msg.data)

    # def jb_callback(self, msg: String):
    #     if msg.data == 'j':
    #         self.jb = 1      # +5도 이동
    #     elif msg.data == 'b':
    #         self.jb = 2      # -5도 이동
    #     else:
    #         self.jb = 0

    """ ----------- CONNECTION WITH UI PICO (UART1) ----------- """
    def uart_tx(self):
        data = self.data

        data[0] = 1
        data[1] = self.motor_ctrl
        data[2] = self.ch1_set
        data[3] = self.ch2_set
        data[4] = self.ch3_set
        data[5] = self.gain
        data[6] = self.cri_set
        data[7] = self.non_cri_set
        data[8] = self.mnq_rst
        #print(self.gain)
        self.data = data
        frame = bytes([0xFF]) + bytes(self.data)
        self.mnq_ser.write(frame)
        #self.mnq_rst = 0

    def uart_rx(self):
        if self.mnq_ser.in_waiting > 0:
            try:
                line = self.mnq_ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    return
                
                #print(line)

                shot, angle = line.split(',')
                
                shot = int(shot)
                msg = Int32()
                msg.data = shot

                angle = float(angle)
                angle_msg = Float32()
                angle_msg.data = angle
                #print(angle_msg)
                
                if shot != self.pre_shot_sig:
                    print(f"shot: {shot}")
                    self.shot_signal_pub.publish(msg)
                    self.pre_shot_sig = shot

                if angle != self.pre_angle:
                    self.angle_pub.publish(angle_msg)
                    self.pre_angle = angle

            except SerialException as e:
                print(f"MCU2 recv error: {e}")
            except ValueError:
                print(f"MCU2 parse error: {line}")
        else:
            #print("MCU2 Cant READ")
            pass

    """ ------------------ MAIN ----------------- """
    def run_loop(self):
        self.get_logger().info("Start mcu2")

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            self.uart_tx()
            self.uart_rx()

            time.sleep(0.01)

def main():
    rclpy.init()
    mc = MNQController()

    try:
        mc.run_loop()
    except KeyboardInterrupt:
        try:
            mc.mnq_ser.write(bytes([0xFF, 0, 0, 0, 0, 0, 65, 1, 1, 0]))
        except Exception:
            pass
    finally:
        try:
            if mc.mnq_ser.is_open:
                mc.mnq_ser.close()
        except Exception:
            pass

        mc.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
