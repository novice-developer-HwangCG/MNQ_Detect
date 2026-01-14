#!/usr/bin/python3
"""

"""
import time
import serial
import math
import rclpy
from rclpy.node import Node
from serial.serialutil import SerialException

class MNQTest(Node):
    def __init__(self):
        super().__init__('mnq_test')

        self.ser = serial.Serial(port="/dev/ttyUSB0",
                        baudrate=115200,
                        bytesize=serial.EIGHTBITS,
                        parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE,
                        timeout=0.05
        )

        self.rx_buf = b""
        self.ser.reset_input_buffer() 

        self.current_time = self.get_clock().now()
        
        self.data = [0xFF, 0, 0, 0]

        self.mode = 0          # 시작: 자동
        self.manual_state = -1

    """ ----------------- ENCODER ---------------- """
    def read_encoder(self):
        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                hit_s1, hit_s2, hit_s3 = line.split(',')
                print(hit_s1, hit_s2, hit_s3)

            except ValueError as e:
                #rospy.logerr(f"value err : {e}")
                print("value error")
        else:
            print("pico no data")
            time.sleep(0.5)
    
    """ ----------- CONNECTION WITH PICO ----------- """   
    def tx_pico(self):
        data = self.data
        
        if self.stop == True:
            self.ser.write(bytes([0xFF, 0, 0, 0]))
        else:
            self.ser.write(bytes([0xFF, 1, 0, 0]))

            if self.mode == 0:
                # 자동일 때 탄 감지 신호만 받기 (제어 안함)
                self.ser.write(bytes([0xFF, 1, 0, 0]))
            if self.mode == 1:
                self.ser.write(bytes([0xFF, 1, 1, 0]))

                # 수동일 때 up/down 조작
                if self.manual_state == 1:
                    self.ser.write(bytes([0xFF, 1, 1, 1]))
                if self.manual_state == 2:
                    self.ser.write(bytes([0xFF, 0, 1, 2]))
        
            
    """ ------------------ MAIN ----------------- """        
    def run_loop(self):
        self.get_logger().info("get hit signal...")
        
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)  # 콜백 처리
            self.read_encoder()
            
            time.sleep(0.01)

def main():
    rclpy.init()
    mt = MNQTest()
    try:
        mt.run_loop()
    except KeyboardInterrupt:
        try:
            mt.ser.write(bytes([0xFF, 0, 0, 0]))
        except Exception:
            pass
    finally:
        mt.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
