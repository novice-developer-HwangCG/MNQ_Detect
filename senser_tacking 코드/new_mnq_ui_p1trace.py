'''
PUB
- ns/mnq/manuql -> 0: auto / 1 : up / 2: down
- nq/ban_cahnnel -> [0,0,0] -> 리스트 내용 0 : o / 1 : x(마네킹 넘기기) 
- ns/rst -> 0: not / 1 : rst

SUB
- shot_signal -> 1, 2, 3


'''

import sys
import math
import json
from datetime import datetime

from PyQt5.QtWidgets import QMainWindow, QApplication, QLabel, QPushButton, QWidget, QGridLayout, QVBoxLayout, QHBoxLayout, QLineEdit
from PyQt5.QtCore import QTimer

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from rcl_interfaces.srv import SetParametersAtomically
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType


from std_msgs.msg import Int32, Int32MultiArray, Float32, UInt32MultiArray
from nav_msgs.msg import Odometry

class MnqManualUi(QMainWindow):
    def __init__(self, node: Node, executor: SingleThreadedExecutor):
        super().__init__()

        self.node = node
        self.executor = executor

        self.xtgt1_mnq = 0      #자동 모드 0  // 수동모드  1 : 올림  2: 내림
        self.xtgt2_mnq = 0      #자동 모드 0  // 수동모드  1 : 올림  2: 내림
        self.xtgt3_mnq = 0      #자동 모드 0  // 수동모드  1 : 올림  2: 내림

        self.xtgt1_count = 0
        self.xtgt2_count = 0
        self.xtgt3_count = 0
        
        self.xtgt1_log = 0
        self.xtgt2_log = 0
        self.xtgt3_log = 0

        self.xtgt1_log_key = 0
        self.xtgt2_log_key = 0
        self.xtgt3_log_key = 0

        self.xtgt1_log_dict = {}
        self.xtgt2_log_dict = {}
        self.xtgt3_log_dict = {}

        self.xtgt1_channel_list = [0, 0, 0]
        self.xtgt2_channel_list = [0, 0, 0]
        self.xtgt3_channel_list = [0, 0, 0]

        #로봇 총 이동거리
        self.xtgt1_km = 0
        self.xtgt2_km = 0
        self.xtgt3_km = 0

        self.xtgt1_prev_xy = None
        self.xtgt2_prev_xy = None
        self.xtgt3_prev_xy = None

        self.xtgt1_hit_scenario = [1, 1]
        self.xtgt2_hit_scenario = [1, 1]
        self.xtgt3_hit_scenario = [1, 1]

        # 파라미터 설정 변경
        self.xtgt1_lidar_target = "/xtgt1/lidar_filter"
        self.xtgt1_lidar_param_cli = self.node.create_client(
            SetParametersAtomically,
            f"{self.xtgt1_lidar_target}/set_parameters_atomically"
        )
        self.xtgt2_lidar_target = "/xtgt2/lidar_filter"
        self.xtgt2_lidar_param_cli = self.node.create_client(
            SetParametersAtomically,
            f"{self.xtgt2_lidar_target}/set_parameters_atomically"
        )
        self.xtgt3_lidar_target = "/xtgt3/lidar_filter"
        self.xtgt3_lidar_param_cli = self.node.create_client(
            SetParametersAtomically,
            f"{self.xtgt3_lidar_target}/set_parameters_atomically"
        )

        #pub & sub
        self.xtgt1_mnq_pub = self.node.create_publisher(Int32, '/xtgt1/mnq_manual', 10)
        self.xtgt2_mnq_pub = self.node.create_publisher(Int32, '/xtgt2/mnq_manual', 10)
        self.xtgt3_mnq_pub = self.node.create_publisher(Int32, '/xtgt3/mnq_manual', 10)

        self.xtgt1_ban_channel_pub = self.node.create_publisher(Int32MultiArray, '/xtgt1/ban_channel', 10)
        self.xtgt2_ban_channel_pub = self.node.create_publisher(Int32MultiArray, '/xtgt2/ban_channel', 10)
        self.xtgt3_ban_channel_pub = self.node.create_publisher(Int32MultiArray, '/xtgt3/ban_channel', 10)

        self.xtgt1_rst1_pub = self.node.create_publisher(Int32, '/xtgt1/rst', 10)
        self.xtgt2_rst1_pub = self.node.create_publisher(Int32, '/xtgt2/rst', 10)
        self.xtgt3_rst1_pub = self.node.create_publisher(Int32, '/xtgt3/rst', 10)

        self.xtgt1_gain1_pub = self.node.create_publisher(Int32, '/xtgt1/gain_pwm', 10)
        self.xtgt2_gain1_pub = self.node.create_publisher(Int32, '/xtgt2/gain_pwm', 10)
        self.xtgt3_gain1_pub = self.node.create_publisher(Int32, '/xtgt3/gain_pwm', 10)

        self.xtgt1_mnq_rst_pub = self.node.create_publisher(Int32, '/xtgt1/mnq_rst', 10)
        self.xtgt2_mnq_rst_pub = self.node.create_publisher(Int32, '/xtgt2/mnq_rst', 10)
        self.xtgt3_mnq_rst_pub = self.node.create_publisher(Int32, '/xtgt3/mnq_rst', 10)

        self.xtgt1_shot_signal_sub = self.node.create_subscription(Int32, '/xtgt1/shot_signal', self.xtgt1_shot_signal_cb, 10)
        self.xtgt2_shot_signal_sub = self.node.create_subscription(Int32, '/xtgt2/shot_signal', self.xtgt2_shot_signal_cb, 10)
        self.xtgt3_shot_signal_sub = self.node.create_subscription(Int32, '/xtgt3/shot_signal', self.xtgt3_shot_signal_cb, 10)

        self.xtgt1_odom_sub = self.node.create_subscription(Odometry, 'xtgt1/odometry/filtered', self.xtgt1_odom_cb, 10)
        self.xtgt2_odom_sub = self.node.create_subscription(Odometry, 'xtgt2/odometry/filtered', self.xtgt2_odom_cb, 10)
        self.xtgt3_odom_sub = self.node.create_subscription(Odometry, 'xtgt3/odometry/filtered', self.xtgt3_odom_cb, 10)

        self.xtgt1_bat_sub = self.node.create_subscription(Int32, '/xtgt1/bat', self.xtgt1_bat_cb, 10)
        self.xtgt2_bat_sub = self.node.create_subscription(Int32, '/xtgt2/bat', self.xtgt2_bat_cb, 10)
        self.xtgt3_bat_sub = self.node.create_subscription(Int32, '/xtgt3/bat', self.xtgt3_bat_cb, 10)

        self.xtgt1_hit_scenario_pub = self.node.create_publisher(Int32MultiArray, '/xtgt1/hit_scenario', 10)
        self.xtgt2_hit_scenario_pub = self.node.create_publisher(Int32MultiArray, '/xtgt2/hit_scenario', 10)
        self.xtgt3_hit_scenario_pub = self.node.create_publisher(Int32MultiArray, '/xtgt3/hit_scenario', 10)
        
        self.xtgt1_angle_sub = self.node.create_subscription(Float32, '/xtgt1/angle', self.xtgt1_angle_cb, 10)
        self.xtgt2_angle_sub = self.node.create_subscription(Float32, '/xtgt2/angle', self.xtgt2_angle_cb, 10)
        self.xtgt3_angle_sub = self.node.create_subscription(Float32, '/xtgt3/angle', self.xtgt3_angle_cb, 10)

        self.xtgt1_p1_trace_sub = self.node.create_subscription(UInt32MultiArray, '/xtgt1/p1_trace', self.xtgt1_p1_trace_cb, 10)
        self.xtgt2_p1_trace_sub = self.node.create_subscription(UInt32MultiArray, '/xtgt2/p1_trace', self.xtgt2_p1_trace_cb, 10)
        self.xtgt3_p1_trace_sub = self.node.create_subscription(UInt32MultiArray, '/xtgt3/p1_trace', self.xtgt3_p1_trace_cb, 10)

        self.ui()

        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self._spin_ros_once)
        self.ros_timer.start(50)

    def _spin_ros_once(self):
        self.executor.spin_once(timeout_sec=0.0)

    def ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        grid = QGridLayout()
        central_widget.setLayout(grid)

        # 로봇 이름 라벨 -> 1단
        xtgt1_main_hbox = QHBoxLayout()
        self.xtgt1_lbl = QLabel('XTGT1', self)
        self.xtgt1_bat_lbl = QLabel('bat', self)
        self.xtgt1_angle_lbl = QLabel('angle', self)
        xtgt1_main_hbox.addWidget(self.xtgt1_lbl)
        xtgt1_main_hbox.addWidget(self.xtgt1_bat_lbl)
        xtgt1_main_hbox.addWidget(self.xtgt1_angle_lbl)

        xtgt2_main_hbox = QHBoxLayout()
        self.xtgt2_lbl = QLabel('XTGT2', self)
        self.xtgt2_bat_lbl = QLabel('bat', self)
        self.xtgt2_angle_lbl = QLabel('angle', self)
        xtgt2_main_hbox.addWidget(self.xtgt2_lbl)
        xtgt2_main_hbox.addWidget(self.xtgt2_bat_lbl)
        xtgt2_main_hbox.addWidget(self.xtgt2_angle_lbl)

        xtgt3_main_hbox = QHBoxLayout()
        self.xtgt3_lbl = QLabel('XTGT3', self)
        self.xtgt3_bat_lbl = QLabel('bat', self)
        self.xtgt3_angle_lbl = QLabel('angle', self)
        xtgt3_main_hbox.addWidget(self.xtgt3_lbl)
        xtgt3_main_hbox.addWidget(self.xtgt3_bat_lbl)
        xtgt3_main_hbox.addWidget(self.xtgt3_angle_lbl)

        # 마네킹 수동 자동 모드 -> 2단
        xtgt1_mnq_hbox =QHBoxLayout()
        self.xtgt1_auto_btn = QPushButton('AUTO', self)
        self.xtgt1_up_btn = QPushButton('UP', self)
        self.xtgt1_down_btn = QPushButton('DOWN', self)
        self.xtgt1_hidden_btn = QPushButton('HID', self)
        xtgt1_mnq_hbox.addWidget(self.xtgt1_auto_btn)
        xtgt1_mnq_hbox.addWidget(self.xtgt1_up_btn)
        xtgt1_mnq_hbox.addWidget(self.xtgt1_down_btn)
        xtgt1_mnq_hbox.addWidget(self.xtgt1_hidden_btn)

        xtgt2_mnq_hbox =QHBoxLayout()
        self.xtgt2_auto_btn = QPushButton('AUTO', self)
        self.xtgt2_up_btn = QPushButton('UP', self)
        self.xtgt2_down_btn = QPushButton('DOWN', self)
        self.xtgt2_hidden_btn = QPushButton('HID', self)
        xtgt2_mnq_hbox.addWidget(self.xtgt2_auto_btn)
        xtgt2_mnq_hbox.addWidget(self.xtgt2_up_btn)
        xtgt2_mnq_hbox.addWidget(self.xtgt2_down_btn)
        xtgt2_mnq_hbox.addWidget(self.xtgt2_hidden_btn)

        xtgt3_mnq_hbox =QHBoxLayout()
        self.xtgt3_auto_btn = QPushButton('AUTO', self)
        self.xtgt3_up_btn = QPushButton('UP', self)
        self.xtgt3_down_btn = QPushButton('DOWN', self)
        self.xtgt3_hidden_btn = QPushButton('HID', self)
        xtgt3_mnq_hbox.addWidget(self.xtgt3_auto_btn)
        xtgt3_mnq_hbox.addWidget(self.xtgt3_up_btn)
        xtgt3_mnq_hbox.addWidget(self.xtgt3_down_btn)         
        xtgt3_mnq_hbox.addWidget(self.xtgt3_hidden_btn)

        # 채널 무시 -> 3단
        xtgt1_ban_hbox = QHBoxLayout()
        self.xtgt1_ban1_btn = QPushButton('1', self)
        self.xtgt1_ban2_btn = QPushButton('2', self)
        self.xtgt1_ban3_btn = QPushButton('3', self)
        xtgt1_ban_hbox.addWidget(self.xtgt1_ban1_btn)
        xtgt1_ban_hbox.addWidget(self.xtgt1_ban2_btn)
        xtgt1_ban_hbox.addWidget(self.xtgt1_ban3_btn)

        xtgt2_ban_hbox = QHBoxLayout()
        self.xtgt2_ban1_btn = QPushButton('1', self)
        self.xtgt2_ban2_btn = QPushButton('2', self)
        self.xtgt2_ban3_btn = QPushButton('3', self)
        xtgt2_ban_hbox.addWidget(self.xtgt2_ban1_btn)
        xtgt2_ban_hbox.addWidget(self.xtgt2_ban2_btn)
        xtgt2_ban_hbox.addWidget(self.xtgt2_ban3_btn)

        xtgt3_ban_hbox = QHBoxLayout()
        self.xtgt3_ban1_btn = QPushButton('1', self)
        self.xtgt3_ban2_btn = QPushButton('2', self)
        self.xtgt3_ban3_btn = QPushButton('3', self)
        xtgt3_ban_hbox.addWidget(self.xtgt3_ban1_btn)
        xtgt3_ban_hbox.addWidget(self.xtgt3_ban2_btn)
        xtgt3_ban_hbox.addWidget(self.xtgt3_ban3_btn)

        # 알람 리셋 -> 4단
        xtgt1_rst_btn = QPushButton('RST', self)
        xtgt2_rst_btn = QPushButton('RST', self)
        xtgt3_rst_btn = QPushButton('RST', self)

        #SHOT LOG -> 5단
        xtgt1_log_hbox = QHBoxLayout()
        xtgt2_log_hbox = QHBoxLayout()
        xtgt3_log_hbox = QHBoxLayout()

        self.xtgt1_log_lbl = QLabel('shot', self)
        self.xtgt2_log_lbl = QLabel('shot', self)
        self.xtgt3_log_lbl = QLabel('shot', self)

        self.xtgt1_count_log_lbl = QLabel('count', self)
        self.xtgt2_count_log_lbl = QLabel('count', self)
        self.xtgt3_count_log_lbl = QLabel('count', self)

        self.xtgt1_clear_btn = QPushButton('clear', self)
        self.xtgt2_clear_btn = QPushButton('clear', self)
        self.xtgt3_clear_btn = QPushButton('clear', self)

        self.xtgt1_print_log_btn = QPushButton('print', self)
        self.xtgt2_print_log_btn = QPushButton('print', self)
        self.xtgt3_print_log_btn = QPushButton('print', self)
        
        xtgt1_log_hbox.addWidget(self.xtgt1_log_lbl)
        xtgt1_log_hbox.addWidget(self.xtgt1_count_log_lbl)
        xtgt1_log_hbox.addWidget(self.xtgt1_clear_btn)
        xtgt1_log_hbox.addWidget(self.xtgt1_print_log_btn)

        xtgt2_log_hbox.addWidget(self.xtgt2_log_lbl)
        xtgt2_log_hbox.addWidget(self.xtgt2_count_log_lbl)
        xtgt2_log_hbox.addWidget(self.xtgt2_clear_btn)
        xtgt2_log_hbox.addWidget(self.xtgt2_print_log_btn)

        xtgt3_log_hbox.addWidget(self.xtgt3_log_lbl)
        xtgt3_log_hbox.addWidget(self.xtgt3_count_log_lbl)
        xtgt3_log_hbox.addWidget(self.xtgt3_clear_btn)
        xtgt3_log_hbox.addWidget(self.xtgt3_print_log_btn)

        #총 이동거리 -> 6단
        self.xtgt1_km_lbl = QLabel(f'{self.xtgt1_km} m', self)
        self.xtgt2_km_lbl = QLabel(f'{self.xtgt2_km} m', self)
        self.xtgt3_km_lbl = QLabel(f'{self.xtgt3_km} m', self)

        #라이다 끄고 키기 -> 7단
        xtgt1_lidar_hbox = QHBoxLayout()
        self.xtgt1_lidar_edit = QLineEdit('1', self)
        xtgt1_lidar_apply_btn = QPushButton('apply', self)
        xtgt1_lidar_hbox.addWidget(self.xtgt1_lidar_edit)
        xtgt1_lidar_hbox.addWidget(xtgt1_lidar_apply_btn)
        
        xtgt2_lidar_hbox = QHBoxLayout()
        self.xtgt2_lidar_edit = QLineEdit('1', self)
        xtgt2_lidar_apply_btn = QPushButton('apply', self)
        xtgt2_lidar_hbox.addWidget(self.xtgt2_lidar_edit)
        xtgt2_lidar_hbox.addWidget(xtgt2_lidar_apply_btn)
        
        xtgt3_lidar_hbox = QHBoxLayout()
        self.xtgt3_lidar_edit = QLineEdit('1', self)
        xtgt3_lidar_apply_btn = QPushButton('apply', self)
        xtgt3_lidar_hbox.addWidget(self.xtgt3_lidar_edit)
        xtgt3_lidar_hbox.addWidget(xtgt3_lidar_apply_btn)

        # a시리즈 gain -> 8단
        xtgt1_gain_hbox = QHBoxLayout()
        self.xtgt1_gain_edit = QLineEdit('65', self)
        xtgt1_gain_btn = QPushButton('gain', self)
        xtgt1_gain_hbox.addWidget(self.xtgt1_gain_edit)
        xtgt1_gain_hbox.addWidget(xtgt1_gain_btn)

        xtgt2_gain_hbox = QHBoxLayout()
        self.xtgt2_gain_edit = QLineEdit('65', self)
        xtgt2_gain_btn = QPushButton('gain', self)
        xtgt2_gain_hbox.addWidget(self.xtgt2_gain_edit)
        xtgt2_gain_hbox.addWidget(xtgt2_gain_btn)

        xtgt3_gain_hbox = QHBoxLayout()
        self.xtgt3_gain_edit = QLineEdit('65', self)
        xtgt3_gain_btn = QPushButton('gain', self)
        xtgt3_gain_hbox.addWidget(self.xtgt3_gain_edit)
        xtgt3_gain_hbox.addWidget(xtgt3_gain_btn)

        # hit 값 적용
        xtgt1_hit_hbox = QHBoxLayout()
        self.xtgt1_cri_edit = QLineEdit('1', self)
        self.xtgt1_non_cri_edit = QLineEdit('1', self)
        xtgt1_hit_apply_btn = QPushButton('apply', self)
        xtgt1_hit_hbox.addWidget(self.xtgt1_cri_edit)
        xtgt1_hit_hbox.addWidget(self.xtgt1_non_cri_edit)
        xtgt1_hit_hbox.addWidget(xtgt1_hit_apply_btn)

        xtgt2_hit_hbox = QHBoxLayout()
        self.xtgt2_cri_edit = QLineEdit('1', self)
        self.xtgt2_non_cri_edit = QLineEdit('1', self)
        xtgt2_hit_apply_btn = QPushButton('apply', self)
        xtgt2_hit_hbox.addWidget(self.xtgt2_cri_edit)
        xtgt2_hit_hbox.addWidget(self.xtgt2_non_cri_edit)
        xtgt2_hit_hbox.addWidget(xtgt2_hit_apply_btn)

        xtgt3_hit_hbox = QHBoxLayout()
        self.xtgt3_cri_edit = QLineEdit('1', self)
        self.xtgt3_non_cri_edit = QLineEdit('1', self)
        xtgt3_hit_apply_btn = QPushButton('apply', self)
        xtgt3_hit_hbox.addWidget(self.xtgt3_cri_edit)
        xtgt3_hit_hbox.addWidget(self.xtgt3_non_cri_edit)
        xtgt3_hit_hbox.addWidget(xtgt3_hit_apply_btn)

        # 마네킹 원점 복귀 -> 10단
        xtgt1_mnq_rst_btn = QPushButton('mnq_rst', self)
        xtgt2_mnq_rst_btn = QPushButton('mnq_rst', self)
        xtgt3_mnq_rst_btn = QPushButton('mnq_rst', self)

        # P1 인터럽트 trace 출력 -> 11단
        self.xtgt1_p1_trace_lbl = QLabel('P1 TRACE\nstart:- end:-\nwidth:- calc:- valid:-\nseq:- F/R:-/-', self)
        self.xtgt2_p1_trace_lbl = QLabel('P1 TRACE\nstart:- end:-\nwidth:- calc:- valid:-\nseq:- F/R:-/-', self)
        self.xtgt3_p1_trace_lbl = QLabel('P1 TRACE\nstart:- end:-\nwidth:- calc:- valid:-\nseq:- F/R:-/-', self)

        self.xtgt1_p1_trace_lbl.setStyleSheet('font-size: 10pt;')
        self.xtgt2_p1_trace_lbl.setStyleSheet('font-size: 10pt;')
        self.xtgt3_p1_trace_lbl.setStyleSheet('font-size: 10pt;')

        #그리드 레이 아웃
        grid.addLayout(xtgt1_main_hbox, 0, 0)
        grid.addLayout(xtgt1_mnq_hbox, 1, 0)
        grid.addLayout(xtgt1_ban_hbox, 2, 0)
        grid.addWidget(xtgt1_rst_btn, 3, 0)
        grid.addLayout(xtgt1_log_hbox, 4, 0)
        grid.addWidget(self.xtgt1_km_lbl, 5, 0)
        grid.addLayout(xtgt1_lidar_hbox, 6, 0)
        grid.addLayout(xtgt1_gain_hbox, 7, 0)
        grid.addLayout(xtgt1_hit_hbox, 8, 0)
        grid.addWidget(xtgt1_mnq_rst_btn, 9, 0)
        grid.addWidget(self.xtgt1_p1_trace_lbl, 10, 0)

        grid.addLayout(xtgt2_main_hbox, 0, 1)
        grid.addLayout(xtgt2_mnq_hbox, 1, 1)
        grid.addLayout(xtgt2_ban_hbox, 2, 1)
        grid.addWidget(xtgt2_rst_btn, 3, 1)
        grid.addLayout(xtgt2_log_hbox, 4, 1)
        grid.addWidget(self.xtgt2_km_lbl, 5, 1)
        grid.addLayout(xtgt2_lidar_hbox, 6, 1)
        grid.addLayout(xtgt2_gain_hbox, 7, 1)
        grid.addLayout(xtgt2_hit_hbox, 8, 1)
        grid.addWidget(xtgt2_mnq_rst_btn, 9, 1)
        grid.addWidget(self.xtgt2_p1_trace_lbl, 10, 1)

        grid.addLayout(xtgt3_main_hbox, 0, 2)
        grid.addLayout(xtgt3_mnq_hbox, 1, 2)
        grid.addLayout(xtgt3_ban_hbox, 2, 2)
        grid.addWidget(xtgt3_rst_btn, 3, 2)
        grid.addLayout(xtgt3_log_hbox, 4, 2)
        grid.addWidget(self.xtgt3_km_lbl, 5, 2)
        grid.addLayout(xtgt3_lidar_hbox, 6, 2)
        grid.addLayout(xtgt3_gain_hbox, 7, 2)
        grid.addLayout(xtgt3_hit_hbox, 8, 2)
        grid.addWidget(xtgt3_mnq_rst_btn, 9, 2)
        grid.addWidget(self.xtgt3_p1_trace_lbl, 10, 2)

        #기능
        self.xtgt1_auto_btn.clicked.connect(self.xtgt1_auto_func)
        self.xtgt1_up_btn.clicked.connect(self.xtgt1_up_func)
        self.xtgt1_down_btn.clicked.connect(self.xtgt1_down_func)
        self.xtgt1_hidden_btn.clicked.connect(self.xtgt1_hidden_func)

        self.xtgt2_auto_btn.clicked.connect(self.xtgt2_auto_func)
        self.xtgt2_up_btn.clicked.connect(self.xtgt2_up_func)
        self.xtgt2_down_btn.clicked.connect(self.xtgt2_down_func)
        self.xtgt2_hidden_btn.clicked.connect(self.xtgt2_hidden_func)

        self.xtgt3_auto_btn.clicked.connect(self.xtgt3_auto_func)
        self.xtgt3_up_btn.clicked.connect(self.xtgt3_up_func)
        self.xtgt3_down_btn.clicked.connect(self.xtgt3_down_func)
        self.xtgt3_hidden_btn.clicked.connect(self.xtgt3_hidden_func)

        self.xtgt1_ban1_btn.clicked.connect(self.xtgt1_ban1_func)
        self.xtgt1_ban2_btn.clicked.connect(self.xtgt1_ban2_func)
        self.xtgt1_ban3_btn.clicked.connect(self.xtgt1_ban3_func)

        self.xtgt2_ban1_btn.clicked.connect(self.xtgt2_ban1_func)
        self.xtgt2_ban2_btn.clicked.connect(self.xtgt2_ban2_func)
        self.xtgt2_ban3_btn.clicked.connect(self.xtgt2_ban3_func)

        self.xtgt3_ban1_btn.clicked.connect(self.xtgt3_ban1_func)
        self.xtgt3_ban2_btn.clicked.connect(self.xtgt3_ban2_func)
        self.xtgt3_ban3_btn.clicked.connect(self.xtgt3_ban3_func)

        xtgt1_rst_btn.clicked.connect(self.xtgt1_rst_func)
        xtgt2_rst_btn.clicked.connect(self.xtgt2_rst_func)
        xtgt3_rst_btn.clicked.connect(self.xtgt3_rst_func)

        self.xtgt1_clear_btn.clicked.connect(self.xtgt1_clear_func)
        self.xtgt2_clear_btn.clicked.connect(self.xtgt2_clear_func)
        self.xtgt3_clear_btn.clicked.connect(self.xtgt3_clear_func)

        self.xtgt1_print_log_btn.clicked.connect(self.xtgt1_print_log_func)
        self.xtgt2_print_log_btn.clicked.connect(self.xtgt2_print_log_func)
        self.xtgt3_print_log_btn.clicked.connect(self.xtgt3_print_log_func)

        xtgt1_lidar_apply_btn.clicked.connect(self.xtgt1_lidar_min_distance_apply)
        xtgt2_lidar_apply_btn.clicked.connect(self.xtgt2_lidar_min_distance_apply)
        xtgt3_lidar_apply_btn.clicked.connect(self.xtgt3_lidar_min_distance_apply)        

        xtgt1_gain_btn.clicked.connect(self.xtgt1_gain_pub_func)
        xtgt2_gain_btn.clicked.connect(self.xtgt2_gain_pub_func)
        xtgt3_gain_btn.clicked.connect(self.xtgt3_gain_pub_func)

        xtgt1_hit_apply_btn.clicked.connect(self.xtgt1_hit_scenario_func)
        xtgt2_hit_apply_btn.clicked.connect(self.xtgt2_hit_scenario_func)
        xtgt3_hit_apply_btn.clicked.connect(self.xtgt3_hit_scenario_func)

        xtgt1_mnq_rst_btn.clicked.connect(self.xtgt1_mnq_rst_pub_func)
        xtgt2_mnq_rst_btn.clicked.connect(self.xtgt2_mnq_rst_pub_func)
        xtgt3_mnq_rst_btn.clicked.connect(self.xtgt3_mnq_rst_pub_func)

        self.show()

    #함수
    def xtgt1_auto_func(self):
        self.xtgt1_mnq = 0
        msg = Int32()
        msg.data = self.xtgt1_mnq
        self.xtgt1_mnq_pub.publish(msg)
        self.xtgt1_lbl.setText('AUTO')
    def xtgt1_up_func(self):
        self.xtgt1_mnq = 1
        msg = Int32()
        msg.data = self.xtgt1_mnq
        self.xtgt1_mnq_pub.publish(msg)
        self.xtgt1_lbl.setText('UP')
    def xtgt1_down_func(self):
        self.xtgt1_mnq = 2
        msg = Int32()
        msg.data = self.xtgt1_mnq
        self.xtgt1_mnq_pub.publish(msg)
        self.xtgt1_lbl.setText('DOWN')
    def xtgt1_hidden_func(self):
        self.xtgt1_mnq = 3
        msg = Int32()
        msg.data = self.xtgt1_mnq
        self.xtgt1_mnq_pub.publish(msg)
        self.xtgt1_lbl.setText("HID")


    def xtgt2_auto_func(self):
        self.xtgt2_mnq = 0
        msg = Int32()
        msg.data = self.xtgt2_mnq
        self.xtgt2_mnq_pub.publish(msg)
        self.xtgt2_lbl.setText('AUTO')
    def xtgt2_up_func(self):
        self.xtgt2_mnq = 1
        msg = Int32()
        msg.data = self.xtgt2_mnq
        self.xtgt2_mnq_pub.publish(msg)
        self.xtgt2_lbl.setText('UP')
    def xtgt2_down_func(self):
        self.xtgt2_mnq = 2
        msg = Int32()
        msg.data = self.xtgt2_mnq
        self.xtgt2_mnq_pub.publish(msg)
        self.xtgt2_lbl.setText('DOWN')
    def xtgt2_hidden_func(self):
        self.xtgt2_mnq = 3
        msg = Int32()
        msg.data = self.xtgt2_mnq
        self.xtgt2_mnq_pub.publish(msg)
        self.xtgt2_lbl.setText("HID")

    def xtgt3_auto_func(self):
        self.xtgt3_mnq = 0
        msg = Int32()
        msg.data = self.xtgt3_mnq
        self.xtgt3_mnq_pub.publish(msg)
        self.xtgt3_lbl.setText('AUTO')
    def xtgt3_up_func(self):
        self.xtgt3_mnq = 1
        msg = Int32()
        msg.data = self.xtgt3_mnq
        self.xtgt3_mnq_pub.publish(msg)
        self.xtgt3_lbl.setText('UP')
    def xtgt3_down_func(self):
        self.xtgt3_mnq = 2
        msg = Int32()
        msg.data = self.xtgt3_mnq
        self.xtgt3_mnq_pub.publish(msg)
        self.xtgt3_lbl.setText('DOWN')
    def xtgt3_hidden_func(self):
        self.xtgt3_mnq = 3
        msg = Int32()
        msg.data = self.xtgt3_mnq
        self.xtgt3_mnq_pub.publish(msg)
        self.xtgt3_lbl.setText("HID")

    def xtgt1_ban1_func(self):
        msg = Int32MultiArray()
        if self.xtgt1_channel_list[0] == 0:
            self.xtgt1_channel_list[0] = 1
            self.xtgt1_ban1_btn.setText('OFF')
        else:
            self.xtgt1_channel_list[0] = 0
            self.xtgt1_ban1_btn.setText('ON')
        msg.data = self.xtgt1_channel_list
        self.xtgt1_ban_channel_pub.publish(msg)
    def xtgt1_ban2_func(self):
        msg = Int32MultiArray()
        if self.xtgt1_channel_list[1] == 0:
            self.xtgt1_channel_list[1] = 1
            self.xtgt1_ban2_btn.setText('OFF')
        else:
            self.xtgt1_channel_list[1] = 0
            self.xtgt1_ban2_btn.setText('ON')
        msg.data = self.xtgt1_channel_list
        self.xtgt1_ban_channel_pub.publish(msg)
    def xtgt1_ban3_func(self):
        msg = Int32MultiArray()
        if self.xtgt1_channel_list[2] == 0:
            self.xtgt1_channel_list[2] = 1
            self.xtgt1_ban3_btn.setText('OFF')
        else:
            self.xtgt1_channel_list[2] = 0
            self.xtgt1_ban3_btn.setText('ON')
        msg.data = self.xtgt1_channel_list
        self.xtgt1_ban_channel_pub.publish(msg)

    def xtgt2_ban1_func(self):
        msg = Int32MultiArray()
        if self.xtgt2_channel_list[0] == 0:
            self.xtgt2_channel_list[0] = 1
            self.xtgt2_ban1_btn.setText('OFF')
        else:
            self.xtgt2_channel_list[0] = 0
            self.xtgt2_ban1_btn.setText('ON')
        msg.data = self.xtgt2_channel_list
        self.xtgt2_ban_channel_pub.publish(msg)
    def xtgt2_ban2_func(self):
        msg = Int32MultiArray()
        if self.xtgt2_channel_list[1] == 0:
            self.xtgt2_channel_list[1] = 1
            self.xtgt2_ban2_btn.setText('OFF')
        else:
            self.xtgt2_channel_list[1] = 0
            self.xtgt2_ban2_btn.setText('ON')
        msg.data = self.xtgt2_channel_list
        self.xtgt2_ban_channel_pub.publish(msg)
    def xtgt2_ban3_func(self):
        msg = Int32MultiArray()
        if self.xtgt2_channel_list[2] == 0:
            self.xtgt2_channel_list[2] = 1
            self.xtgt2_ban3_btn.setText('OFF')
        else:
            self.xtgt2_channel_list[2] = 0
            self.xtgt2_ban3_btn.setText('ON')
        msg.data = self.xtgt2_channel_list
        self.xtgt2_ban_channel_pub.publish(msg)

    def xtgt3_ban1_func(self):
        msg = Int32MultiArray()
        if self.xtgt3_channel_list[0] == 0:
            self.xtgt3_channel_list[0] = 1
            self.xtgt3_ban1_btn.setText('OFF')
        else:
            self.xtgt3_channel_list[0] = 0
            self.xtgt3_ban1_btn.setText('ON')
        msg.data = self.xtgt3_channel_list
        self.xtgt3_ban_channel_pub.publish(msg)
    def xtgt3_ban2_func(self):
        msg = Int32MultiArray()
        if self.xtgt3_channel_list[1] == 0:
            self.xtgt3_channel_list[1] = 1
            self.xtgt3_ban2_btn.setText('OFF')
        else:
            self.xtgt3_channel_list[1] = 0
            self.xtgt3_ban2_btn.setText('ON')
        msg.data = self.xtgt3_channel_list
        self.xtgt3_ban_channel_pub.publish(msg)
    def xtgt3_ban3_func(self):
        msg = Int32MultiArray()
        if self.xtgt3_channel_list[2] == 0:
            self.xtgt3_channel_list[2] = 1
            self.xtgt3_ban3_btn.setText('OFF')
        else:
            self.xtgt3_channel_list[2] = 0
            self.xtgt3_ban3_btn.setText('ON')
        msg.data = self.xtgt3_channel_list
        self.xtgt3_ban_channel_pub.publish(msg)

    def xtgt1_rst_func(self):
        msg = Int32()
        msg.data = 1
        self.xtgt1_rst1_pub.publish(msg)

    def xtgt2_rst_func(self):
        msg = Int32()
        msg.data = 1
        self.xtgt2_rst1_pub.publish(msg)

    def xtgt3_rst_func(self):
        msg = Int32()
        msg.data = 1
        self.xtgt3_rst1_pub.publish(msg)

    def xtgt1_shot_signal_cb(self, msg: Int32):
        if msg.data!=0:
            log = f"{datetime.now():%H:%M:%S} -> {str(msg.data)}"
            self.xtgt1_log_lbl.setText(log)
            self.xtgt1_count += 1
            self.xtgt1_count_log_lbl.setText(str(self.xtgt1_count))
            self.xtgt1_log_dict[self.xtgt1_log_key] = log
            self.xtgt1_log_key += 1

    def xtgt2_shot_signal_cb(self, msg: Int32):
        if msg.data!=0:
            log = f"{datetime.now():%H:%M:%S} -> {str(msg.data)}"
            self.xtgt2_log_lbl.setText(log)
            self.xtgt2_count += 1
            self.xtgt2_count_log_lbl.setText(str(self.xtgt2_count))
            self.xtgt2_log_dict[self.xtgt2_log_key] = log
            self.xtgt2_log_key += 1

    def xtgt3_shot_signal_cb(self, msg: Int32):
        if msg.data!=0:
            log = f"{datetime.now():%H:%M:%S} -> {str(msg.data)}"
            self.xtgt3_log_lbl.setText(log)
            self.xtgt3_count += 1
            self.xtgt3_count_log_lbl.setText(str(self.xtgt3_count))
            self.xtgt3_log_dict[self.xtgt3_log_key] = log
            self.xtgt3_log_key += 1

    def xtgt1_clear_func(self):
        self.xtgt1_count = 0
        self.xtgt1_count_log_lbl.setText(str(self.xtgt1_count))   

    def xtgt2_clear_func(self):
        self.xtgt2_count = 0
        self.xtgt2_count_log_lbl.setText(str(self.xtgt2_count))

    def xtgt3_clear_func(self):
        self.xtgt3_count = 0
        self.xtgt3_count_log_lbl.setText(str(self.xtgt3_count))

    def xtgt1_print_log_func(self):
        print(self.xtgt1_log_dict)

    def xtgt2_print_log_func(self):
        print(self.xtgt2_log_dict)

    def xtgt3_print_log_func(self):
        print(self.xtgt3_log_dict)

    def xtgt1_odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.xtgt1_prev_xy is None:
            self.xtgt1_prev_xy = (x, y)
            return
        
        px, py = self.xtgt1_prev_xy
        d_m = math.hypot(x - px, y - py)
        self.xtgt1_km += d_m
        self.xtgt1_prev_xy = (x, y)

        self.xtgt1_km_lbl.setText(f"{self.xtgt1_km:.2f} m")
    
    def xtgt2_odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.xtgt2_prev_xy is None:
            self.xtgt2_prev_xy = (x, y)
            return
        
        px, py = self.xtgt2_prev_xy
        d_m = math.hypot(x - px, y - py)
        self.xtgt2_km += d_m
        self.xtgt2_prev_xy = (x, y)

        self.xtgt2_km_lbl.setText(f"{self.xtgt2_km:.2f} m")

    def xtgt3_odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.xtgt3_prev_xy is None:
            self.xtgt3_prev_xy = (x, y)

            return
        
        px, py = self.xtgt3_prev_xy
        d_m = math.hypot(x - px, y - py)
        self.xtgt3_km += d_m
        self.xtgt3_prev_xy = (x, y)

        self.xtgt3_km_lbl.setText(f"{self.xtgt3_km:.2f} m")
    
    def xtgt1_lidar_min_distance_apply(self):
        distance = float(self.xtgt1_lidar_edit.text())

        if not self.xtgt1_lidar_param_cli.service_is_ready():
            if not self.xtgt1_lidar_param_cli.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().warn(f"param service not ready: {self.xtgt1_lidar_target}/set_parameters_atomically")
                return
        
        pv = ParameterValue()
        pv.type = ParameterType.PARAMETER_DOUBLE
        pv.double_value = distance

        p = Parameter()
        p.name = "min_distance_threshold"
        p.value = pv

        req = SetParametersAtomically.Request()
        req.parameters = [p]

        future = self.xtgt1_lidar_param_cli.call_async(req)

        def _done_cb(fut):
            try:
                resp = fut.result()
                if resp.result.successful:
                    self.node.get_logger().info(f"xtgt1 min_distance_threshold -> {distance}")
                else:
                    self.node.get_logger().error(f"set failed : {resp.result.reason}")
            except Exception as e:
                self.node.get_logger().error(f"service call exception : {e}")

        future.add_done_callback(_done_cb)

    def xtgt2_lidar_min_distance_apply(self):
        distance = float(self.xtgt2_lidar_edit.text())

        if not self.xtgt2_lidar_param_cli.service_is_ready():
            if not self.xtgt2_lidar_param_cli.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().warn(f"param service not ready: {self.xtgt2_lidar_target}/set_parameters_atomically")
                return
        
        pv = ParameterValue()
        pv.type = ParameterType.PARAMETER_DOUBLE
        pv.double_value = distance

        p = Parameter()
        p.name = "min_distance_threshold"
        p.value = pv

        req = SetParametersAtomically.Request()
        req.parameters = [p]

        future = self.xtgt2_lidar_param_cli.call_async(req)

        def _done_cb(fut):
            try:
                resp = fut.result()
                if resp.result.successful:
                    self.node.get_logger().info(f"xtgt2 min_distance_threshold -> {distance}")
                else:
                    self.node.get_logger().error(f"set failed : {resp.result.reason}")
            except Exception as e:
                self.node.get_logger().error(f"service call exception : {e}")

        future.add_done_callback(_done_cb)

    def xtgt3_lidar_min_distance_apply(self):
        distance = float(self.xtgt3_lidar_edit.text())

        if not self.xtgt3_lidar_param_cli.service_is_ready():
            if not self.xtgt3_lidar_param_cli.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().warn(f"param service not ready: {self.xtgt3_lidar_target}/set_parameters_atomically")
                return
        
        pv = ParameterValue()
        pv.type = ParameterType.PARAMETER_DOUBLE
        pv.double_value = distance

        p = Parameter()
        p.name = "min_distance_threshold"
        p.value = pv

        req = SetParametersAtomically.Request()
        req.parameters = [p]

        future = self.xtgt3_lidar_param_cli.call_async(req)

        def _done_cb(fut):
            try:
                resp = fut.result()
                if resp.result.successful:
                    self.node.get_logger().info(f"xtgt3 min_distance_threshold -> {distance}")
                else:
                    self.node.get_logger().error(f"set failed : {resp.result.reason}")
            except Exception as e:
                self.node.get_logger().error(f"service call exception : {e}")

        future.add_done_callback(_done_cb)

    def xtgt1_gain_pub_func(self):
        gain = int(self.xtgt1_gain_edit.text())
        msg = Int32()
        msg.data = gain
        self.xtgt1_gain1_pub.publish(msg)

    def xtgt2_gain_pub_func(self):
        gain = int(self.xtgt2_gain_edit.text())
        msg = Int32()
        msg.data = gain
        self.xtgt2_gain1_pub.publish(msg)

    def xtgt3_gain_pub_func(self):
        gain = int(self.xtgt3_gain_edit.text())
        msg = Int32()
        msg.data = gain
        self.xtgt3_gain1_pub.publish(msg)

    def xtgt1_bat_cb(self, msg):
        bat = msg.data
        self.xtgt1_bat_lbl.setText(f"{bat}")

    def xtgt2_bat_cb(self, msg):
        bat = msg.data
        self.xtgt2_bat_lbl.setText(f"{bat}")

    def xtgt3_bat_cb(self, msg):
        bat = msg.data
        self.xtgt3_bat_lbl.setText(f"{bat}")

    def xtgt1_angle_cb(self, msg):
        ang = msg.data
        self.xtgt1_angle_lbl.setText(f"{ang:.2f}")
    def xtgt2_angle_cb(self, msg):
        ang = msg.data
        self.xtgt2_angle_lbl.setText(f"{ang:.2f}")
    def xtgt3_angle_cb(self, msg):
        ang = msg.data
        self.xtgt3_angle_lbl.setText(f"{ang:.2f}")            

    def _update_p1_trace_label(self, label, msg):
        # msg.data 순서:
        # [0]=p1_start, [1]=p1_end, [2]=p1_width, [3]=p1_seq, [4]=p1_fall, [5]=p1_rise
        if len(msg.data) < 6:
            return

        p1_start = int(msg.data[0]) & 0xFFFFFFFF
        p1_end = int(msg.data[1]) & 0xFFFFFFFF
        p1_width = int(msg.data[2]) & 0xFFFFFFFF
        p1_seq = int(msg.data[3]) & 0xFFFFFFFF
        p1_fall = int(msg.data[4]) & 0xFFFFFFFF
        p1_rise = int(msg.data[5]) & 0xFFFFFFFF

        calc_width = (p1_end - p1_start) & 0xFFFFFFFF
        valid = calc_width >= 150

        label.setText(
            f"P1 TRACE\n"
            f"start:{p1_start} end:{p1_end}\n"
            f"width:{p1_width} calc:{calc_width} valid:{valid}\n"
            f"seq:{p1_seq} F/R:{p1_fall}/{p1_rise}"
        )

    def xtgt1_p1_trace_cb(self, msg):
        self._update_p1_trace_label(self.xtgt1_p1_trace_lbl, msg)

    def xtgt2_p1_trace_cb(self, msg):
        self._update_p1_trace_label(self.xtgt2_p1_trace_lbl, msg)

    def xtgt3_p1_trace_cb(self, msg):
        self._update_p1_trace_label(self.xtgt3_p1_trace_lbl, msg)

    # hit_scenario 값 전송: [cri, non_cri]
    def _publish_hit_scenario(self, robot_name, cri_edit, non_cri_edit, publisher, attr_name):
        try:
            cri = int(cri_edit.text().strip())
            non_cri = int(non_cri_edit.text().strip())
        except ValueError:
            self.node.get_logger().error(f"{robot_name} hit_scenario 입력값은 정수만 가능합니다.")
            return

        if cri < 0 or non_cri < 0:
            self.node.get_logger().error(f"{robot_name} hit_scenario 입력값은 0 이상이어야 합니다.")
            return

        scenario = [cri, non_cri]
        setattr(self, attr_name, scenario)

        msg = Int32MultiArray()
        msg.data = scenario
        publisher.publish(msg)

        self.node.get_logger().info(
            f"{robot_name} hit_scenario -> cri:{cri}, non_cri:{non_cri}"
        )

    def xtgt1_hit_scenario_func(self):
        self._publish_hit_scenario(
            "xtgt1",
            self.xtgt1_cri_edit,
            self.xtgt1_non_cri_edit,
            self.xtgt1_hit_scenario_pub,
            "xtgt1_hit_scenario"
        )

    def xtgt2_hit_scenario_func(self):
        self._publish_hit_scenario(
            "xtgt2",
            self.xtgt2_cri_edit,
            self.xtgt2_non_cri_edit,
            self.xtgt2_hit_scenario_pub,
            "xtgt2_hit_scenario"
        )

    def xtgt3_hit_scenario_func(self):
        self._publish_hit_scenario(
            "xtgt3",
            self.xtgt3_cri_edit,
            self.xtgt3_non_cri_edit,
            self.xtgt3_hit_scenario_pub,
            "xtgt3_hit_scenario"
        )

    def xtgt1_mnq_rst_pub_func(self):
        msg = Int32()
        msg.data = 1
        self.xtgt1_mnq_rst_pub.publish(msg)

    def xtgt2_mnq_rst_pub_func(self):
        msg = Int32()
        msg.data = 1
        self.xtgt2_mnq_rst_pub.publish(msg)

    def xtgt3_mnq_rst_pub_func(self):
        msg = Int32()
        msg.data = 1
        self.xtgt3_mnq_rst_pub.publish(msg)


    def save_settings(self):
        filename = f"log/{datetime.now():%Y-%m-%d_%H-%M-%S}.json"
        data = {
            "xtgt1": self.xtgt1_log_dict,
            "xtgt2": self.xtgt2_log_dict,
            "xtgt3": self.xtgt3_log_dict,
        }

        with open(filename, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=4, ensure_ascii=False)

        print(f" 저장 완료 : {filename}")

    def closeEvent(self, event):
        try:
            self.save_settings()
            event.accept()
        except Exception as e:
            print(f" 저장 실패 : {e}")
            event.accept()    
    

if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node('xtgt_mnq_node')
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    app = QApplication(sys.argv)
    mnq_m_ui = MnqManualUi(node, executor)

    def _cleanup():
        mnq_m_ui.ros_timer.stop()
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()

    app.aboutToQuit.connect(_cleanup)
    sys.exit(app.exec_())    
