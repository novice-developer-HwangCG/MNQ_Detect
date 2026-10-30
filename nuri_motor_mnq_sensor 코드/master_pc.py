#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
master_pc.py

Master PC UI -> ROS2 topic -> SBC -> Pico 센서 진단 설정 전송용 간단 UI.

실행:
  source /opt/ros/humble/setup.bash
  python3 master_pc.py

SBC 쪽 sbc_data.py가 /pico_sensor_diag/cmd 를 구독하고 있어야 합니다.
"""

import json
import threading
import tkinter as tk
from tkinter import ttk, messagebox

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


DEFAULT_TOPIC = "/pico_sensor_diag/cmd"


class SensorCmdNode(Node):
    def __init__(self, topic_name: str = DEFAULT_TOPIC):
        super().__init__("master_pico_sensor_cmd_ui")
        self.pub = self.create_publisher(String, topic_name, 10)
        self.topic_name = topic_name

    def send_json(self, payload: dict):
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub.publish(msg)
        self.get_logger().info(f"PUB {self.topic_name}: {msg.data}")

    def send_raw(self, line: str):
        msg = String()
        msg.data = line.strip()
        self.pub.publish(msg)
        self.get_logger().info(f"PUB {self.topic_name}: {msg.data}")


class App:
    def __init__(self, root: tk.Tk, node: SensorCmdNode):
        self.root = root
        self.node = node
        self.root.title("Pico Sensor Diagnostic Config")
        self.root.geometry("520x430")
        self.root.resizable(False, False)

        self.status_var = tk.StringVar(value="대기 중")

        main = ttk.Frame(root, padding=14)
        main.pack(fill="both", expand=True)

        title = ttk.Label(main, text="Pico 센서 진단 설정 전송", font=("Arial", 15, "bold"))
        title.grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 12))

        self.p1_us = self._add_entry(main, 1, "P1 확인 시간 [us]", "1000")
        self.delay_us = self._add_entry(main, 2, "P1 LOW 이후 delay [us]", "1000")
        self.p2_us = self._add_entry(main, 3, "P2 확인 시간 [us]", "500")
        self.cooldown_us = self._add_entry(main, 4, "재감지 방지 시간 [us]", "70000")
        self.gain = self._add_entry(main, 5, "GAIN PWM [%]", "0")

        btn_frame = ttk.Frame(main)
        btn_frame.grid(row=6, column=0, columnspan=3, sticky="ew", pady=(16, 8))

        ttk.Button(btn_frame, text="CFG 전송", command=self.send_cfg).grid(row=0, column=0, padx=4, ipadx=12)
        ttk.Button(btn_frame, text="센서 ON", command=lambda: self.send_ena(1)).grid(row=0, column=1, padx=4, ipadx=12)
        ttk.Button(btn_frame, text="센서 OFF", command=lambda: self.send_ena(0)).grid(row=0, column=2, padx=4, ipadx=12)
        ttk.Button(btn_frame, text="STAT 요청", command=self.send_stat).grid(row=0, column=3, padx=4, ipadx=12)
        ttk.Button(btn_frame, text="HDR 요청", command=self.send_hdr).grid(row=0, column=4, padx=4, ipadx=12)

        raw_box = ttk.LabelFrame(main, text="Raw 명령 직접 전송")
        raw_box.grid(row=7, column=0, columnspan=3, sticky="ew", pady=(12, 8))
        self.raw_entry = ttk.Entry(raw_box, width=48)
        self.raw_entry.insert(0, "CFG,1000,1000,500,70000,0")
        self.raw_entry.grid(row=0, column=0, padx=8, pady=8, sticky="ew")
        ttk.Button(raw_box, text="Raw 전송", command=self.send_raw).grid(row=0, column=1, padx=8, pady=8)

        help_text = (
            "사용 순서\n"
            "1) SBC에서 python3 sbc_data.py 실행\n"
            "2) 이 UI에서 값 입력 후 CFG 전송\n"
            "3) 센서 ON 상태에서 테스트\n"
            "4) 결과 CSV/raw log는 SBC에 저장됨"
        )
        ttk.Label(main, text=help_text, justify="left").grid(row=8, column=0, columnspan=3, sticky="w", pady=(12, 6))

        ttk.Label(main, textvariable=self.status_var, foreground="blue").grid(row=9, column=0, columnspan=3, sticky="w", pady=(8, 0))

        for c in range(3):
            main.columnconfigure(c, weight=1)

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _add_entry(self, parent, row, label, default):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", pady=5)
        entry = ttk.Entry(parent, width=18)
        entry.insert(0, default)
        entry.grid(row=row, column=1, sticky="w", pady=5)
        return entry

    def _get_int(self, entry: ttk.Entry, name: str, min_v: int, max_v: int) -> int:
        try:
            value = int(entry.get().strip())
        except ValueError:
            raise ValueError(f"{name} 값이 정수가 아닙니다.")
        if value < min_v or value > max_v:
            raise ValueError(f"{name} 값은 {min_v}~{max_v} 범위여야 합니다.")
        return value

    def send_cfg(self):
        try:
            payload = {
                "cmd": "cfg",
                "p1_us": self._get_int(self.p1_us, "P1 확인 시간", 1, 5000000),
                "delay_us": self._get_int(self.delay_us, "P1 LOW 이후 delay", 0, 5000000),
                "p2_us": self._get_int(self.p2_us, "P2 확인 시간", 1, 5000000),
                "cooldown_us": self._get_int(self.cooldown_us, "재감지 방지 시간", 0, 10000000),
                "gain": self._get_int(self.gain, "GAIN PWM", 0, 100),
            }
            self.node.send_json(payload)
            self.status_var.set(f"CFG 전송 완료: {payload}")
        except Exception as e:
            messagebox.showerror("입력 오류", str(e))

    def send_ena(self, enable: int):
        payload = {"cmd": "ena", "enable": int(enable)}
        self.node.send_json(payload)
        self.status_var.set(f"센서 {'ON' if enable else 'OFF'} 전송 완료")

    def send_stat(self):
        self.node.send_json({"cmd": "stat"})
        self.status_var.set("STAT 요청 전송 완료")

    def send_hdr(self):
        self.node.send_json({"cmd": "hdr"})
        self.status_var.set("HDR 요청 전송 완료")

    def send_raw(self):
        line = self.raw_entry.get().strip()
        if not line:
            messagebox.showerror("입력 오류", "Raw 명령이 비어 있습니다.")
            return
        self.node.send_raw(line)
        self.status_var.set(f"Raw 전송 완료: {line}")

    def on_close(self):
        self.root.destroy()


def main():
    rclpy.init()
    node = SensorCmdNode(DEFAULT_TOPIC)

    # Publisher only UI라서 spin이 필수는 아니지만, ROS 내부 처리를 위해 백그라운드 spin.
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    root = tk.Tk()
    app = App(root, node)
    try:
        root.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
