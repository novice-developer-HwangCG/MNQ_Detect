#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sbc_data_monitor.py

SBC에서 실행하는 ROS2 -> UART(Pico) 중개 + Pico 연속 모니터링 출력/저장 코드.

실행:
  source /opt/ros/humble/setup.bash
  python3 sbc_data_monitor.py

기본값:
  ROS2 구독 topic: /pico_sensor_diag/cmd
  Pico UART: /dev/ttyMNQ, 115200bps
  로그 저장 폴더: ./pico_sensor_logs

필요 패키지:
  pip3 install pyserial

Pico가 보내는 줄:
  # ...        상태/헤더/명령 응답
  MON,...     주기 모니터링: 핀 상태, IRQ 카운터, 마지막 P1 폭 등
  EDG,...     P1/P2 raw edge 발생 로그
  PULSE,...   active pulse 폭 로그
  SD,...      알고리즘 판정 결과 또는 reject 이유
  HIT,...     간단 결과값
"""

import argparse
import csv
import datetime as dt
import json
import os
import queue
import signal
import sys
import threading
import time
from typing import Dict, Optional

import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


TOPIC_NAME = "/pico_sensor_diag/cmd"

SD_FIELDS = [
    "event_id",
    "type",
    "now_us",
    "p1_confirm_us",
    "delay_after_p1_end_us",
    "p2_confirm_us",
    "cooldown_us",
    "p1_active_start_us",
    "p1_active_end_us",
    "p1_active_width_us",
    "p1_valid",
    "reason",
    "p2_active_start_rel_us",
    "p2_active_end_rel_us",
    "p2_active_width_us",
    "p2_active_at_delay_start",
    "p2_active_after_delay_us",
    "p2_valid",
    "result_channel",
    "p1_rise_cnt",
    "p1_fall_cnt",
    "p2_rise_cnt",
    "p2_fall_cnt",
    "p1_pin_now",
    "p2_pin_now",
    "p1_active_now",
    "p2_active_now",
]

MON_FIELDS = [
    "now_us",
    "state",
    "enabled",
    "p1_pin",
    "p2_pin",
    "p1_active",
    "p2_active",
    "p1_rise_cnt",
    "p1_fall_cnt",
    "p2_rise_cnt",
    "p2_fall_cnt",
    "last_p1_active_start_us",
    "last_p1_active_end_us",
    "last_p1_active_width_us",
    "event_id",
    "edge_overflow_cnt",
    "pulse_overflow_cnt",
]

EDG_FIELDS = [
    "seq",
    "time_us",
    "gpio",
    "edge",
    "p1_pin",
    "p2_pin",
    "p1_active",
    "p2_active",
    "state",
]

PULSE_FIELDS = [
    "seq",
    "gpio",
    "start_us",
    "end_us",
    "width_us",
    "active_low",
    "state",
]

HIT_FIELDS = ["result_channel", "now_us"]


def with_common(fields):
    return ["pc_time_iso", "pc_time_unix"] + fields + ["raw_line", "parse_error"]


CSV_FIELDS = with_common(SD_FIELDS)
MON_CSV_FIELDS = with_common(MON_FIELDS)
EDG_CSV_FIELDS = with_common(EDG_FIELDS)
PULSE_CSV_FIELDS = with_common(PULSE_FIELDS)
HIT_CSV_FIELDS = with_common(HIT_FIELDS)


class PicoUartWorker:
    def __init__(self, port: str, baud: int, paths: Dict[str, str], print_all: bool = True):
        self.port = port
        self.baud = baud
        self.paths = paths
        self.print_all = print_all
        self.ser: Optional[serial.Serial] = None
        self.stop_event = threading.Event()
        self.tx_q: "queue.Queue[str]" = queue.Queue()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.lock = threading.Lock()
        self.files = {}
        self.writers = {}

    def start(self):
        for key, path in self.paths.items():
            if key == "raw":
                self.files[key] = open(path, "w", encoding="utf-8")
                continue

            fields = {
                "sd": CSV_FIELDS,
                "mon": MON_CSV_FIELDS,
                "edg": EDG_CSV_FIELDS,
                "pulse": PULSE_CSV_FIELDS,
                "hit": HIT_CSV_FIELDS,
            }[key]
            f = open(path, "w", newline="", encoding="utf-8")
            w = csv.DictWriter(f, fieldnames=fields)
            w.writeheader()
            self.files[key] = f
            self.writers[key] = w

        self.ser = serial.Serial(self.port, self.baud, timeout=0.02, write_timeout=0.2)
        time.sleep(0.2)
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        try:
            self.tx_q.put_nowait("")
        except Exception:
            pass
        if self.thread.is_alive():
            self.thread.join(timeout=1.5)
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        for f in self.files.values():
            try:
                f.flush()
                f.close()
            except Exception:
                pass

    def send_line(self, line: str):
        line = line.strip()
        if not line:
            return
        self.tx_q.put(line)

    def _run(self):
        rx_buf = bytearray()
        while not self.stop_event.is_set():
            try:
                while True:
                    line = self.tx_q.get_nowait().strip()
                    if line:
                        self._uart_write(line)
            except queue.Empty:
                pass

            try:
                data = self.ser.read(512) if self.ser else b""
                if data:
                    rx_buf.extend(data)
                    while b"\n" in rx_buf:
                        raw, _, rest = rx_buf.partition(b"\n")
                        rx_buf = bytearray(rest)
                        line = raw.decode("utf-8", errors="replace").strip()
                        if line:
                            self._handle_rx_line(line)
            except serial.SerialException as e:
                print(f"[UART ERROR] {e}", file=sys.stderr, flush=True)
                time.sleep(0.1)
            except Exception as e:
                print(f"[UART RX ERROR] {e}", file=sys.stderr, flush=True)
                time.sleep(0.05)

            time.sleep(0.001)

    def _uart_write(self, line: str):
        out = (line.strip() + "\n").encode("utf-8")
        with self.lock:
            if not self.ser or not self.ser.is_open:
                return
            self.ser.write(out)
            self.ser.flush()
        print(f"[SBC -> Pico] {line}", flush=True)
        raw = self.files.get("raw")
        if raw:
            raw.write(f"{dt.datetime.now().isoformat(timespec='milliseconds')},TX,{line}\n")
            raw.flush()

    def _handle_rx_line(self, line: str):
        now = time.time()
        iso = dt.datetime.fromtimestamp(now).isoformat(timespec="milliseconds")

        if self.print_all:
            print(f"[Pico -> SBC] {line}", flush=True)

        raw = self.files.get("raw")
        if raw:
            raw.write(f"{iso},RX,{line}\n")
            raw.flush()

        if line.startswith("SD,"):
            self._write_csv("sd", self._parse_line(line, "SD,", SD_FIELDS, CSV_FIELDS, iso, now))
        elif line.startswith("MON,"):
            self._write_csv("mon", self._parse_line(line, "MON,", MON_FIELDS, MON_CSV_FIELDS, iso, now))
        elif line.startswith("EDG,"):
            self._write_csv("edg", self._parse_line(line, "EDG,", EDG_FIELDS, EDG_CSV_FIELDS, iso, now))
        elif line.startswith("PULSE,"):
            self._write_csv("pulse", self._parse_line(line, "PULSE,", PULSE_FIELDS, PULSE_CSV_FIELDS, iso, now))
        elif line.startswith("HIT,"):
            self._write_csv("hit", self._parse_line(line, "HIT,", HIT_FIELDS, HIT_CSV_FIELDS, iso, now))

    def _parse_line(self, line: str, prefix: str, fields, all_fields, iso: str, now: float) -> Dict[str, str]:
        row = {k: "" for k in all_fields}
        row["pc_time_iso"] = iso
        row["pc_time_unix"] = f"{now:.6f}"
        row["raw_line"] = line
        row["parse_error"] = ""

        values = line[len(prefix):].split(",") if line.startswith(prefix) else []
        if len(values) != len(fields):
            row["parse_error"] = f"field_count={len(values)}, expected={len(fields)}"
        for name, value in zip(fields, values):
            row[name] = value
        return row

    def _write_csv(self, key: str, row: Dict[str, str]):
        w = self.writers.get(key)
        f = self.files.get(key)
        if w and f:
            w.writerow(row)
            f.flush()


class SbcBridgeNode(Node):
    def __init__(self, uart: PicoUartWorker):
        super().__init__("sbc_pico_sensor_diag_bridge")
        self.uart = uart
        self.sub = self.create_subscription(String, TOPIC_NAME, self.on_cmd, 10)
        self.get_logger().info(f"Subscribe: {TOPIC_NAME}")

    def on_cmd(self, msg: String):
        text = msg.data.strip()
        if not text:
            return
        try:
            line = self._convert_to_pico_line(text)
            if line:
                self.uart.send_line(line)
                self.get_logger().info(f"ROS cmd -> Pico: {line}")
        except Exception as e:
            self.get_logger().error(f"명령 처리 실패: {text} / {e}")

    def _convert_to_pico_line(self, text: str) -> str:
        upper = text.upper()
        if upper.startswith(("CFG,", "ENA,", "GAIN,", "MON,", "STAT", "HDR")):
            return text

        data = json.loads(text)
        cmd = str(data.get("cmd", "")).lower().strip()

        if cmd == "cfg":
            p1_us = int(data["p1_us"])
            delay_us = int(data["delay_us"])
            p2_us = int(data["p2_us"])
            cooldown_us = int(data.get("cooldown_us", 70000))
            gain = int(data.get("gain", 0))
            monitor_us = data.get("monitor_us", None)
            self._check_range("p1_us", p1_us, 1, 5000000)
            self._check_range("delay_us", delay_us, 0, 5000000)
            self._check_range("p2_us", p2_us, 1, 5000000)
            self._check_range("cooldown_us", cooldown_us, 0, 10000000)
            self._check_range("gain", gain, 0, 254)
            if monitor_us is None:
                return f"CFG,{p1_us},{delay_us},{p2_us},{cooldown_us},{gain}"
            monitor_us = int(monitor_us)
            self._check_range("monitor_us", monitor_us, 0, 10000000)
            return f"CFG,{p1_us},{delay_us},{p2_us},{cooldown_us},{gain},{monitor_us}"

        if cmd == "ena":
            enable = int(data.get("enable", 1))
            return f"ENA,{1 if enable else 0}"

        if cmd == "gain":
            gain = int(data["gain"])
            self._check_range("gain", gain, 0, 254)
            return f"GAIN,{gain}"

        if cmd == "mon":
            monitor_us = int(data.get("monitor_us", 100000))
            self._check_range("monitor_us", monitor_us, 0, 10000000)
            return f"MON,{monitor_us}"

        if cmd == "stat":
            return "STAT"

        if cmd == "hdr":
            return "HDR"

        raise ValueError(f"지원하지 않는 cmd: {cmd}")

    @staticmethod
    def _check_range(name: str, value: int, min_v: int, max_v: int):
        if value < min_v or value > max_v:
            raise ValueError(f"{name}={value}, allowed {min_v}~{max_v}")


def make_log_paths(log_dir: str):
    os.makedirs(log_dir, exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    return {
        "sd": os.path.join(log_dir, f"sensor_diag_{stamp}.csv"),
        "mon": os.path.join(log_dir, f"sensor_monitor_{stamp}.csv"),
        "edg": os.path.join(log_dir, f"sensor_edge_{stamp}.csv"),
        "pulse": os.path.join(log_dir, f"sensor_pulse_{stamp}.csv"),
        "hit": os.path.join(log_dir, f"sensor_hit_{stamp}.csv"),
        "raw": os.path.join(log_dir, f"sensor_raw_{stamp}.log"),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyMNQ", help="Pico UART port")
    ap.add_argument("--baud", type=int, default=115200, help="Pico UART baudrate")
    ap.add_argument("--log-dir", default="./pico_sensor_logs", help="CSV/raw log 저장 폴더")
    ap.add_argument("--no-print", action="store_true", help="터미널에 Pico 수신 줄을 출력하지 않고 저장만 함")
    args = ap.parse_args()

    paths = make_log_paths(args.log_dir)
    for label, path in paths.items():
        print(f"[LOG {label.upper()}] {path}", flush=True)

    uart = PicoUartWorker(args.port, args.baud, paths, print_all=(not args.no_print))
    uart.start()

    rclpy.init()
    node = SbcBridgeNode(uart)
    stop_requested = False

    def _sigint_handler(signum, frame):
        nonlocal stop_requested
        stop_requested = True
        print("\n[EXIT] Ctrl+C received", flush=True)

    signal.signal(signal.SIGINT, _sigint_handler)
    signal.signal(signal.SIGTERM, _sigint_handler)

    try:
        uart.send_line("HDR")
        uart.send_line("STAT")
        # 모니터링 기본 ON. 100ms 주기. 너무 빠르면 MON,200000 등으로 변경.
        uart.send_line("MON,100000")

        while rclpy.ok() and not stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print("\n[EXIT] KeyboardInterrupt", flush=True)
    finally:
        print("[EXIT] stopping node/uart...", flush=True)
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
        uart.stop()
        print("[DONE]", flush=True)


if __name__ == "__main__":
    main()
