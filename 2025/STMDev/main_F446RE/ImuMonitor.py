#!/usr/bin/env python3
# DAPLink(SWD)経由で、動作中のSTM32のRAMから robot.info.imuState.euler を読み出して
# ターミナルに常時表示する。CPUは止めないので、ファームはそのまま動き続ける。
#
# 使い方: python3 ImuMonitor.py   (Enterでyawの基準を今の値にリセット、Ctrl+Cで終了)
# 注意: build/main_F446RE.elf から変数のアドレスを引くので、書き込んだファームと同じビルドで実行すること

import math
import os
import select
import shutil
import signal
import socket
import struct
import subprocess
import sys
import time

CURRENT = os.path.dirname(os.path.abspath(__file__))
ELF = os.path.join(CURRENT, "build", "main_F446RE.elf")
OPENOCD_DIR = os.path.expanduser("~/.platformio/packages/tool-openocd")
TCL_PORT = 6666
INTERVAL_S = 0.1


def find_euler_address():
    out = subprocess.run(
        ["arm-none-eabi-gdb", "-batch", "-ex", "p/x &robot.info.imuState.euler", ELF],
        capture_output=True, text=True, check=True).stdout
    # 例: "$1 = 0x20000a18"
    return int(out.strip().split("=")[-1].split()[0], 16)


def find_fusion_mode():
    with open(os.path.join(CURRENT, "src", "app", "config.h")) as f:
        for line in f:
            if line.startswith("#define IMU_FUSION "):
                return line.split()[2]
    return "?"


class OpenOcd:
    def __init__(self):
        self.proc = subprocess.Popen(
            [os.path.join(OPENOCD_DIR, "bin", "openocd"), "-s", os.path.join(OPENOCD_DIR, "openocd", "scripts"),
             "-f", "interface/cmsis-dap.cfg", "-f", "target/stm32f4x.cfg",
             "-c", f"tcl_port {TCL_PORT}", "-c", "gdb_port disabled", "-c", "telnet_port disabled", "-c", "init"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        for _ in range(50):
            try:
                self.sock = socket.create_connection(("127.0.0.1", TCL_PORT), timeout=2)
                return
            except OSError:
                if self.proc.poll() is not None:
                    raise RuntimeError("openocdが起動できませんでした (DAPLinkの接続を確認)")
                time.sleep(0.1)
        raise RuntimeError("openocdに接続できませんでした")

    def cmd(self, c):
        self.sock.sendall(c.encode() + b"\x1a")
        buf = b""
        while not buf.endswith(b"\x1a"):
            buf += self.sock.recv(4096)
        return buf[:-1].decode()

    def read_floats(self, addr, n):
        words = [int(w, 0) for w in self.cmd(f"read_memory {addr:#x} 32 {n}").split()]
        return [struct.unpack("<f", struct.pack("<I", w))[0] for w in words]

    def close(self):
        try:
            self.sock.close()
        finally:
            self.proc.terminate()
            self.proc.wait()


def wrap180(deg):
    return (deg + 180.0) % 360.0 - 180.0


def main():
    addr = find_euler_address()
    print(f"euler @ {addr:#x}   IMU_FUSION(config.h) = {find_fusion_mode()}")
    print("Enter: yawの基準を今の値にリセット / Ctrl+C: 終了")
    # ターミナルを閉じた/killされた場合もopenocdを残さないよう、Ctrl+Cと同じ終了処理に流す
    def on_signal(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGHUP, on_signal)

    ocd = OpenOcd()
    try:
        watch_stdin = True
        yaw0 = None
        t0 = time.monotonic()
        while True:
            yaw, pitch, roll = (math.degrees(v) for v in ocd.read_floats(addr, 3))  # euler_t {yaw, pitch, roll}
            now = time.monotonic()
            if watch_stdin and select.select([sys.stdin], [], [], 0)[0]:
                if sys.stdin.readline():
                    yaw0 = None
                else:
                    watch_stdin = False  # EOF(端末以外から起動)ならリセット入力は受け付けない
            if yaw0 is None:
                yaw0, t0 = yaw, now
            elapsed = now - t0
            dyaw = wrap180(yaw - yaw0)
            rate = dyaw / elapsed * 60.0 if elapsed > 5.0 else 0.0
            line = (f"{elapsed:7.1f}s  yaw {yaw:+8.2f}°  Δyaw {dyaw:+8.3f}°  ({rate:+7.3f}°/min)"
                    f"  roll {roll:+6.2f}°  pitch {pitch:+6.2f}°")
            # 端末幅を超えると折り返して\rで上書きできなくなるので、幅に収める
            width = shutil.get_terminal_size().columns - 1
            sys.stdout.write("\r" + line[:width].ljust(width))
            sys.stdout.flush()
            time.sleep(INTERVAL_S)
    except KeyboardInterrupt:
        print()
    finally:
        ocd.close()


if __name__ == "__main__":
    main()
