#!/usr/bin/env python3
# DAPLink(SWD)経由で、動作中のSTM32から異常カウンタとCANのエラー状態を読み出して表示する。
# CPUは止めないので、試合形式の動作テスト中にそのまま確認できる。
#
# 使い方: python3 Diag.py          (1回表示)
#         python3 Diag.py --watch  (1秒ごとに更新、Ctrl+Cで終了)
# 注意: build/main_F446RE.elf から変数のアドレスを引くので、書き込んだファームと同じビルドで実行すること

import os
import signal
import struct
import subprocess
import sys
import time

from ImuMonitor import ELF, OpenOcd

# (表示名, gdbで評価する式, 型)
COUNTERS = [
    ("IMU(BNO055)の有無", "robot.imuAvailable", "bool"),
    ("2025年版ドリブラー基板", "robot.newDribblerDetected", "bool"),
    ("フォトセンサの値", "robot.info.photoSensorValue", "u16"),
    ("ウォッチドッグによるリセット", "persistResetCount", "u32"),
    ("Pi通信途絶で停止", "robot.rasSignalTimeoutCount", "u32"),
    ("Piパケット破棄(受信エラー)", "robot.rasPacketErrorCount", "u32"),
    ("Piパケット破棄(CRC・形式)", "robot.rasCrcErrorCount", "u32"),
    ("ループ周期オーバー(main)", "mainMode.loopOverrunCount", "u32"),
    ("ループ周期オーバー(camera)", "cameraMode.loopOverrunCount", "u32"),
    ("UART受信エラー PC(USART1)", "robot.serial1.rxErrorCount", "u32"),
    ("UART受信エラー UI(UART4)", "robot.serial4.rxErrorCount", "u32"),
    ("UART受信エラー Pi(UART5)", "robot.serial5.rxErrorCount", "u32"),
    ("UART送信スキップ UI(UART4)", "robot.serial4.txDropCount", "u32"),
    ("UART送信スキップ Pi(UART5)", "robot.serial5.txDropCount", "u32"),
    ("I2Cバス復旧(BNO055)", "robot.bno.busRecoverCount", "u32"),
    ("I2C読み出しエラー(BNO055)", "robot.bno.i2cErrorCount", "u32"),
    ("  復旧の内訳: 時間切れ", "robot.bno.recoverTimeoutCount", "u32"),
    ("  復旧の内訳: BUSYのまま", "robot.bno.recoverBusyCount", "u32"),
    ("    そのときSDAがLow", "robot.bno.busySdaLowCount", "u32"),
    ("    そのときSCLがLow", "robot.bno.busySclLowCount", "u32"),
    ("    そのとき線はHigh", "robot.bno.busyLinesHighCount", "u32"),
    ("    そのとき受信データあり", "robot.bno.busyRxDataCount", "u32"),
    ("    そのときまだマスター", "robot.bno.busyMslCount", "u32"),
    ("  復旧後もBUSYで再リセット", "robot.bno.recoverStillBusyCount", "u32"),
    ("  読み出し開始の失敗", "robot.bno.startFailCount", "u32"),
    ("  読み出し完了", "robot.bno.asyncOkCount", "u32"),
    ("  エラー: NACK", "robot.bno.errAfCount", "u32"),
    ("  エラー: バスエラー", "robot.bno.errBerrCount", "u32"),
    ("  エラー: アービトレーション", "robot.bno.errArloCount", "u32"),
    ("  エラー: オーバーラン", "robot.bno.errOvrCount", "u32"),
    ("IMU全て0で捨てた(CONFIGのまま)", "robot.imuZeroSampleCount", "u32"),
    ("IMU設定し直し", "robot.imuReconfigCount", "u32"),
    ("CAN送信できず破棄", "robot.can.txDropCount", "u32"),
    ("ジャイロバイアスX[dps]", "robot.bnoGyroBias.x", "f32"),
    ("ジャイロバイアスY[dps]", "robot.bnoGyroBias.y", "f32"),
    ("ジャイロバイアスZ[dps]", "robot.bnoGyroBias.z", "f32"),
]
CAN1_ESR = 0x40006418  # CAN1(0x40006400) + ESR(0x18)
CAN_LEC = ["なし", "スタッフ", "フォーム", "ACKなし", "劣性ビット", "優性ビット", "CRC", "(ソフト設定)"]


def find_addresses():
    args = ["arm-none-eabi-gdb", "-batch"]
    for _, expr, _ in COUNTERS:
        args += ["-ex", f"p/x &{expr}"]
    out = subprocess.run(args + [ELF], capture_output=True, text=True, check=True).stdout
    addrs = [int(line.split()[-1], 16) for line in out.splitlines() if line.startswith("$")]
    if len(addrs) != len(COUNTERS):
        raise RuntimeError("変数のアドレスを取得できませんでした (ビルドし直してください)")
    return addrs


def read_word(ocd, addr):
    return int(ocd.cmd(f"read_memory {addr:#x} 32 1").split()[0], 0)


def show(ocd, addrs):
    lines = []
    for (name, _, kind), addr in zip(COUNTERS, addrs):
        if kind in ("bool", "u16"):
            # 4byte境界にないことがあるので、境界から読んでずらす
            w = read_word(ocd, addr & ~3) >> ((addr & 3) * 8)
            w &= 0xFF if kind == "bool" else 0xFFFF
        else:
            w = read_word(ocd, addr)
        if kind == "f32":
            value = f"{struct.unpack('<f', struct.pack('<I', w))[0]:+.4f}"
        elif kind == "bool":
            value = "あり" if w & 0xFF else "なし"
        else:
            value = str(w)
        lines.append(f"  {name:<28} {value:>10}")
    esr = read_word(ocd, CAN1_ESR)
    state = "バスオフ" if esr & 0x4 else "エラーパッシブ" if esr & 0x2 else "エラー警告" if esr & 0x1 else "正常"
    lines.append(f"  CAN状態: {state}  送信エラー数={(esr >> 16) & 0xFF}  受信エラー数={(esr >> 24) & 0xFF}"
                 f"  直近のエラー={CAN_LEC[(esr >> 4) & 0x7]}")
    return "\n".join(lines)


def main():
    watch = "--watch" in sys.argv[1:]
    addrs = find_addresses()

    def on_signal(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGHUP, on_signal)

    ocd = OpenOcd()
    try:
        while True:
            text = show(ocd, addrs)
            if watch:
                sys.stdout.write("\033[H\033[J")  # 画面を消して先頭から描き直す
                print(time.strftime("%H:%M:%S"), "(Ctrl+Cで終了)")
            print(text)
            if not watch:
                break
            time.sleep(1.0)
    except KeyboardInterrupt:
        print()
    finally:
        ocd.close()


if __name__ == "__main__":
    main()
