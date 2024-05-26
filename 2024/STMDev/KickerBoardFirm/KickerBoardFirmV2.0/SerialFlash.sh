#!/bin/bash

# Macの場合
if [[ $(uname) == "Darwin" ]]; then
    # /devディレクトリ内のシリアルポートを取得
    serial_ports=$(ls /dev/cu.usb*  2>/dev/null)
else
    # Ubuntuの場合
    serial_ports=$(ls /dev/ttyS* /dev/ttyUSB* /dev/ttyACM* 2>/dev/null)
fi

# シリアルデバイスのリストを表示
echo "Available serial ports:"
index=1
for port in $serial_ports; do
    echo "$index: $port"
    ((index++))
done

# ユーザーにシリアルポートの選択を促す
read -p "Enter the number of the serial port you want to use: " selected_index

# 選択されたシリアルポートを取得
selected_port=$(echo "$serial_ports" | sed -n "${selected_index}p")

# バイナリの相対パス
CURRENT=$(cd $(dirname $0);pwd)
echo $CURRENT
DIR_NAME=`echo "$CURRENT" | sed -e 's/.*\/\([^\/]*\)$/\1/'`
echo $DIR_NAME
binary_path="$CURRENT/build/$DIR_NAME.elf"

# STM32_Programmer_CLIを使用して書き込みを行う

# Serial Flashing
STM32_Programmer_CLI -c port="$selected_port" br=115200 -w "$binary_path" 0x08000000 -s
