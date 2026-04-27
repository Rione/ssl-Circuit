#!/bin/bash

set -e

# バイナリの相対パス
CURRENT=$(cd $(dirname $0);pwd)
echo $CURRENT
DIR_NAME=`echo "$CURRENT" | sed -e 's/.*\/\([^\/]*\)$/\1/'`
echo $DIR_NAME

# Makefile の TARGET は firmware のため、通常はこのファイルが生成される。
binary_path="$CURRENT/build/firmware.bin"

# 既存運用との互換として、ディレクトリ名.bin もフォールバックする。
if [ ! -f "$binary_path" ]; then
	alt_binary_path="$CURRENT/build/$DIR_NAME.bin"
	if [ -f "$alt_binary_path" ]; then
		binary_path="$alt_binary_path"
	else
		echo "Error: Binary file not found. Tried:"
		echo "  $CURRENT/build/firmware.bin"
		echo "  $CURRENT/build/$DIR_NAME.bin"
		exit 1
	fi
fi

# STM32_Programmer_CLIを使用して書き込みを行う
# STLink Flashing
STM32_Programmer_CLI -c port=SWD -w "$binary_path" 0x08000000 -s