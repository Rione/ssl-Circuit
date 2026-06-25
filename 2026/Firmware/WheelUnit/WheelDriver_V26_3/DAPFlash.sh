#!/bin/bash

# バイナリの相対パス
CURRENT=$(cd $(dirname $0);pwd)
echo $CURRENT
DIR_NAME=`echo "$CURRENT" | sed -e 's/.*\/\([^\/]*\)$/\1/'`
echo $DIR_NAME
binary_path="$CURRENT/build/$DIR_NAME.bin"

# バイナリが存在するか確認
if [ ! -f "$binary_path" ]; then
    echo "Error: Binary file not found at $binary_path"
    exit 1
fi

# OpenOCDがインストールされているか確認
if ! command -v openocd &> /dev/null
then
    echo "Error: openocd could not be found. Please install it (e.g., 'brew install openocd')."
    exit 1
fi

# DAPLinkを使って書き込みを行う
# OpenOCD + CMSIS-DAP (DAPLink) Flashing
echo "Flashing with DAPLink (OpenOCD)..."
openocd -f interface/cmsis-dap.cfg -f target/stm32g4x.cfg -c "program $binary_path 0x08000000 verify reset exit"