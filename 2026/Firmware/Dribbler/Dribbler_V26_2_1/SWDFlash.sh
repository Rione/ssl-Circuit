#!/bin/bash

set -e

CURRENT="$(cd "$(dirname "$0")" && pwd)"
BINARY_PATH="$CURRENT/build/Dribbler_V26_2_1.bin"

if [ ! -f "$BINARY_PATH" ]; then
  echo "Error: Binary file not found: $BINARY_PATH" >&2
  exit 1
fi

PROGRAMMER="/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/Resources/bin/STM32_Programmer_CLI"

if [ ! -x "$PROGRAMMER" ]; then
  PROGRAMMER="$(command -v STM32_Programmer_CLI || true)"
fi

if [ -z "$PROGRAMMER" ]; then
  echo "Error: STM32_Programmer_CLI was not found." >&2
  exit 1
fi

"$PROGRAMMER" -c port=SWD -w "$BINARY_PATH" 0x08000000 -s
