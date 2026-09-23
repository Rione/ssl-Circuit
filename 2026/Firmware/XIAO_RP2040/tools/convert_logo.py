#!/usr/bin/env python3
"""assets/logo.png を 4bit グレースケールの C 配列 (include/logo_bitmap.h) に変換する。

使い方: python3 tools/convert_logo.py [高さpx(既定180)]
白=15, 黒=0。1byteに2画素(上位ニブルが左画素)を詰める。
"""
import sys
from pathlib import Path

from PIL import Image

ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / "assets" / "logo.png"
DST = ROOT / "include" / "logo_bitmap.h"

height = int(sys.argv[1]) if len(sys.argv) > 1 else 180

im = Image.open(SRC).convert("L")
bbox = im.point(lambda v: 255 if v > 16 else 0).getbbox()  # 余白の黒を除去
im = im.crop(bbox)
width = round(im.width * height / im.height)
width += width % 2  # 2画素/byteのため偶数幅にする
im = im.resize((width, height), Image.LANCZOS)

px = [v * 15 // 255 for v in im.getdata()]
data = bytes((px[i] << 4) | px[i + 1] for i in range(0, len(px), 2))

lines = []
for i in range(0, len(data), 16):
    lines.append("  " + ", ".join(f"0x{b:02X}" for b in data[i:i + 16]) + ",")

DST.write_text(
    "// 自動生成: tools/convert_logo.py (元画像 assets/logo.png)。直接編集しない。\n"
    "#pragma once\n\n"
    "#include <stdint.h>\n\n"
    f"#define LOGO_WIDTH {width}\n"
    f"#define LOGO_HEIGHT {height}\n\n"
    "// 4bit グレースケール、1byte に 2 画素 (上位ニブル = 左)\n"
    "static const uint8_t kLogoBitmap[LOGO_WIDTH * LOGO_HEIGHT / 2] = {\n"
    + "\n".join(lines)
    + "\n};\n"
)
print(f"{DST.relative_to(ROOT)}: {width}x{height}, {len(data)} bytes")
