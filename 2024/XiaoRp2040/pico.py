import shutil
import os
import platform

# ファームウェアファイルのパス
source_file = ".pio/build/seeed_xiao_rp2040/firmware.uf2"

# OSに応じた外部ドライブのマウントポイントを設定
os_type = platform.system()
if os_type == "Windows":
    destination_path = "E:\\"
elif os_type == "Darwin":  # macOS
    destination_path = "/Volumes/RPI-RP2/"
else:  # Linux
    destination_path = os.path.join("/media", os.getenv("USER"), "RPI-RP2")

# コピー先のファイルパス
destination_file = os.path.join(destination_path, "firmware.uf2")

# コピー処理
try:
    if not os.path.exists(destination_path):
        raise FileNotFoundError(f"{destination_path} が見つかりません。外部ドライブがマウントされていることを確認してください。")
    shutil.copy(source_file, destination_file)
    print(f"ファームウェアが {destination_file} にコピーされました。")
except FileNotFoundError as e:
    print(e)
except PermissionError:
    print("アクセス権限が不足しています。")
except Exception as e:
    print(f"エラーが発生しました: {e}")
