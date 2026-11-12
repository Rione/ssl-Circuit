# Raspberry Pi 4B ⇔ 旧 MainBoard UART 通信プロトコル

対象: 旧機体の MainBoard (STM32F446、このファーム) と、Raspberry Pi 4B の上の ssl-RAVEN-Wing (`-board pi4`)。

- ファームの実装: [`src/unit/RasProtocol.cpp`](../src/unit/RasProtocol.cpp) (パケットの組み立てと取り出し)、[`src/unit/Robot.cpp`](../src/unit/Robot.cpp) の `Robot::rasSendSerial` (送信) と `Robot::rasRecvSerial` (受信)
- Wing の実装: `internal/mainboard/` の `crc16.go`・`cobs.go`・`uart_frame.go`・`uart_parser.go`・`uart_link.go`
- この文書の 1〜7 節は、ssl-RAVEN-Wing の `UART_PROTOCOL.md` と同じ内容。変えるときは両方を直すこと。

2026-11 に、COBS・CRC・IMU の値を足した形式に変えた。古い形式 (FF … AA、チェックサムなし) とは通信できない (7 節)。

---

## 1. 物理層

| 項目 | 値 |
|---|---|
| ペリフェラル | UART5 (TX: PC12、RX: PD2) |
| Pi 側のデバイス | `/dev/serial0` (PL011 の `ttyAMA0`) |
| 速度 | 230400bps |
| 形式 | 8N1、フロー制御なし |
| 多バイト値のバイト順 | リトルエンディアン (下位 byte が先) |

- MainBoard は、Pi が送るかどうかに関係なく、**約 8ms ごと** (`rasSendSerial(info, 8)`、約 125Hz) にテレメトリーを送る。
- MainBoard は受信バッファを 1ms ごとに読む。Pi はいつ送ってもよい。
- Wing は、テレメトリーが届くたびに 1 周期を回し、すぐに指令を 1 パケット送り返す。

---

## 2. パケットの形

```
パケット = COBS( ペイロード ‖ CRC16 ) ‖ 0x00
```

| 項目 | 決まり |
|---|---|
| 区切り | `0x00`。COBS で符号化すると中に `0x00` が無くなるので、`0x00` が来たらそこがパケットの終わり |
| COBS | Consistent Overhead Byte Stuffing。254byte より短いパケットでは 1byte だけ増える |
| CRC | CRC-16/CCITT-FALSE (多項式 0x1021、初期値 0xFFFF、入力・出力の反転なし、最後の XOR なし)。`"123456789"` で `0x29B1` |
| CRC をかける範囲 | ペイロード全体 (type から最後まで)。リトルエンディアンでペイロードの後ろに付ける |
| ペイロードの先頭 | 0 バイト目が type、1 バイト目が seq |
| seq | パケットごとに 1 ずつ増える。255 の次は 0。向きごとに別々に数える |

- 受け取る側は、`0x00` までのバイトを COBS で復号し、**長さ・CRC・type がすべて合うときだけ**使う。合わないものは捨てて数える。
- 区切りが来ないまま長くなったら (ノイズ・ボーレートの誤り)、次の `0x00` まで読み捨てる。
- 形式を変えるときは、type に新しい番号を使う。

COBS の例: `11 22 00 33` → `03 11 22 02 33` (データの中の `00` は「次の `00` までの距離」に置き換わり、先頭にも同じ形の 1byte が付く)。

| type | 向き | ペイロード | パケット (CRC・COBS・区切りを足した長さ) |
|---|---|---:|---:|
| `0x01` | テレメトリー (MainBoard → Pi) | 22byte | 26byte (230400bps で約 1.1ms) |
| `0x02` | 指令 (Pi → MainBoard) | 20byte | 24byte |

---

## 3. テレメトリー (MainBoard → Pi) — type 0x01、22byte

| 位置 | 大きさ | フィールド | 型 | スケール | 単位 | 内容 |
|---:|---:|---|---|---|---|---|
| 0 | 1 | type | uint8 | - | - | `0x01` |
| 1 | 1 | seq | uint8 | - | - | パケットごとに 1 ずつ増える |
| 2 | 1 | `battery` | uint8 | ×10 | V | 電池の電圧 (0.1V 単位) |
| 3 | 1 | `status` | uint8 (bitfield) | - | - | bit0: ボールを検知 / bit1: ボールを保持 / bit2: 2025 年版ドリブラー基板 / **bit3: `imu_valid`** / bit4〜7: 0 |
| 4 | 1 | `cap_val_estimate` | uint8 | - | - | キッカーのコンデンサーの**推定値 (0〜100)**。昇圧した電圧 [V] ではない |
| 5–6 | 2 | wheel0 (M0、55°) | int16 | ×100 | rad/s | ホイールの角速度 |
| 7–8 | 2 | wheel1 (M1、135°) | int16 | ×100 | rad/s | 〃 |
| 9–10 | 2 | wheel2 (M2、−135°) | int16 | ×100 | rad/s | 〃 |
| 11–12 | 2 | wheel3 (M3、−55°) | int16 | ×100 | rad/s | 〃 |
| 13–14 | 2 | `accel_x` | int16 | ×1000 | g | 加速度 (BNO055 の x 軸のまま) |
| 15–16 | 2 | `accel_y` | int16 | ×1000 | g | 加速度 (BNO055 の y 軸のまま) |
| 17–18 | 2 | `yaw_rate` | int16 | ×900 | rad/s | ジャイロの z (バイアスを引いたもの)。**反時計回りが正** |
| 19–20 | 2 | `yaw` | int16 | ×10000 | rad | 姿勢角。起動したときを 0 とした相対角 (−π〜π)。**反時計回りが正** |
| 21 | 1 | `rx_bad` | uint8 | - | - | MainBoard が捨てた指令の数 (UART の受信エラー・CRC・形式)。255 の次は 0 |

- status の bit0〜2 は、ファームの `dribbleStatus` と同じ。bit3 は送るときに重ねる (`dribbleStatus` の共用体には入れない)。
- ホイールの並びは、モーターの番号の順 (M0〜M3) で、Wing の wheel0〜3 (SPI の新機体と同じ) と同じ。値はモーターの角速度 × 15/56 × 100 で、−32100〜32000 に制限してある (`MotorDriver::motorToWheelScaled`)。
- IMU の欄 (13〜20 バイト目) の単位とスケールは、新機体の SPI と同じ。
- **`imu_valid`** は、BNO055 が起動時に見つかっていて、直近 50ms (`RAS_IMU_STALE_MS`) 以内に新しい値が読めたときだけ 1。0 のとき IMU の欄は 0。
  - IMU を載せていない機体では常に 0。I2C が止まったときも 0 になる。
  - `IMU_FUSION_BNO_IMUPLUS` のときは、BNO055 の yaw が時計回りで正なので、反転して送る。
- int16 に収まらない値は、上限か下限の値にして送る (オーバーフローで符号が反転しないように)。

---

## 4. 指令 (Pi → MainBoard) — type 0x02、20byte

| 位置 | 大きさ | フィールド | 型 | スケール | 内容 |
|---:|---:|---|---|---|---|
| 0 | 1 | type | uint8 | - | `0x02` |
| 1 | 1 | seq | uint8 | - | パケットごとに 1 ずつ増える |
| 2–3 | 2 | `vel_x` | int16 | ×1000 | 並進速度の指令 x [m/s] |
| 4–5 | 2 | `vel_y` | int16 | ×1000 | 並進速度の指令 y [m/s] |
| 6–7 | 2 | `vel_angular` | int16 | ×1000 | 角速度の指令 [rad/s] |
| 8 | 1 | `dribble_power` | uint8 | - | ドリブラーの出力 |
| 9 | 1 | `kicker_straight` | uint8 | - | ストレートキックの出力 (0〜100) |
| 10 | 1 | `kicker_chip` | uint8 | - | チップキックの出力 (0〜100) |
| 11–12 | 2 | `relative_position_x` | int16 | - | (使わない) |
| 13–14 | 2 | `relative_position_y` | int16 | - | (使わない) |
| 15–16 | 2 | `relative_theta` | int16 | - | (使わない) |
| 17 | 1 | `camera.x` | uint8 | - | カメラのボールの x (0〜255) |
| 18 | 1 | `camera.y` | uint8 | - | カメラのボールの y (0〜255) |
| 19 | 1 | `status` | uint8 (bitfield) | - | bit0: `emergency_stop` / bit1: `do_direct_straight` / bit2: `do_direct_chip` / bit3: 予約 / bit4: `do_charge` / bit5: `is_signal_received` / bit6: `is_ctrl_by_robot` / bit7: `parity` |

2〜19 バイト目の 18byte は、古い形式の `0xFF` のあとの 18byte と同じ並びで、新機体の SPI のフレームの 1〜18 バイト目とも同じ。

旧 MainBoard のファームでの扱いは、次のとおり。

| フィールド | 扱い |
|---|---|
| `vel_x`・`vel_y` (×1000) | mm/s として使う (`setVelocityFF`) |
| `vel_angular` (×1000) | mrad/s として使う |
| `dribble_power` | 0 より大きければ、出力 100 にする |
| `kicker_straight`・`kicker_chip` | ×2.55 をせず、そのままキッカーの基板へ送る。500ms に 1 回までに制限する |
| `relative_*`・status の bit6 (`is_ctrl_by_robot`)・bit7 (`parity`)・bit3 | 読むが使わない |
| `camera.x`・`camera.y` | カメラモードのときだけ使う |

---

## 5. 安全のための決まり

**MainBoard (このファーム)**

- 新しい形式の指令だけを受け付ける。古い形式の指令 (RACOON-Pi2 など) は形・CRC が合わないので捨てる (機体は止まったまま)。
- パケットの途中で UART の受信エラーが起きたときは、CRC が合っていてもそのパケットを捨てる (`rasPacketErrorCount`)。
- CRC・長さ・type が合わないパケットと、区切りが来ないまま長くなったものは捨てる (`rasCrcErrorCount`)。
- **200ms** (`RAS_SIGNAL_TIMEOUT_MS`) のあいだ正しい指令が届かないと、速度・ドリブラー・キックを 0 にし、status を 0 にして止まる。
- 1 秒のあいだ 1byte も届かないと、受信バッファを捨てて、パケットの始まりを待つところからやり直す。

**Pi (Wing)**

- **直近 3 周期 (24ms) 以内に正しいテレメトリー (CRC が合ったもの) を受け取ったときだけ、指令を送る。**
  古い形式のファームは、新しい形式のバイト列の中の `0xFF` を指令の始まりと取り違え、続く 18byte を指令として使ってしまう。古いファームからは正しいテレメトリーが届かないので、この決まりで何も送らずに済む。
- テレメトリーが一度に何パケットも届いたときは、最新のものだけを使う。
- `imu_valid` が 0 のパケットでは、IMU の値を使わない (Alula にも RAVEN にも渡さない)。

---

## 6. 試験の例

ファームの `test/ras_protocol_test.cpp` と、Wing の `internal/mainboard/uart_test.go` で同じバイト列を確かめている。

**指令**: `vel_x` 0.5 m/s、`vel_y` −0.25 m/s、`vel_angular` 1.0 rad/s、ドリブラー 1、ストレートキック 50、status bit5 (`is_signal_received`)、seq 7。

```
ペイロード  02 07 F4 01 06 FF E8 03 01 32 00 00 00 00 00 00 00 00 00 20
CRC         0xD3EB (EB D3)
パケット    0B 02 07 F4 01 06 FF E8 03 01 32 01 01 01 01 01 01 01 01 04 20 EB D3 00
```

**テレメトリー**: 電池 148 (14.8V)、status 0b101 + `imu_valid`、キッカー 77、ホイール 1234・250・−3000・−100、加速度 123・−456、`yaw_rate` 1350、`yaw` −7854、seq 200、`rx_bad` 3。

```
ペイロード  01 C8 94 0D 4D D2 04 FA 00 48 F4 9C FF 7B 00 38 FE 46 05 52 E1 03
CRC         0x3C12 (12 3C)
パケット    09 01 C8 94 0D 4D D2 04 FA 06 48 F4 9C FF 7B 0A 38 FE 46 05 52 E1 03 12 3C 00
```

---

## 7. 古い形式 (2026-11 まで、対応しない)

```
MainBoard → Pi: [0xFF][電池][dribbleStatus][キッカー][ホイール M3, M1, M2, M0 (int16)][0xAA] = 13byte
Pi → MainBoard: [0xFF][4 節の 2〜19 バイト目と同じ 18byte] = 19byte (フッタ無し)
```

- チェックサムが無く、データの中にも `0xFF` や `0xAA` が出てくるので、区切りを取り違えることがあった。
- このファームとも、今の Wing とも通信できない。RACOON-Pi2 はこの形式なので、新しいファームの機体は動かせない。
- Wing は、古い形式のテレメトリーらしいバイト列 (13byte ごとに `0xFF` と `0xAA`) が続くと、ログと `uart_test` に「MainBoard のファームが古い形式」と出し、指令は送らない。
