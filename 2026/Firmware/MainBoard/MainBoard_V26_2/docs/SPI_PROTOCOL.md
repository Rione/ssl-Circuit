# Rock5A ⇔ MainBoard SPI通信プロトコル

対象: MainBoard_V26_2 (STM32F446xx) / SPI2

実装: [`src/unit/robot.c`](../src/unit/robot.c) の `Robot_RockBuildTxPacket`(送信) / `Robot_RockApplyRecvPacket`(受信)

OmniDrive の電圧制御の値 (輪ごとの印加電圧・速度 PI の積分・FF を縮めた比率・状態の bit・滑りの判定) を毎周期 Rock5A へ送るため、ペイロードを19→38byte、フレームを21→40byteに拡張した(2026-10)。**21byte の Wing / ファームとはつながらないので、MainBoard のファームと Wing は 40byte のもの同士にすること。**

---

## 1. 物理層 / フレーム構成

| 項目 | 値 |
|---|---|
| ペリフェラル | SPI2 |
| 役割 | MainBoardがSlave、Rock5AがMaster |
| モード | Mode0 (CPOL=0, CPHA=0) |
| ビット順 | MSBファースト |
| データサイズ | 8bit |
| NSS | ハードウェア入力 (Rock5Aがアサート) |
| 多バイト値のバイト順 | リトルエンディアン (下位byteが先) |

マスター(Rock5A)が周期的にクロックを出す必要があり、MainBoard側から能動的に送信を開始することはできない(全二重で、送受信は同一トランザクション内で同時に行われる)。

### フレームフォーマット

```
[ヘッダ 0xFF][ペイロード 38byte][フッタ 0xAA]  = 40byte / トランザクション
```

- 送信(MainBoard→Rock5A)・受信(Rock5A→MainBoard)とも同じ40byteフレーム長で同期させる。
- MainBoardはソフトウェア的にヘッダ/フッタを検査し、直近2フレーム分(80byte)のスライディングウィンドウから正しいフレーム位置を探して再同期する(`Robot_RockFindFrame`)。ビットずれで通信がハングした場合は750ms進捗が無いことを検出して強制的に再Armする(詳細は [`HANDOVER.md` 5.1](HANDOVER.md#51-rock5a--mainboard-spi2-slave) 参照)。

---

## 2. 送信ペイロード (MainBoard → Rock5A) — 38byte

フレーム全体でのバイト位置(0=ヘッダ)を基準に記載。

| フレーム位置 | サイズ | フィールド | 型 | スケール | 単位 | 内容 |
|---:|---:|---|---|---|---|---|
| 0 | 1 | (ヘッダ) | - | - | - | 固定値 `0xFF` |
| 1 | 1 | `battery_voltage` | uint8 | ×10 | V | バッテリー電圧。実値 = raw / 10 |
| 2 | 1 | `dribble_status` | uint8 (bitfield) | - | - | `is_detected_ball`(bit0) / `is_hold_ball`(bit1) / `is_new_drib`(bit2) |
| 3 | 1 | `kicker_cap_val` | uint8 | - | - | キッカー用コンデンサ電圧(PowerBoardからのCAN受信値をそのまま転送) |
| 4–5 | 2 | `wheel0_angular_vel` | int16 LE | ×100 | rad/s | ホイール0(取付角55°)の実角速度 |
| 6–7 | 2 | `wheel1_angular_vel` | int16 LE | ×100 | rad/s | ホイール1(取付角135°) |
| 8–9 | 2 | `wheel2_angular_vel` | int16 LE | ×100 | rad/s | ホイール2(取付角-135°) |
| 10–11 | 2 | `wheel3_angular_vel` | int16 LE | ×100 | rad/s | ホイール3(取付角-55°) |
| 12–13 | 2 | `accel_x` | int16 LE | ×1000 | g | IMU加速度X (機体前後方向、**新規**) |
| 14–15 | 2 | `accel_y` | int16 LE | ×1000 | g | IMU加速度Y (機体左右方向、**新規**) |
| 16–17 | 2 | `yaw_rate` | int16 LE | ×900 | rad/s | IMUジャイロZ(旋回方向の角速度、**新規**) |
| 18–19 | 2 | `yaw_rad` | int16 LE | ×10000 | rad | Madgwickフィルタによる姿勢角yaw(**新規**、詳細は3節) |
| 20–21 | 2 | `cmd_voltage[0]` | int16 LE | ×1000 | V | ホイール0に送った印加電圧(`OmniDrive.cmd_voltage`、速度モード中は0) |
| 22–23 | 2 | `cmd_voltage[1]` | int16 LE | ×1000 | V | ホイール1 |
| 24–25 | 2 | `cmd_voltage[2]` | int16 LE | ×1000 | V | ホイール2 |
| 26–27 | 2 | `cmd_voltage[3]` | int16 LE | ×1000 | V | ホイール3 |
| 28–29 | 2 | `vel_fb_integral[0]` | int16 LE | ×1000 | V | 機体速度PIの積分 x(並進、`OmniDrive.vel_fb_integral`) |
| 30–31 | 2 | `vel_fb_integral[1]` | int16 LE | ×1000 | V | 同 y(並進) |
| 32–33 | 2 | `vel_fb_integral[2]` | int16 LE | ×1000 | V | 同 w(回転) |
| 34 | 1 | `ff_scale` | uint8 | ×255 | - | 上限でFFの分に掛けた比率(`OmniDrive.log_ff_scale`)。255=1(縮めていない)、速度モード中も255 |
| 35 | 1 | `drive_status` | uint8 (bitfield) | - | - | `volt_saturated`(bit0) / `volt_pi_saturated`(bit1) / `vel_fb_rot_stalled`(bit2) / `volt_traction_limited`(bit3)。前周期の状態 |
| 36 | 1 | `slip_status` | uint8 (bitfield) | - | - | bit0〜2: `tcs.slip_flags`(bit0:幾何拘束残差 / bit1:旋回残差 / bit2:加速度残差) / bit7: `tcs.is_slipping` |
| 37–38 | 2 | (予約) | - | - | - | `0x00` |
| 39 | 1 | (フッタ) | - | - | - | 固定値 `0xAA` |

実値への変換はすべて `raw / スケール`。例: `accel_x` の raw値が `1234` なら `1234 / 1000 = 1.234g`。`yaw_rad` の raw値が `15708` なら `15708 / 10000 ≈ 1.5708rad`(≈90°)。

### 変更点 (旧プロトコルとの差分)

- ペイロードが18byte→19byte、フレーム全体が20byte→21byteに拡張。
- フレーム位置12〜18(旧仕様では未使用の0埋め領域7byte)を全て使い切り、IMUの4フィールド(8byte)を追加。
- 2026-10: ペイロードが19byte→38byte、フレーム全体が21byte→40byteに拡張。フレーム位置20〜36にOmniDriveの電圧制御の値(17byte)を追加し、37〜38は予約。250kHzで1トランザクション約1.3ms。

---

## 3. IMUフィールドの補足

センサー: LSM6DSO32XTR (SPI1接続、加速度計+ジャイロの6軸IMU、`CommonLib-C/lsm6dso32/lsm6dso32.h`)
設定: ODR 104Hz、加速度レンジ±4G、ジャイロレンジ±2000dps (`src/unit/imu.c`)

- ジャイロレンジは`ROBOT_MAX_ANG_VEL`(10rad/s≈573dps、`parammeter.h`)に対して十分な余裕を持たせて±2000dpsとしている。レンジを超えるとセンサーが飽和し、高速回転時にyawの追従が実際の回転に追いつかなくなる点に注意(±500dpsだと通常の最大角速度指令だけで飽和してしまう)。
- **`accel_x` / `accel_y`**: 加速度計の生値をそのまま(積分・フィルタ処理なし)、機体座標系のX/Y軸成分。
- **`yaw_rate`**: ジャイロZ軸の角速度生値[rad/s](センサー生値[dps]から変換)。旋回方向(yaw)の角速度で、姿勢角ではない点に注意。
- **`yaw_rad`**: Madgwickフィルタ(`CommonLib-C/ahrs/madgwick.h`、加速度計+ジャイロのみのIMU版、磁気センサ不使用)で推定した姿勢クォータニオンからZYXオイラー角のyaw成分を抽出した値[rad]。
  - ロール・ピッチは加速度計の重力方向で常に補正されドリフトしないが、**磁気センサが無いためyawには絶対方位の基準が無く、起動時(電源投入時)の姿勢を0radとした相対角(範囲: -π~π)**。長時間運用ではジャイロのバイアス起因でドリフトする。
  - Rock5A側でカメラ等の絶対方位情報と組み合わせて補正することを想定している。

---

## 4. 受信ペイロード (Rock5A → MainBoard) — 38byte

参考として、送信側と対になる受信フォーマットも記載する(今回のIMU対応による変更はなし)。使用実装は `Robot_RockApplyRecvPacket`。

| フレーム位置 | サイズ | フィールド | 型 | スケール | 単位 | 内容 |
|---:|---:|---|---|---|---|---|
| 0 | 1 | (ヘッダ) | - | - | - | 固定値 `0xFF` |
| 1–2 | 2 | `vel_x` | int16 LE | ×1000 | m/s | 並進速度指令X |
| 3–4 | 2 | `vel_y` | int16 LE | ×1000 | m/s | 並進速度指令Y |
| 5–6 | 2 | `vel_angular` | int16 LE | ×1000 | rad/s | 角速度指令 |
| 7 | 1 | `dribble_power` | uint8 | - | - | ドリブラーパワー(0で停止、非0で最大出力) |
| 8 | 1 | `kicker_straight` | uint8 | ×2.55 | - | ストレートキック出力(0-100 → 0-255に変換して使用) |
| 9 | 1 | `kicker_chip` | uint8 | ×2.55 | - | チップキック出力(同上) |
| 10–11 | 2 | `relative_position_x` | int16 LE | - | - | (用途はRock5A側アプリ依存) |
| 12–13 | 2 | `relative_position_y` | int16 LE | - | - | (同上) |
| 14–15 | 2 | `relative_theta` | int16 LE | - | - | (同上) |
| 16 | 1 | `camera.x` | uint8 | - | - | (同上) |
| 17 | 1 | `camera.y` | uint8 | - | - | (同上) |
| 18 | 1 | `status` | uint8 (bitfield) | - | - | bit0:`emergency_stop` / bit1:`do_direct_straight` / bit2:`do_direct_chip` / bit3:予約 / bit4:`do_charge` / bit5:`is_signal_received` / bit6:`is_ctrl_by_robot` / bit7:`parity` |

受信側はフレーム位置1〜18(payload先頭18byte)のみを使用し、フレーム位置19〜38は未使用(予約、`0x00`を送信すること)。送受信フレーム長は40byteで揃える必要がある点に注意。

---

## 5. 変更履歴

| 日付 | 内容 |
|---|---|
| 2026-09 | IMU(加速度xy・角速度yaw・Madgwickフィルタ姿勢yaw)を送信ペイロードに追加。ペイロード18byte→19byte、フレーム20byte→21byteに拡張。 |
| 2026-09 | `yaw_rate`/`yaw_rad`の単位をdps/degからrad/s/radに変更(コード全体でradに統一する方針のため)。スケールは `yaw_rate` ×10→×1000、`yaw_rad`(旧`yaw_deg`) ×100→×10000に変更。バイト位置・サイズに変更なし。 |
| 2026-10 | OmniDriveの電圧制御の値(印加電圧×4・速度PIの積分×3・FFの比率・状態のbit・滑りの判定)を送信ペイロードに追加。ペイロード19byte→38byte、フレーム21byte→40byteに拡張(受信側は予約が増えただけ)。 |
| 2026-09 | ジャイロレンジを±500dps→±2000dpsに変更(`ROBOT_MAX_ANG_VEL`超過による飽和で高速回転時にyawが追従しなくなる不具合の修正)。それに伴い`yaw_rate`のスケールを×1000→×900に変更(int16オーバーフロー回避)。 |
