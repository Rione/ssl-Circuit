# MainBoard_V26_2 UI 対応 実装プロンプト

以下を MainBoard_V26_2 ディレクトリで起動した Claude Code にそのまま渡す。

---

## 目的

`/Users/banbatakumi/GitHub/ssl-Circuit/2026/Firmware/MainBoard/MainBoard_V26_2`（STM32F446）のファームウェアを、新しい UI 基板（XIAO RP2040 + 2.8" ILI9341 タッチ液晶、`../../XIAO_RP2040`）に対応させる。UI 基板は UART4（250000bps, 8N1）でつながっており、次の用途に使う。

- MainBoard から UI への表示用テレメトリ: 電源電圧、昇圧電圧、ボール検知/保持、ホイール実角速度×4、IMU（加速度 XY・ヨー角速度・ヨー角）、充電状態など
- UI から MainBoard へのテスト指令: ドリブラー ON/OFF、充電/放電、ストレート/チップキック、ホイール単体回転

UI 側の実装は完成済みで、ビルドも通っている。**通信仕様は UI 側の `../../XIAO_RP2040/include/ui_protocol.h` を正とする。** MainBoard 側の都合でこの仕様を変えないこと。変える必要があると判断した場合は、実装せずに理由を報告すること。

## 最初に読むもの

- `docs/HANDOVER.md`（全体構成と規約）
- `../../XIAO_RP2040/include/ui_protocol.h`（フレーム形式、ペイロードのオフセット、単位、フラグ、CRC8、受信パーサ）
- `../../XIAO_RP2040/src/robot_link.cpp` と `../../XIAO_RP2040/src/ui.cpp`（UI がいつ、どのフィールドを送るか、どのフラグをどう表示するか）
- MainBoard 側の関連コード: `src/unit/ui.[ch]`、`src/unit/robot.[ch]`、`src/mode/main_mode.c`、`src/control/local_controller.c`、`src/unit/kicker.c`、`src/unit/dribbler.c`、`src/unit/omni_drive.c`、`src/unit/imu.h`、`src/app/app.c`、`src/config/parammeter.h`
- `../../PowerBoard/PowerBoard_V26_1/src/app/app.c`（CAN 0x50 の中身。`data[1]` は電源電圧×5、`data[2]` は昇圧電圧[V]。0x11 充電と 0x12 放電はラッチ動作）

## プロトコル要約（詳細はヘッダ）

- フレーム形式: `[0xFF][TYPE][LEN][PAYLOAD][CRC8][0xAA]`。CRC8 は TYPE・LEN・PAYLOAD にかける（多項式 0x07、初期値 0）。多バイト値はリトルエンディアン。
- `TYPE 0x01` テレメトリ（MainBoard → UI）: LEN=24、**20ms 周期**で送る。
- `TYPE 0x02` コマンド（UI → MainBoard）: LEN=8、UI が 20ms 周期で送ってくる。
- `UI_PROTO_LINK_TIMEOUT_MS`（200ms）の間コマンドが届かなければ、UI テストは中止する。

## 実装内容

### 1. プロトコルヘッダを取り込む

`../../XIAO_RP2040/include/ui_protocol.h` を `src/config/ui_protocol.h` へ**一字一句そのまま**コピーする。ヘッダのコメントにある同期ルールもそのまま残す。Makefile の include パスに `src/config` が入っているかを確認し、入っていなければ追加する。

### 2. `src/unit/ui.[ch]` の作り直し

旧仕様（`UiStatus`、4byte 固定長フレーム）は削除して置き換える。

- `UI` 構造体に次を持たせる。
  - `UiProtoParser`
  - 最新のコマンド（デコード済みの構造体）
  - 最終受信 tick
  - テレメトリ送信タイマー
  - **static な TX バッファ**
- `UI_Recv`: `Serial_Available` の間は**全バイトを**パーサへ投入する。現状は 1 周期（1ms）に 1 byte しか読んでいない。有効な `TYPE 0x02` を受け取ったら、コマンドを更新して受信 tick を記録する。
- `UI_SendTelemetry`: 20ms ごとに `UiProto_Build` でフレームを組み立てて送る。
  - `Serial_Write` は `HAL_UART_Transmit_DMA` なので、**送信バッファはスタック変数にしてはいけない**。現行の `UI_Send` はローカル配列を DMA に渡しているバグがある。
  - 前回の DMA 送信が完了していなければ（`huart->gState != HAL_UART_STATE_READY`）、その回はスキップしてバッファを上書きしない。
- `UI_IsLinkAlive(self)`: 最終受信から `UI_PROTO_LINK_TIMEOUT_MS` 以内かを返す。

テレメトリ各フィールドの出どころ:

| フィールド | 元データ | 変換 |
|---|---|---|
| BATTERY [0.01V] | ADC による電源電圧 | 後述の「3. 電源電圧」を参照 |
| CAP [V] | `info.kicker_status.cap_val` | そのまま |
| F1 BALL_DETECTED / BALL_HOLD | `info.dribble_status.is_detected_ball / is_hold_ball` | |
| F1 CHARGE_DONE | `info.kicker_status.done_charge` | |
| F1 ROCK_LINK | Rock5A から信号を受信中（後述の「4. Rock5A 受信のタイムアウト」を反映した値） | |
| F1 EMERGENCY | `info.status.emergency_stop`（ROCK_LINK が 0 のときは 0 にする） | |
| F1 TEST_ACTIVE | 後述の UI テストが有効か | |
| F1 CHARGING | 現在 MainBoard が出している指令が充電なら 1、放電なら 0 | |
| F1 IMU_READY | `imu.is_ready` | |
| F2 WHEEL_EMG / WHEEL_READY | `omni_drive.emg / ready` | |
| WHEEL[4] [0.01rad/s] | `omni_drive.vel_wheel_angular[i]` | ×100 して int16 |
| ACCEL_X/Y [mg] | `imu.accel_x/y` [g] | ×1000 |
| YAW_RATE [0.1deg/s] | `imu.yaw_rate` [rad/s] | ×(180/π)×10 |
| YAW [0.01deg] | `imu.yaw_rad` [rad] | ×(180/π)×100 |
| DRIBBLE [%] | 実際にドリブラーへ出している指令 | 0〜100 に換算 |
| KICK_ACK | 最後に**実際に CAN 送信した** UI キックの `kick_seq` | |

int16 へ変換するときは、オーバーフローしないよう飽和させる。

### 3. 電源電圧

`RobotInfo.battery_voltage` が `uint8_t` のため、整数 [V] に切り捨てられている。表示には 0.01V 単位が必要。

- このフィールドを `float` に変更する。
- SPI 送信の `dst[1] = info->battery_voltage * 10` は、明示的に `(uint8_t)` へ飽和キャストする。これで Rock5A 側の値は整数 V×10 から実測の 0.1V 分解能になり、互換性を保ったまま精度が上がる。この点は報告に書くこと。
- `app.c` の CAN 0x50 ハンドラでも `battery_voltage` を上書きしているが、直後に ADC 値で上書きされて意味がなくなっている。どちらを正とするかは変更せず、報告だけすること。

### 4. Rock5A 受信のタイムアウト（既存の安全上の問題）

`robot.c` の `rock_last_recv_tick` は書き込まれるだけで、どこからも参照されていない。そのため Rock5A が止まったり外れたりしても `info.status.is_signal_received` は最後の値（1）のまま残り、最後の速度・キック指令を実行し続ける。この状態では UI テストも永久に有効にならない。

`Robot_RockUpdateSPI` の中で、最終受信から `ROCK_SIGNAL_TIMEOUT_MS`（`parammeter.h` に追加、500ms）を超えたら次の処理をする。

- `status.is_signal_received = 0`
- 速度指令を 0 にする

既存の SPI 再同期、ストール検出、ダブルバッファの仕組みは壊さないこと。

### 5. UI テストモード

`src/mode/main_mode.c` の分岐を次の 3 段にする。UI テストの処理は `src/control/ui_test_controller.[ch]` を新規に作って置く（`LocalController` と同じ `構造体 + 関数(self, ...)` の形式）。

```
if (!emergency_stop && is_signal_received)   → 既存の通常動作（Rock5A を優先）
else if (UiTest 有効)                         → UiTestController_Run
else                                          → LocalController_Stop（既存）
```

**UiTest が有効になる条件**: 次のすべてを満たすとき。

- `UI_IsLinkAlive`
- コマンドの `UI_CMD_F_TEST_ENABLE` が 1
- `!is_signal_received`

**有効な間の動作**:

- **ドリブラー**
  - `DRIBBLE_ON` なら `Robot_SendDribble(r, pct * 255 / 100, 0)`、OFF なら 0 を送る。
  - `Dribbler_Send` は現状 0 か最大かの 2 値制御だが、その仕様は変更しない。
- **充放電**
  - `UI_CMD_F_CHARGE` が 1 なら `Kicker_Charge`、0 なら `Kicker_Discharge` を呼ぶ（既存の 100ms 間隔制限をそのまま使う）。
  - このときは `info.status.do_charge` を使わない。
- **キック**
  - `kick_seq` が前回処理した値から変わったときだけ、1 回だけキックする。
  - 種類は `kick_type`（STRAIGHT=1 / CHIP=2）、強さは `kick_pct * 255 / 100` を `Kicker_Kick` に渡す。
  - **UiTest が無効から有効に変わった最初のフレームでは、seq を記録するだけでキックしない。** リンク復帰時やリセット直後に誤って発射しないため。
  - `Kicker_Kick` は `ROBOT_KICK_INTERVAL_MS` の間隔内だと黙って捨てる。これを「送信したか」を返す形に変える（既存の呼び出し元は戻り値を無視してよい）。実際に送信できたときだけ `KICK_ACK` を更新する。捨てられた seq は再送しない。
  - `info.kicker.straight/chip` や `do_direct_*` は使わない。UI テストの最中は `Robot_SendKicker` を呼ばない。
- **ホイール**
  - `WHEEL_RUN` が 1 かつ `wheel_mask` が 0 以外のとき、マスクの立っている motor だけに目標角速度を与え、それ以外は 0 にする。
  - 目標角速度は `UI_TEST_WHEEL_MAX_RADPS`（`parammeter.h` に追加、50.0f）で飽和させてから ×100 する。
  - 送信は `OmniDrive_Send(m, 1)` で直接行う。単体テストなので `MAF` は通さない。
  - `WHEEL_RUN` が 0 なら `OmniDrive_SetFree`。
  - UI 側は押している間だけ `WHEEL_RUN=1` を送るデッドマン方式になっている。

**有効から無効に変わったとき**（タイムアウト、ロック、Rock5A の復帰を含む）は、1 回だけ終了処理をする。

- ドリブラーを 0 にする（`force_send=1`）。
- ホイールを Free にする。
- `Kicker_Discharge` を呼ぶ。PowerBoard の充電はラッチ式なので、放電指令を送らないと充電されたまま残る。ただし 100ms の間隔制限で送信がスキップされることがあるので、**確実に 1 回は送信されるまで**放電要求を保持すること。
- その後は既存の分岐に戻す。

### 6. 旧 UI 処理の撤去

`Robot_UpdateFromUi` の旧ロジックと `RobotInfo.ui_status` を削除する。旧ロジックは、`is_locked` が 0 のとき Rock5A の指令を UI の値で上書きしていた。`MainMode_Loop` のシリアル通信の位置では、`UI_Recv` と `UI_SendTelemetry` を呼ぶ形にする。

### 7. ドキュメント

- `docs/HANDOVER.md` の 5.4 節を、新プロトコルの概要と正のヘッダの場所に書き換える。
- 8 節の「UI基板があれば〜」を、実際の UiTest の条件（Rock5A 未接続、UI でアーム、200ms タイムアウト）に合わせて更新する。
- 4 節のループ説明も更新する。

## 制約

- `Core/`、`Drivers/` は USER CODE マーカーの外を編集しない。`.ioc` も変更しない（UART4 の 250000bps はそのまま使う）。
- `../../CommonLib-C` は変更しない（他の基板と共有しているため）。
- Rock5A との SPI プロトコル（`docs/SPI_PROTOCOL.md`）のバイト配置は変更しない。
- 定数は `src/config/parammeter.h` に置く。命名と書式は既存コードに合わせる。
- コミットはしない。

## 検証

1. `make` が警告を増やさずに通ること。変更前後の警告数を比べて報告する。
2. `src/config/ui_protocol.h` と `../../XIAO_RP2040/include/ui_protocol.h` が一致すること（`diff` の結果が空）。
3. ホスト上の gcc で簡単なテストを書き、実行結果を報告する。テストはリポジトリに残さず、scratch に置く。
   - UI が作るコマンドフレームを MainBoard 側のパーサで復元できること
   - テレメトリの変換（負値、飽和、角度の単位）が正しいこと
4. 変更点、実機で確認すべき項目、判断に迷った点を最後に一覧で報告する。実機で確認すべき項目は次のとおり。
   - Rock5A 未接続で UI からアームするとテストできる
   - UI の電源を抜くと 200ms 以内に停止・放電する
   - Rock5A を接続すると UI テストが無効になる
   - キックはボタン 1 回につき 1 回だけ出る
