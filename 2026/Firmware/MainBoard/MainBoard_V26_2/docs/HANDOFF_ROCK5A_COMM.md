# Rock5A ⇔ MainBoard 通信不良の調査 引き継ぎ

最終更新: 2026-09-24　／　ブランチ: `FW/MainBoard_V26_2_tc`（最新コミット `e30a4c5d`、push済み）
関連文書: [`HANDOVER.md`](HANDOVER.md) 5.1、[`SPI_PROTOCOL.md`](SPI_PROTOCOL.md)

## 1. 症状と経緯
Rock5Aからの指令でロボットの動作をテストしたところ、以下3つの症状が出た。
1. ロボットが動かない（今も未解決）。
2. Rock5A側PCで読めるバッテリー電圧が0.4Vと25.0Vを行き来する（**解決済み**）。
3. MainBoard上の圧電素子からブザー音が鳴り続ける（**解決済み**、詳細は`HANDOFF_VOLTAGE_CONTROL.md`は無関係、この文書と別件）。

`mainboard_V26.2_BugFix`というブランチ（ユーザー側で別途進めていた通信まわりの修正）を`FW/MainBoard_V26_2_tc`にマージし（コミット`c3d9a08a`）、症状2・3は解決した。

- 症状2の原因: `Robot_RockBuildTxPacket`で`dst[1] = battery_voltage * 10`としていたが、25.5Vを超えるとuint8（0-255）がオーバーフローしていた実バグ。`×5`に修正済み。
- 症状2のもう一つの原因: `HAL_SPI_TxRxCpltCallback`のRX受信バッファが単一で、ISRとメインループ間で競合状態（読み出し中に次の受信で上書き）があった。ダブルバッファ化して解消。
- 症状3の原因: ハートビートLED用PWM（`Robot_UpdateHeartBeat`、2秒周期でsin波状にdutyを変化）が`TIM1_CH1`（PA8）に出ていたが、このタイマーのキャリア周波数が約1〜2kHzと可聴域で、基板上の圧電素子がこれに反応して鳴っていた。`TIM2_CH2`に出力先を変更して解消。

症状1（動かない）だけ、まだ解決していない。

## 2. 症状1の調査状況

### 2.1 分かっていること
- Rock5A側の速度指令は機体座標系（ロボット自身から見た前後・左右）で送られている。MainBoardの逆運動学（`v_wheel = -vx·sinθ + vy·cosθ + Rω`、`src/unit/omni_drive.c`の`OmniDrive_SetVelEx`）はグローバル座標からの変換を一切せず、受け取った`vel_x/vel_y`をそのまま機体座標として使う実装なので、**ここは問題ない**（Rock5A側がカメラ座標などから機体座標へ変換してから送っている前提）。
- `src/mode/main_mode.c`の起動条件はこの1行だけ:
  ```c
  if (!r->info.status.emergency_stop && r->info.status.is_signal_received) { /* 動く */ }
  else { LocalController_Stop(...); }  // 動かない
  ```
- `emergency_stop`・`is_signal_received`は、**Rock5Aから受信したSPIパケットの状態byte（フレーム位置18、bit0=emergency_stop、bit5=is_signal_received、`docs/SPI_PROTOCOL.md`参照）からしか来ない。** MainBoard側に物理的な非常停止入力や他の代入経路は無い（コード上、`status.emergency_stop`への代入は`Robot_RockApplyRecvPacket`の1箇所のみと確認済み）。
- 今回のマージで、**有効なSPIフレームを300ms（`ROCK_SPI_SIGNAL_TIMEOUT_MS`）受信できないと、`is_signal_received`が強制的に0にクリアされる**仕組みが追加された（`Robot_RockUpdateSPI`末尾）。
- バッテリー電圧が直ったことから、**SPIのフレーム同期自体はおおむね機能している**とみられる（同期が大きく崩れていれば全フィールドが同様に乱れるはず）。

### 2.2 有力な仮説
1. **Rock5A側が status byte の bit0(emergency_stop)・bit5(is_signal_received) を正しく組み立てられていない**（常に0または1のダミー値になっている、ビット位置がMainBoardの想定とズレている、など）。
2. まだ残っている通信の途切れやすさ（300ms以内に新しい有効フレームが来ないことがある）。

### 2.3 一時的な診断ログ（追加済み、コミット`e30a4c5d`）
`src/unit/robot.c`の`Robot_RockUpdateSPI`末尾に、400msごとに1行、以下をUSART1へ出力するコードを追加した。**原因が分かったら削除すること。**

```
# rock_debug: estop=<0/1> sig=<0/1> vel=(<vx>,<vy>,<vw>) last_valid_ms_ago=<ms>
```

- `estop`・`sig`: MainBoardが解釈した`emergency_stop`・`is_signal_received`。
- `vel`: 受信した速度指令（mm/s, mm/s, mrad/s）。
- `last_valid_ms_ago`: 最後に有効なフレームを受信してからの経過時間。これが頻繁に300ms近くまで伸びていれば通信が不安定な証拠。
- 既存の`printf("Rock SPI stall detected...")`（`ROCK_SPI_STALL_TIMEOUT_MS=750ms`進捗なしで発火）も、通信の途切れやすさの直接的な手がかりになる。

**まだユーザーが書き込み・ログ取得を行っていない。** 取得方法: `tools\serial_log.ps1 -Port COM3 -Seconds 30`（ポート名は環境に合わせる。USB-UARTをPA9に、GNDを共通にして接続）。

## 3. 次にやること
1. ユーザーに書き込み・Rock5A接続テストを依頼し、上記の診断ログを`serial_log.ps1`で取得してもらう。
2. ログを解析し、
   - `estop`/`sig`が想定どおりの値（emergency_stop=0, is_signal_received=1）で来ているか
   - `vel`がRock5Aの操作と対応して変化しているか
   - `last_valid_ms_ago`が安定して小さいか、"Rock SPI stall detected"が頻発していないか
   を確認する。
3. **Rock5A側のSPI送信コードで、status byteの組み立て（特にemergency_stop・is_signal_receivedビット）を確認する。** ユーザーは`gh`（GitHub CLI）でRock5A側のリポジトリにアクセスできる想定。
   - もしログで`sig=0`が続いている・`estop=1`のままなら、ほぼ確実にRock5A側でこの2ビットが正しく送られていない。
   - もし`sig=1`・`estop=0`で来ているのに動かないなら、`vel`の値自体を見て、速度指令の単位・スケール（`docs/SPI_PROTOCOL.md`のvel_x/y/angular ×1000）がRock5A側と一致しているか確認する。
4. 原因を特定したら、診断ログ（`robot.c`の`# rock_debug`ブロック）を削除する。

## 4. 参考: このセッションでの他の気づき（今回の調査とは別件、記録として）
- 作業ディレクトリのHEADが、途中で正しいブランチ（`FW/MainBoard_V26_2_tc`）から外れてdetached HEAD状態になっていたことがあった。原因不明（おそらく複数チャットが同じ作業ディレクトリを同時に触ったため）。作業開始時は`git status`・`git rev-parse --abbrev-ref HEAD`で現在のブランチを確認すること。
- 別チャットの自動チューニング作業（`docs/HANDOFF_AUTOTUNE.md`参照）で、`local_controller.c/h`の変更がコミットされておらず、`auto_tune.c`が依存する関数が無くてビルドできなくなっていたことがあった（コミット`46b908f8`で復旧済み）。同じ作業ディレクトリを複数チャットで並行して触ると、コミット漏れや競合が起きやすいので注意。
