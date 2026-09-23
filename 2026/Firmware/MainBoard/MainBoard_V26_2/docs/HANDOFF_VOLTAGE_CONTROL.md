# ロボカップSSL 足回り：速度制御TCS → 電圧制御（疑似トルク制御）移行 引き継ぎ

## 1. 目的
- 4輪オムニ（ダイレクトドライブ）の足回りを、**WheelUnitの速度制御**から**MainBoard主導の電圧制御（疑似トルク制御）**に移行する。
- 狙い:
  - 4輪が独立に速度制御して押し合う問題をなくす。
  - トラクションコントロール（TCS）を「トルク（電圧）の上限」として素直に実装する。
  - 反応を速くする。
- Rock5A（上位）からの指令は、今までどおり機体速度 (vx, vy, ω) のまま。MainBoardが機体速度を制御して、各輪の電圧に変換する。

## 2. システム構成と前提
- **MainBoard**（STM32F446, ベアメタルC, 1ms制御ループ）
  - パス: `2026/Firmware/MainBoard/MainBoard_V26_2`
  - ブランチ: `FW/MainBoard_V26_2_tc`（最新 `50e512a2 速度制御でのTCSはそれなりにできた`、push済み）
- **WheelUnit ×4**（STM32G431 + STSPIN32G4、センサ付きFOC）
  - パス: `2026/Firmware/WheelUnit/WheelDriver_V26_3`
  - 最新FWはブランチ `origin/FW/WheelDriver_V26.3`（最新 `422560b8 電圧制御対応`）にある。
  - ⚠ `FW/MainBoard_V26_2_tc` ブランチ内の WheelUnit ソース（`src/app/app.c` のみ）は古い版。フレーム形式も実機と合わないので参照しないこと。
  - ⚠ ユーザーは最新のWheelUnit FWをまだpull・書き込みしていない。**これまでのCSVは中身不明の古いFWで取ったもの。**
- 機体パラメータ:
  - モータ配置角 `{55°, 135°, -135°, -55°}`、車輪半径 0.03m、車輪基底半径 0.075m
  - IMU: LSM6DSO32。取付はセンサ-y = 機体前方(+x)、センサ+x = 機体左(+y)
- 重心位置は未知。電流センサは無い（トルクは電圧と逆起電力のモデルで推定するしかない）。

## 3. MainBoard 側の現状（速度制御ベースのTCS、`50e512a2`）
- 処理の流れ（1msごと）:
  上位指令 → `TCS_Update`（S字加減速 → 対地速度推定 → スリップ検知 → 介入）→ 逆運動学 → UARTでWheelUnitへ角速度指令
- 主なファイル:
  - `src/control/traction_control.c/.h` … TCS本体
  - `src/unit/omni_drive.c/.h` … 順運動学、送受信、`OmniDrive_SetVelEx`
  - `src/unit/imu.c` … 機体座標の加速度、バイアス
  - `src/control/tcs_log.c` … RAMロガー
  - `src/control/local_controller.c` … テスト
  - `src/config/parammeter.h` … パラメータ
- 実装済みで、電圧制御でも**そのまま使える**もの:
  - 順運動学（4輪→機体速度を最小二乗の疑似逆行列で計算。`OmniDrive.fk`, `OmniDrive_GetVelF`）
  - IMUの機体座標変換と起動時バイアス補正（ジャイロ3軸＋加速度xy。`IMU_CALIBRATE_ON_BOOT`）
  - 対地速度推定（IMU加速度の積分＋グリップ中のみオドメトリで相補補正）
  - スリップ検知3系統:
    - 幾何残差 `w0 - k·w1 + k·w2 - w3`（k=√2·sin55°）
    - 旋回残差（オドメトリ角速度 − ジャイロ）
    - 加速度残差（指令加速度方向の a_odom − a_imu）
  - 制御周期の実測dt（`OmniDrive_MeasureDt`）
  - 車輪速度の単発異常値除去（20rad/s超の跳びは次フレームで確認するまで保留）
  - Rock5A SPI のストール復帰を非ブロッキング化（RCCリセット＋`HAL_SPI_Init`）。以前は `HAL_SPI_Abort` が約1秒ごとに225ms制御ループを止めていた。
  - RAMロガー（下記6）
- 電圧制御で**作り直す**もの:
  - S字加減速（速度指令の平滑化）
  - `TCS_LimitToGround`（車輪速度を対地速度＋0.15m/sで頭打ち）
  - `accel_gain`（S字の加速度上限の倍率。スリップ突入時に最大4割カットし、解除後に6/sで回復）
  - → いずれも「トルク（電圧）上限」「機体速度フィードバック」の形に置き換える。

## 4. CSVで分かったこと（古いWheelUnit FWでの結果）
- **WheelUnitは目標回転数に追従できていなかった**（ログの `t*`=目標、`w*`=実測）:
  - 前進巡航: 全輪が目標の+7〜17%
  - 後退巡航: w0・w3が+30〜50%、w1はほぼ目標どおり
  - 発進直後: w3が目標の約2.8倍
  - 原因の推定（最新FWの `bldc.c` から）: 速度PIのゲインが低く積分が遅い（kp=0.05, ki=0.5 [V/(rad/s)]）ため、加速中にたまった積分が巡航で抜けない。加えて、4輪の独立した速度制御が3自由度の機体の上で押し合っている。
- MainBoardは横方向の指令を出していない（`cmd_vy`=0）のに、車輪ごとのずれでヨーが出ていた（ジャイロで最大±1.3rad/s）。機体が傾いたまま走るので、**横にずれる**。
- 巡航中も幾何残差が定常的に閾値（15rad/s）を超え、`is_slipping` が解除されず、`accel_gain` が0.25に張り付く（特に後退巡航）。反転時の減速が遅くなり、行き過ぎる。
- 4150ms付近の急減速（IMU -31m/s²、ジャイロ -2.8rad/s）。w2が目標+42rad/sに対して-13.6rad/sまで逆転、w1も低下。衝突ではないとのこと。WheelUnitの低電圧停止（下記5）の可能性があるが、状態バイトを記録していなかったため未確認。
- ⚠ **ユーザー報告: IMUのキャリブレーションデータが入っていなかった可能性がある。** 次の現象の一部はIMUのドリフトによるかもしれない。電圧制御の評価の前に、IMUのキャリブレーション状態を必ず確認すること。
  - ヘディング保持（ジャイロ積分）のずれ → 横ずれ
  - `ground_vy` の±0.6m/sのずれ
  - `rot_res` の偏り
  - 加速度残差の誤検知

## 5. WheelUnit 最新FW（`origin/FW/WheelDriver_V26.3` @ `422560b8`）の仕様と注意点
- **通信**（UART 250kbps 8N1）:
  - MainBoard→WheelUnit: `0xAA, cmd, m0H, m0L, m1H, m1L, m2H, m2L, m3H, m3L, 0xFF`（11byte）
    - MainBoardは1本のUART（`serials[2]`）から全輪に送り、各WheelUnitは自分のIDの枠を読む。
  - `cmd=1`: 速度モード（値×0.01 = rad/s）
  - `cmd=2`: **電圧モード**（値×0.01 = V、±`MAX_AMP_VOLT` でクランプ）→ `BLDC_VoltageControl()`
  - それ以外: mode 0（停止）
  - WheelUnit→MainBoard: `0xFF, status, speedH, speedL, 0xAA`（500µs周期。speed×0.01 = rad/s）
    - status: bit0 = mode≠0、bit1 = 電圧範囲外、bit2 = 過熱
  - 受信タイムアウト2.0sで mode 0。
- **電圧モードの中身**:
  - `amp = LPF(amp_volt / supply_volt)`（係数0.5、±0.5にクランプ）→ SVPWM。電源電圧での正規化はWheelUnit側で行う。
  - 速度モードにある逆起電力FF・摩擦FF・PIは**かからない**（生の電圧）。
  - 進角 `K_ADV=0.01`×角速度（±1.5rad）は電圧モードでもかかる。
- **モータ・制御定数**:
  - Kv=185rpm/V → 逆起電力定数 `K_SPEED_FF=0.052 V/(rad/s)`
  - 極対数8
  - 最大印加電圧 `MAX_AMP_VOLT=5.0V`（7.5→6→5と下げられてきた）
  - 回転数は5msごとに角度差から計算し、LPF 0.5
  - FOC制御周期100µs
  - 摩擦FF 0.25V（速度モードのみ）
- ⚠ **注意点**:
  - **5V上限と逆起電力**: 2m/s（約60rad/s）で逆起電力が約3.1V。加速に使える電圧は約1.9Vしか残らない。無負荷の最高回転数は約96rad/s（約2.9m/s）。
  - **ホイールロック検知は `amp_volt == +MAX_AMP_VOLT`（正側のみ）が1秒続くと出力を0にする**（5秒まで）。電圧モードで+5Vを1秒以上指令し続けると切れる。しかも正側だけで非対称。
  - **電源電圧が15V（`SUPPLY_VOLTAGE_MIN_LIMIT`）を下回ると `BLDC_Stop()`**。15.5Vを超えるまで復帰しない。
  - **`BLDC_Stop()`（mode 0 も同じ）は3相とも50%デューティになり、短絡ブレーキになる。** MainBoardの `OmniDrive_SetFree`（cmd 0）も空転ではなくブレーキになる。
  - 過熱（60℃超）でも停止し、50℃で復帰。
  - 停止中の処理に `HAL_Delay(100)` の点滅がある（その間、制御ループは止まる）。
  - キャリブレーション値（エンコーダ範囲・オフセット・ID）はフラッシュから読む（`BLDC_Init(false, ...)`）。書き込み後に起動ログで確認すること。

## 6. テスト・ログ環境（MainBoard）
- テスト: `LocalController_TestTCSAcceleration`。Rock5Aから未受信のとき `main_mode.c` から呼ばれる。
  - 0〜10s 静止待機（LED0 0.5s点滅）
  - 10〜20s 2000mm/sで1.5m前後往復（TCS常時有効、ジャイロ積分ヘディングのPD保持 Kp=2.5, Kd=0.2）
  - 20s以降停止。さらに40s後にCSVを出力する。
  - ⚠ Rock5A接続中に信号が途切れても、このテストが動く経路のまま。試合前に `LocalController_Stop` に戻すこと。
- CSV: USART1（PA9/PA10, **250000bps 8N1**）。10ms周期・最大1000サンプル。
  - 列: `t_ms,tcs_on,slip,target_vx,cmd_vx,odom_vx,odom_vy,ground_vx,ground_vy,a_odom_x_cm,a_imu_x_cm,accel_res_cm,geom_res_x10,rot_res_mrad,accel_gain_x1000,w0..w3_x100,cmd_vy,cmd_w_mrad,gyro_mrad,t0..t3_x100`
  - `slip`: bit0=幾何、bit1=旋回、bit2=加速度、bit7=is_slipping
  - `_write` は送信タイムアウト10msなので、250kbpsでは1回の送信は約250文字まで。長いヘッダは `fputs` と `fflush` で分割している。
- ビルド: **PowerShell** で `make clean; make -j`。Git BashはTEMPの問題でgccが失敗する。サイズ確認は `arm-none-eabi-size build/MainBoard_V26_2.elf`。
- PCにホスト用Cコンパイラ・Pythonは無い（`python` はStoreのスタブ）。数値確認はPowerShellで行った。

## 7. 次にやること（推奨順）
1. **WheelUnit最新FWをpullして4輪に書き込む**（`origin/FW/WheelDriver_V26.3`）。
   - IDと向き、キャリブレーション値の読み込みを確認する。
   - MainBoardの作業ブランチを残したいなら `git worktree` を使う。
2. **IMUのキャリブレーション状態を確認する**（起動時に静止しているか、バイアス値が妥当か）。
3. 速度モードのまま今のTCSテストを1回走らせ、`w*` と `t*` を比べて、新FWでの基準データにする。
4. **車輪を浮かせて電圧モードを単体評価する。** MainBoardにテストモード（`cmd=2`）を追加し、各輪にステップ電圧・ランプ電圧をかけて回転数を記録する。
   - 前進・後退の対称性
   - 逆起電力定数（0.052の妥当性）
   - 摩擦で動き出さない範囲（デッドバンド）
   - 応答時定数
   - ログには車輪ごとの状態バイトと電源電圧も入れる。
5. **MainBoardで電圧制御を実装する**。段階的に:
   - (a) フィードフォワードのみ: 電圧 = 逆起電力定数×ω ＋ 加速に必要な電圧 ＋ 摩擦補償
   - (b) 機体速度 (vx, vy, ω) のフィードバック（オドメトリ＋IMU）
   - (c) TCSをトルク（電圧）上限と、スリップ輪のトルクカットとして組み込む
   - 押し合い成分（幾何拘束の零空間）は0にする。
   - 5V上限・ホイールロック検知・低電圧停止を踏まえて上限を設計する。
