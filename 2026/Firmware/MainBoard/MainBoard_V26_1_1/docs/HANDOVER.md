# MainBoard ファームウェア 開発引き継ぎ資料

対象: MainBoard_V26_1_1 (STM32F446xx)
想定読者: C言語の授業経験・簡単なロボット制作経験がある理工学部ロボティクス学科3回生

このボードはロボットの「頭脳」で、上位機(Rock5A)からの指令を受けて、足回り(ホイールユニット)・キッカー・ドリブラーなど各サブボードに指示を出す**中継・統合役**です。制御アルゴリズムそのもの(画像処理・経路計画など)はこのボードの外(Rock5A側)にあり、ここでは「もらった指令を各ボードのプロトコルに変換して送る」「各ボードの状態を集めて返す」ことが中心になります。

---

## 1. システム全体像

```
                 SPI (Slave, 20byteフレーム, ヘッダ0xFF/フッタ0xAA)
  Rock5A(上位機) <───────────────────────────────► MainBoard (このボード)
  (画像処理・経路計画)                                  │
                                                        │ CAN (100kbps)
                                                        ├──────────► PowerBoard (電源・キッカー昇圧)
                                                        ├──────────► Dribbler基板
                                                        │
                                                        │ UART (250kbps, ヘッダ0xFF/フッタ0xAA)
                                                        ├──────────► WheelUnit ×4 (足回りモーター基板)
                                                        │
                                                        └──────────► UI基板 (バッテリー電圧・キッカー電圧の表示/操作)
```

- **MainBoard**: このリポジトリ。全体の取りまとめ役。
- **PowerBoard**: バッテリー電圧監視、キッカー用コンデンサの昇圧・放電。CANで指示を受け、CANで状態を返す。
- **Dribbler基板**: ドリブラーモーターの駆動とボール検知センサー。CANで指示/状態をやり取り。
- **WheelUnit**: オムニホイール4個それぞれに1枚。UARTで角速度指令を受け、エンコーダから実角速度を返す(`../../WheelUnit/`)。
- **CommonLib-C** (`../../CommonLib-C/`): 複数ボード共通の薄いラッパー群(CAN, UART, タイマー, PWM, GPIO, 移動平均フィルタ, 数学関数)。**このディレクトリを変更すると他ボードのファームウェアにも影響するので注意。**

---

## 2. ディレクトリ構成

| ディレクトリ | 内容 | 触る頻度 |
|---|---|---|
| `src/app` | エントリポイント (`Setup()` / `MainApp()`)、CAN受信割り込みハンドラ | 低 |
| `src/mode` | メインループ本体 (`MainMode_Loop`)。1周期ごとの処理順序を管理 | 中 |
| `src/control` | ローカル制御 (`LocalController`)。信号ロスト時の停止処理など | 中 |
| `src/unit` | 各アクチュエータ/センサーのドライバ層 (`robot`, `omni_drive`, `kicker`, `dribbler`, `ui`) | **高** |
| `src/config` | CAN ID定義、機体パラメータ | 中 (パラメータ調整で頻繁に開く) |
| `Core/`, `Drivers/` | STM32CubeMXの自動生成コード・HALライブラリ | **基本的に直接編集しない** |
| `docs/` | このファイルなど |
| `MainBoard_V26_1_1.ioc` | CubeMXのプロジェクトファイル(ピン配置・クロック・周辺機器設定) | ピン/クロック変更時のみ |

`Core/Src`, `Core/Inc` はCubeMXで `.ioc` を編集して再生成される部分です。`/* USER CODE BEGIN */`〜`/* USER CODE END */` の外側に書いたコードは再生成時に消えるので注意してください。基本的にアプリケーションロジックは全部 `src/` 側に書く設計になっています。

---

## 3. ビルド・書き込み

- ビルド: `make`(Makefile直書き、CubeIDE不要)。`arm-none-eabi-gcc` が必要。
  - `C_SOURCES` に `src` 以下と `../../CommonLib-C` 以下が `find` で自動的に含まれる(新しい `.c` ファイルを追加してもMakefile編集は不要)。
  - 生成物は `build/MainBoard_V26_1_1.bin` など。
- 書き込み:
  - `./SWDFlash.sh` : ST-Link + `STM32_Programmer_CLI` (STM32CubeProgrammerに同梱)
  - `./DAPFlash.sh` : DAPLink + OpenOCD (`brew install openocd`)
- `.ioc` はSTM32CubeMX(GUIツール)で開いて編集し、コード生成(Generate Code)すると `Core/` 以下が更新されます。ピン配置やクロック設定を変えたいとき以外は触らなくてOKです。

---

## 4. 処理の全体フロー

`Core/Src/main.c` はCubeMX生成のペリフェラル初期化(`MX_XXX_Init()`)をした後、`Setup()` → `MainApp()` を呼ぶだけです(`src/app/app.c`)。

```c
void Setup(void) {
  Robot_Initialize(&robot);      // GPIO/UART/CAN/SPIの初期化、各ユニットの初期化
  MainMode_Init(&main_mode, &robot);
}

void MainApp(void) {
  while (1) {
    MainMode_Loop(&main_mode);   // ここが毎周期回る
  }
}
```

`MainMode_Loop`(`src/mode/main_mode.c`)が心臓部で、**1ms周期**(`ROBOT_CONTROL_LOOP_DT_US`, `src/config/parammeter.h`)で以下を繰り返します。周期はループ末尾のビジーウェイトで維持しています(RTOS不使用、ベアメタル)。

1. `Robot_UpdateSensor` : バッテリー電圧をADCから取得
2. `Robot_UpdateFromUi` : UI基板からのボタン入力を受信、UI基板へ電圧等を返送(100周期に1回)
3. `Robot_RockUpdateSPI` : Rock5AとのSPI通信を進める(詳細は5.1)
4. `OmniDrive_Recv` : 各WheelUnitからのUART受信をパース
5. **通常時**(緊急停止でなく、Rock5Aから信号を受信中): ドリブル・キック・キッカー充放電・ホイール速度指令を送信
   **信号ロスト/緊急停止時**: `LocalController_Stop` でその場停止 (5.2)
6. `Robot_UpdateHeartBeat` : ハートビートLEDをsin波でPWM点灯(生きている合図)
7. LED表示更新(キッカー充電完了、信号受信中、など)

指令元は基本的に `RobotInfo`(`src/unit/robot.h`)という1つの構造体に集約されています。SPI受信で埋まり、UI入力で上書きされ、各Send関数がそこから値を読んで送信する、という「共有状態を介した」シンプルな設計です。

---

## 5. 通信プロトコルの詳細

### 5.1 Rock5A ⇔ MainBoard (SPI2, Slave)

- Rock5Aがマスター、MainBoardはスレーブ(`SPI2.Mode=SPI_MODE_SLAVE`)。**マスター側が周期的にクロックを出さないと通信が進まない**ため、MainBoard側から能動的に送ることはできず、割り込み(`HAL_SPI_TxRxCpltCallback`)駆動で受動的にやり取りします。
- 1フレーム20byte固定: `[0xFF][ペイロード18byte][0xAA]`。送信・受信とも同じフレームサイズで同期させています。
- ソフトウェアNSS(チップセレクトをソフトで見るタイプ)のスレーブは、ビットずれが起きると「完了もエラーもせずBUSYのまま固まる」ことがあるため、`robot.c` では
  - 直近2フレーム分(40byte)のスライディングウィンドウで受信データから正しいフレーム位置を毎回探し直す(`Robot_RockFindFrame`)、再同期の仕組み
  - 750ms(`ROCK_SPI_STALL_TIMEOUT_MS`)進捗が無ければ強制Abort→再Armするストール検出
  - TXは2面バッファ(ISRがArm中のバッファとmainが更新するバッファを分離)
  
  という対策が入っています。**SPI周りが不安定になったら、まずこの3つの仕組み(再同期ウィンドウ/ストールタイムアウト/ダブルバッファ)がどう動いているかを`robot.c`の`Robot_RockUpdateSPI`から追うとよいです。**
- 受信ペイロードの中身(速度指令・キック・ドリブル・座標・カメラ座標・statusビット)は `Robot_RockApplyRecvPacket` を、送信ペイロード(バッテリー電圧・ドリブル状態・キッカー電圧・ホイール角速度)は `Robot_RockBuildTxPacket` を参照してください。

### 5.2 MainBoard ⇔ WheelUnit ×4 (UART, 250kbps)

- `robot.c` で `huart5, huart6, huart2, huart3` の4本が `md_serials[0..3]` に割り当てられています。
- **送信は `serials[2]`(huart2)の1本だけ**に、4輪分の指令をまとめたパケットを一度に送信します(`omni_drive.c` の `OmniDrive_Send`)。これはWheelUnit側が同じ送信線をハードウェア的に共有(バス接続)しており、各WheelUnitが自分のBLDC IDに対応する2byteだけを抜き出して使う設計になっているためです(`WheelUnit/WheelDriver_V26_3/src/app/app.c` の `RecvSerial` 参照)。**受信(状態返送)は4本それぞれ独立**しているため `md_serials[0..3]` 全部を使います。
- フォーマット: `[0xFF][command 1byte][motor0 2byte][motor1 2byte][motor2 2byte][motor3 2byte][0xFF]`。command=1がDrive、0がFree(空転)。
- 速度指令には移動平均フィルタ(`MAF`, `CommonLib-C/filter`)がかかっています(急激な指令変化の緩和)。

### 5.3 MainBoard ⇔ PowerBoard / Dribbler基板 (CAN1, 100kbps)

CAN IDは `src/config/can_id.h` に集約。増やすときもここに追記してください。

| 方向 | ID | 内容 |
|---|---|---|
| TX | 0x10 | 電源オフ |
| TX | 0x11 | キッカー充電 |
| TX | 0x12 | キッカー放電 |
| TX | 0x13 | ストレートキック |
| TX | 0x14 | チップキック |
| TX | 0x20 | ドリブラー指令 |
| RX | 0x50 | PowerBoardから: 充電完了・電圧・コンデンサ電圧 |
| RX | 0x70 | Dribbler基板から: ボール検知状態 |

CAN受信はポーリングではなく割り込み(`HAL_CAN_RxFifo0MsgPendingCallback`, `src/app/app.c`)で処理し、届いたらすぐ `robot.info` に反映します。

### 5.4 MainBoard ⇔ UI基板 (UART4, 250kbps)

- `[0xFF][flagsビット列][kicker_power:4bit/dribbler_power:4bit][0xAA]` の4byte固定長。
- UI基板からのボタン入力(`is_locked`が立っていない間だけ有効)でドリブル/キック/充放電をローカル操作でき、遠隔操作(Rock5A)と共存できるようになっています。

---

## 6. 主要モジュール解説

すべて `構造体 + 関数(self, ...)` という「Cで書くオブジェクト指向」のスタイルで統一されています(例: `OmniDrive_SetVel(&robot->omni_drive, ...)` は他言語の `robot.omni_drive.set_vel(...)` に相当)。初期化は `Xxx_Init`、更新は `Xxx_Update`/`Xxx_Send`/`Xxx_Recv` という命名です。

- **`robot.h/.c`**: 全ユニットを束ねる最上位構造体。SPI通信の実装もここにある(サイズが大きいので新規メンバー追加時は既存の並びを崩さないよう注意)。
- **`omni_drive.h/.c`**: 並進・回転速度(mm/s, mrad/s)を4輪の角速度に変換(オムニホイールの逆運動学)。ホイール取り付け角は `parammeter.c` の `ROBOT_MOTOR_DEGREE` (55, 135, -135, -55度)。
- **`kicker.h/.c`**: キック・充電・放電のCAN送信。連射防止のタイマー付き(`ROBOT_KICK_INTERVAL_MS` など)。
- **`dribbler.h/.c`**: 現状パワーは0か最大かの2値制御。値変化時 or 一定間隔で再送。
- **`ui.h/.c`**: UI基板との4byteプロトコル送受信。
- **`main_mode.h/.c`**: 1周期分の処理順序をまとめた「司令塔」。
- **`local_controller.h/.c`**: 信号ロスト・緊急停止時の安全停止処理。`LocalController_TestMove`/`TestMoveForwardBack` は動作確認用のテストルーチンで、`main_mode.c` 内でコメントアウトされています(足回り単体の動作確認時に有効化して使う)。
- **`parammeter.h/.c`**: 機体形状・制御周期・タイマー間隔など調整用定数はここに集約。**マジックナンバーを直書きせずここに足すのが慣習**です。

---

## 7. つまずきやすいポイント / 既知の設計事情

- **`CommonLib-C` は共有ライブラリ**。ここを変更すると同時にビルドされる他ボード(WheelUnit, PowerBoard, Dribbler)にも影響します。バグ修正はよいですが、MainBoard専用の都合で仕様変更しないこと。
- **`.ioc` の USER CODEマーカーの外は再生成で消える。** CubeMXでピン/クロックを変えたら、生成後に `src/` 側のコードが壊れていないかビルドで確認。
- **SPIはスレーブでビットずれによるハングが起きうる**。5.1のストール検出/再同期の仕組みを壊さないよう変更時は注意。
- **足回りの送信は1本のUARTに4輪分をまとめて流すバス構成**。WheelUnit側のBLDC ID設定と `ROBOT_MOTOR_DEGREE` の並び順(motor0〜3に対応する物理配置)がズレると変な方向に動くので、新しい足回り基板を繋いだときはまずID設定を確認。
- **RTOSは使っていない**(ベアメタル、`while(1)`のポーリングループ)。割り込みは SPI/UART DMA/CAN受信/ADC DMA程度で、共有変数へのアクセスは `Robot_RockUpdateSPI` のように `__disable_irq()` で保護している箇所がある点に注意(排他制御が必要な処理を増やす場合は同様の配慮が要る)。
- **`do_direct_straight` / `do_direct_chip`**: ボールセンサー反応をトリガーにした自動キック機能。通常のキック指令と排他的に扱われる(`robot.c` の `Robot_SendKicker`)。ロスト時は誤発火防止のため明示クリアしている(`local_controller.c`)。

---

## 8. 動作確認の勧め

- ロボット単体での動作確認は `main_mode.c` 内の `LocalController_TestMove` / `TestMoveForwardBack` のコメントアウトを外すと、Rock5Aなしで足回りの動作確認ができます(確認後は必ずコメントアウトに戻す)。
- UI基板があれば、Rock5Aを繋がずにキック/ドリブル/充放電の単体確認が可能です。
- CANの単体確認は市販のCAN-USBアダプタ等でバス上のフレームを覗くのが早いです(ボーレート100kbpsに注意)。

---

## 9. 関連ボードのファームウェア(同リポジトリ内)

- `../../PowerBoard/PowerBoard_V26_1/`
- `../../Dribbler/Dribbler_V26_1_1/`
- `../../WheelUnit/WheelDriver_V26_2_1/`, `../../WheelUnit/WheelDriver_V26_3/`
- `../../CommonLib-C/`(共有ライブラリ、上記4ボード全てから参照)
