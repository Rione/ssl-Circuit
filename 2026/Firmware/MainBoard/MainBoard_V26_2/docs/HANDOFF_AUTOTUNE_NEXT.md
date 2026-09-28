# 自動チューニング 引き継ぎ（次のチャット用の入口）

最終更新: 2026-09-29　／　詳細: [`HANDOFF_AUTOTUNE.md`](HANDOFF_AUTOTUNE.md)（特に 8.1、10.7〜10.12.1）、前提: [`HANDOFF_VOLTAGE_CONTROL.md`](HANDOFF_VOLTAGE_CONTROL.md)

この文書だけで作業を再開できるようにまとめた。数値の根拠や経緯は `HANDOFF_AUTOTUNE.md` を見る。

## 0. 状態（2026-09-29 時点）
- 作業ディレクトリ: `c:\src\GitHub\ssl-Circuit\2026\Firmware\MainBoard\MainBoard_V26_2`
- ブランチ: **`FW/MainBoard_V26_2_tc`**（push 済み）。**このブランチの最新は Rock5A から動かせる**（`AUTOTUNE_IGNORE_ROCK_COMMANDS 0`、下記）
- 機体のフラッシュ: 調整値の保存は**消去済み**（`-ClearSaved`）。IMU の較正値も**もともと保存されていない**（0x000 は全部 0xFF）。
- 既定値（`src/config/parammeter.h`）: トルク上限 2.8 V、S字の加速度 5.0 m/s²、角加速度 38 rad/s²、`ka_lin` 0.5、`ka_lat` 0.5（**変更しない結論**、4章）。
- **Rock5A から動かせる設定**: `src/config/parammeter.h` の **`AUTOTUNE_IGNORE_ROCK_COMMANDS` を 0 にした**（2026-09-29、Rock5A から動かせる FW としてこのブランチの最新を使えるように）。
  - 0（今）: Rock5A の信号を受けている間は Rock5A の指令で動き、ST-Link から指示したテストは取り消される。**ST-Link のテストは、Rock5A の信号が来ていない（Rock5A を外す・止める）ときだけ走る。**
  - 1（開発用）: Rock5A の指令を無視する。2026-09-24〜28 の床の試験はすべて 1 で行った。Rock5A を付けたまま ST-Link のテストを走らせたいときは、`parammeter.h` で 1 にして書き込む（コミットには 0 で戻す）。
  - 設定の分岐は `src/mode/main_mode.c` の `MainMode_Loop`（`rock_commands_enabled`）。起動時に 1 なら UART に `# WORKAROUND: AUTOTUNE_IGNORE_ROCK_COMMANDS=1` と出る。
  - 参考: 1 を入れる前の、Rock5A から動かせる最後のコミットは `6aee606e`（09-24 13:13。ST-Link のテストは無い）。Rock5A から動かせるようになった修正は `c648dc8c`（`FW/MainBoard_V26.2_BugFix`、`c3d9a08a` でマージ）。
- ⚠ `AUTOTUNE_LOAD_SAVED 1` のままだと、フラッシュに保存された調整値が試合でも使われる（今は何も保存されていない）。

## 1. 目的と到達点
- 目的: 電圧制御（疑似トルク制御）の4輪オムニの足回りの値を、**一度書き込んで ST-Link から開始を指示すれば、機体（MainBoard）が自分で試験をくり返し、値を更新し、検証し、合格ならフラッシュに保存する**ようにする。PC は最後に結果を読むだけ。区間や動きの追加は人、決めた変数の最適化は自動。
- 到達点: **枠組みは完成し、床で一通り動いた**（最適化 → 検証 → 保存 → 電源を入れ直して読み込み、まで確認）。最初のタスクは FF 係数（`ka_lin`、`ka_lat`）で、「指令の加速度に対する実際の加速度の比を 1 にする」目的で作った。
- ところが、その目的で合わせた値（`ka_lat` 0.9〜1.0）は、**動作パターンの合計時間ではむしろ遅い**（4章）。**次は目的を「動作パターンの合計時間（制約つき）」に差し替える**のが本筋。

## 2. ユーザーとの進め方（必ず守る）
- **書き込み・電源・ST-Link の抜き差し・機体の置き直しは、ユーザーが行う。** こちらは、書き込みの前と走行の前に、何をするかを伝えて確認する。
- **走行は、ユーザーが「開始してください」と言ってから**指示を書く（`tools/autotune_start.ps1`）。指示を書いたら「ST-Link を抜いて離れてください。10秒後に走り出します」と伝える。終わったら、ユーザーが「終わりました」と言い、ずれ（前後・左右の cm、目視）を教えてくれるので、ST-Link で結果を読む。
- 作業の前に、**ブランチと状態を確認する**（ユーザーは別の作業でブランチを切り替えることがある。detached HEAD などになっていたら `git switch FW/MainBoard_V26_2_tc` で戻してよいと言われている）。
- コミットはローカルでよい。**push はユーザーが言ったときだけ。**
- 床のエリア: 今は **3.5 × 4.0 m**。機体は**エリアのちょうど中心**に、**前を長辺（4.0 m）に沿わせて**置く → `-Rect 4.0,3.5`（範囲は各端から 0.2 m 内側: 前後 ±1.8 m、左右 ±1.55 m）。エリアの大きさはユーザーに聞いて変わることがある。
- 返答は日本語。ユーザーは要点と数値を好む。

## 3. 環境の注意（踏んだもの）
- ビルド: PowerShell で `$env:TMP="$env:LOCALAPPDATA\Temp"; $env:TEMP=$env:TMP; make -j8`（Git Bash の make は TEMP の問題で gcc が失敗する）。成果物 `build/MainBoard_V26_2.elf/.bin`。
- gdb: `C:\ST\STM32CubeCLT_1.21.0\GNU-tools-for-STM32\bin\arm-none-eabi-gdb.exe`。**RAM のアドレスはビルドごとに変わる**ので、固定値を使わず、ツール（`tools/stlink_ram.ps1` の `Get-SymbolAddress`、`Get-GdbValue`）で ELF から引く。**書き込み済みの FW と手元の ELF が同じであること**が前提。
- ST-Link: `tools/stlink_ram.ps1`（`Read-Ram32`、`Read-RamBytes`、`Write-Ram32`）が `STM32_Programmer_CLI` を呼ぶ。「No STM32 target found」は、機体の電源が切れている・ケーブルが外れている。**走行中は ST-Link を抜く**（挿したままだと WheelUnit からの受信が途切れやすい）。
- **Python・ホスト用 C コンパイラは無い**。解析は PowerShell 5.1。スクリプト（.ps1）は **UTF-8 BOM 付き・CRLF**。PowerShell にヒアドキュメントは無い（git commit のメッセージは Bash の `git commit -F - <<'EOF'` か `-m` を使う）。新しいファイルは Write ツールで書くのが安全。
- 機体の保護: WheelUnit は +5.0 V が1秒続く・60℃超・電源 15 V 未満で出力を止める。電池は 24〜25 V。

## 4. 床で分かったこと（要点。2026-09-24）
### FF 係数と最適化
- FF 試験の軽量版（`-Speeds fast`、前後左右 × 2.5 m/s² × 3回、ka=0.5）: **車輪の比** 前後 1.05〜1.10・左右 0.89〜1.00、**IMU の比** 前後 0.78〜0.84・左右 0.54〜0.61。左右は車輪が空転しやすく、車輪の値が実加速度を表さない。→ 比の基準を「相対」（前後=車輪、左右=IMU の左右/前後）にした。
- 判定を ±0.03/0.05 にしたら雑音（同じ ka で比が ±0.04）に負けて 8 走行で収束しなかった → 3回×4方向、±0.05/±0.07、割線法は ka が 10% 以上動いたときだけ、に直したら 5 走行・約 3 分で合格。
  - 保存なし: `ka_lin` 0.469、`ka_lat` 0.913。保存あり: `ka_lin` 0.5、`ka_lat` 1.001（保存・電源の入れ直し後の読み込みを確認 → その後 `-ClearSaved` で消去）。
### 動作パターン（試合に近い 13 区間、合計時間。小さいほど速い）
| `ka_lat` | 合計 [s] | メモ |
|---|---|---|
| **0.5（既定）** | **18.68、18.58**（連続比較）/ 19.71（以前の基準） | 最速。左右の行き過ぎ 0〜12 mm |
| 0.75 | 19.44、19.58、19.56 | 約 0.9 秒遅い |
| 1.0 | 20.25 | 左右の3区間が各 +0.3 秒。行き過ぎは 0 mm。「キビキビ動く」（ユーザーの目視） |
- **結論: FF の比を 1 に合わせる目的は、動作パターンの速さと合わない。`ka_lat` は 0.5 のまま。** 0.3〜0.5 はまだ試していない。
- データ: `docs/data/motion_batch_ka_lat_20260924.csv`（run 1・3 が 0.75、2・4 が 0.5）。
### 位置のずれ（原因は未特定）
- 最適化（FF_FAST をくり返す）: 5 走行で**前へ 50 cm**（ka_lat 0.9）、**前へ 100 cm**（ka_lat 1.0）。ka_lat 0.5 の FF_FAST 1 回: 後ろ 3 cm・右 3 cm。ユーザーの見立て:「中心から右へ行くとき、少し前にずれて動く」。
- 動作パターン: 1.0 で右 20 cm、0.75 で右 15 cm・後ろ 5 cm、0.5 でほぼ 0、連続 4 回で後ろ 10 cm。
- **IMU の二重積分の位置（診断用に実装、`ff_drift`）は、12 本で 2 m 以上ずれて使えない**（バイアス・傾き）。
### それ以前（ランプ試験など。詳細は `HANDOFF_AUTOTUNE.md` 10.7〜10.11）
- 左右の実加速度は前後の約 0.67 倍。滑り始め: 前後 1.9〜2.3 V、左右 2.0〜2.7 V。斜めは電圧の余裕で頭打ち。
- ブレーキは加速より強い（ピーク 6 vs 4 m/s²）。ブレーキのみ 2 m/s → 0.50〜0.55 m で停止。
- IMU の加速度は車輪の約 0.77 倍を示す（どちらが正しいか未解決）。
### ブザー
- この機体では PA8（TIM1_CH1）に圧電素子がつながっておらず、音は出ない（`-BeepTest` で確認）。終わりの合図は LED0 だけ。

## 5. 実装済みの機能と使い方
### ST-Link からの開始（`src/control/auto_tune.c/.h`、`autotune_ctrl` 64 byte）
| test_id | 内容 | コマンド（`powershell -ExecutionPolicy Bypass -File tools\autotune_start.ps1 ...`） |
|---|---|---|
| 1 | 動作パターンを1周（`-KaLat` などで1回だけ上書き可） | `-TestId 1 [-KaLat 0.5]` |
| 2 | ランプ試験・速度別・ブレーキ・FF 試験 | `-TestId 2 -Speeds fast -Rect 4.0,3.5`（`0,1,1.5,2,2d,3,b2,b2.5,b2l,ff,fast`、`-Rotate`） |
| 4 | 自動最適化（FF 係数） | `-Optimize -Rect 4.0,3.5 [-NoSave] [-Ref rel/imu/wheel]` |
| 5 | 保存された調整値を消す（IMU 較正は残す。すぐ実行） | `-ClearSaved` |
| 6 | ブザーの確認（この機体では鳴らない） | `-BeepTest` |
| 7 | 動作パターンを ka_lat 0.75→0.5→0.75→0.5 で連続4回（間に15秒停止。値は `auto_tune.c` の `kBatchKaLat` に固定） | `-MotionBatch` |
- 状態を見るだけ: `-Status`。開始後 10 秒待つ（LED0 速い点滅）→ 走行。Rock5A の緊急停止（信号あり）で取り消し。**`AUTOTUNE_IGNORE_ROCK_COMMANDS 0` の今は、Rock5A の信号が来ていると取り消される（Rock5A を外すか止めてから）。**
- 1回だけの上書き: `-TractionV`、`-MaxAccel`、`-MaxAngAccel`、`-KaLat`（終われば既定値に戻る）。
### 結果の読み出し
- `tools\read_opt_results.ps1`（最適化: 走行ごとの ka・比・更新後の ka、結果、保存したか、今の `volt_tune`）
- `tools\read_ff_results.ps1`（FF 試験: 1本ごとの比、位置の診断）
- `tools\read_motion_results.ps1 -Csv logs\xxx.csv`（動作パターン: 走行ごとの合計・13区間の時間・スリップ・行き過ぎ・止まった位置。RAM に直近 8 回ぶん）
- `tools\read_ramp_results.ps1`、`tools\read_tcs_log.ps1`、`tools\check_wheels.ps1`（ランプ試験、波形、4輪の確認）
### 自動最適化（`src/control/optimizer.c/.h`）
- 冷却 10 秒 → 事前確認（電池 ≥ 21 V、WheelUnit の状態 bit1/bit2 = 0、過熱は最大 5 分待つ）→ FF_FAST（12 本、約 40 秒、終わりに原点へ戻る `PH_HOME`）→ 評価・更新 → 収束したら同じ値で検証 → 合格なら保存。
- 更新: r ∝ ka^e（初回 e=0.7、以降は割線法、0.4〜1.0）、1反復 ±30%、範囲 0.3〜1.2。走行は最大 8 回、全体 15 分。**走行中の安全停止は、やり直さず失敗で止まる**（位置が分からなくなるため）。
- 結果 `opt_result`（416 byte、反復ログ 16 件）。タスクの選択 `opt_task_mask`（今は FF だけ）、`opt_flags`（bit0 保存、bit1 IMU 基準、bit2 車輪基準）。
### フラッシュ保存（`src/config/volt_tune.c`、`src/unit/imu.c`）
- セクタ7（書くたびに 128 KB 丸ごと消える）。0x000 = IMU 較正（`ImuCalibData`）、**0x100 = 調整値のブロック**（magic "VTSF"、版 1、`ka_lin`、`ka_lat`、予約 12 個、チェックサム）。どちらを書くときも **512 byte を読んで差し替えて書き直す**（もう一方を消さない）。
- 起動時 `OmniDrive_Init` → `VoltTune_LoadSaved` → 有効なら `VoltTune_SetDefaults` の既定値を置き換える。API: `VoltTune_SaveTuned`、`VoltTune_ClearSaved`、`VoltTune_HasSaved`。
- ⚠ IMU 較正値が**ある状態**での「消えないこと」の実機確認は、まだしていない。

## 6. 未解決の問題
1. **FF_FAST をくり返すと原点が前へずれる**（左右の FF が強いほど大きい）。原因不明。IMU の積分は使えない。テープで測る以外の手段が今は無い（SSL-Vision は将来）。
2. **IMU の加速度が車輪の約 0.77 倍**（左右は約 0.6 倍）。どちらが実際に近いか未判定（テープで既知の距離を測る案をユーザーは保留中）。
3. IMU 較正値の保護の実機確認（較正値がある状態で調整値を保存し、較正値が残るか）。
4. 動作パターンの1回ごとのばらつき（±0.5 秒程度）。比較は交互に複数回走らせて平均する。

## 7. 次にやること（推奨順）
1. **最適化の目的を「動作パターンの合計時間（制約つき）」に差し替える。** 8.1 の決定: 停止位置のずれ ≤ 5 cm、向きの行き過ぎ ≤ 5°、安全停止なし、スリップ ≤ 2.8 V のときの値。
   - まず `ka_lat` を 0.3〜0.5（必要なら `ka_lin` も）で、候補ごとに動作パターンを 2 回（交互の順で）走らせ、合計時間の平均で決める。黄金分割か座標探索。
   - 部品: `-MotionBatch`（test_id 7）の連続走行を一般化し、候補の値を `autotune_ctrl` から渡せるようにする（今は `kBatchKaLat` に固定）。区間ごとの要約 `motion_results`（`src/control/motion_summary.c`、直近 8 回の循環）を `optimizer.c` の新しいタスクから読む。停止位置・行き過ぎ・スリップの制約は、要約の `end_d`・`h_over`・`slip` で判定する。
   - 動作パターンは原点から x −0.5〜1.5 m、y ±1.5 m を使う（3.5 × 4.0 m に収まる）。走行ごとに置き直さなくても、4 回で 10 cm 程度のずれ（ka_lat 0.5 のとき）。
2. 加速度・トルクの上限を**前後と左右で別**にする（左右は前後の約 0.67 倍しか出ない。`max_accel_lat` の設計）。ランプ試験の結果から候補を決め、1 と同じ方法で確かめる。
3. PI（`kp_lin`、`ki_lin`）を速度のステップ応答で調整。
4. 保存する変数を増やす（調整値のブロックの予約を使う。版を上げる）。
5. 位置のずれ・IMU と車輪の食い違いの調査（テープの実測、将来は SSL-Vision）。
6. Rock5A からの床の走行確認（`AUTOTUNE_IGNORE_ROCK_COMMANDS` は 0 にしてある）（`HANDOFF_VOLTAGE_CONTROL.md` 7章の6）。見つけた加速度の上限は Rock5A 側の経路計画にも反映する必要がある。

## 8. 主なファイル
- `src/control/auto_tune.c/.h`（開始の入口）、`optimizer.c/.h`（自動最適化）、`ramp_test.c/.h`（ランプ・FF 試験、`PH_HOME`、`ff_drift`）、`motion_summary.c/.h`（動作パターンの区間の要約）、`local_controller.c`（動作パターン本体）、`traction_control.c`（S字・スリップ検知）
- `src/unit/omni_drive.c`（電圧制御: center + FF + PI、`force_alloc`、FF 優先で縮める）、`src/control/wheel_voltage.c`（表と床の負荷分）、`src/unit/buzzer.c`、`src/unit/imu.c`
- `src/config/volt_tune.c/.h`（実行時の値・安全範囲・フラッシュ保存）、`src/config/parammeter.h`（既定値・開発用のフラグ）
- `src/mode/main_mode.c`（Rock5A 未接続時に `AutoTune_Poll` を呼ぶ）
- `tools/*.ps1`（5章）、`docs/data/*.csv`（床のデータ）
