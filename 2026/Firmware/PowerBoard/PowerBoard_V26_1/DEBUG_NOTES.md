# PowerBoard V26.1 デバッグメモ(キック/チャージ問題)

## 症状
- 一度キックするとチャージ(昇圧)しなくなる
- 何回かキックすると壊れる/たまに直る
- 正常時は「キュイーン」、壊れた時は「キューッ」と一瞬鳴ってすぐ止まる

## ハード前提(回路図より)
- 昇圧IC: LT3751UFD(フライバック式 高圧コンデンサチャージャ)
- MCU: STM32F303KBTx
- MCU I/O: LT_CHARGE=PB6(出力), LT_DISCHARGE=PA15(出力), LT_DONE=PB7(入力)
- キック: KICK1=PA1/TIM2_CH2, KICK2=PA2/TIM2_CH3 → FAN3227(U9) → IGBT Q5/Q6 で 340V を放電
- **FAULTピンはMCUに配線されていない**(BoostCircuitブロックの出力はDONEのみ)
- VOLTAGE_SENSE=PB0, BOOST_SENSE=PB1 はあるが **ADC未設定でFWは未使用**

## LT3751 重要仕様(データシートより)
- CHARGEピン: 1.5V以上で充電開始、0.3V以下で充電停止。**新しい充電/ラッチ解除にはCHARGEのLow→Highトグルが必須**
- DONEピン: **オープンコレクタ、アクティブLow**。完了またはFAULTで内部Trがオン→Lowに引く。未完了時は外部プルアップでHigh
- **DONEとFAULTは同じ極性**=MCUからは「完了」と「FAULT」を区別できない

## 根本原因
1. キック後の充電などでFAULT(過電圧/低電圧ラッチ等)が発生しラッチ停止
2. ラッチ中はDONEがLow → DoneCheck()=1(完了と誤認)
3. Kicker_Charge()が lt_charge=1 を書くだけでトグルしていなかったため、
   既に1の状態だとラッチ解除されず復活しなかった
4. キック(lt_charge 0→1)が偶然トグルになり、たまに復活していた

## 実施した修正(src/kicker.c, kicker.h, app/app.c)
- キックをワンショット化: ISRではPWM ON＋タイマ開始のみ(HAL_Delayでの
  ISRブロック=デッドロックを排除)。Kicker_Update()をMainAppループで呼び、
  KICK_TIME_MS(8ms)後にPWMを0へ戻す
- キック後に自動で Kicker_Charge() を呼び再チャージ
- Kicker_DoneCheck() を **反転**(アクティブLow対応): `!DigitalIn_Read(&lt_done)`
- Kicker_Kick の先頭に `if(!Kicker_DoneCheck()) return;`(完了時のみキック)
- **Kicker_Charge() でCHARGEを必ずLow→Highトグル**(CHARGE_RESET_US=200us)して
  ラッチ解除＋新充電サイクル開始

## まだ残る課題 / 次の一手
- 上記はFAULTからの「復帰」を確実にするが、FAULTの「発生原因」自体は未解決。
  キックのたびに物理的にFAULTし続ける場合、復帰してもまた止まる可能性。
- 根本対策候補:
  1. **ADCでBOOST_SENSEを読む**(PB1)。実際の340V電圧で「充電完了」「キック可否」を
     判断し、曖昧なDONEに依存しない
  2. FAULTピンをMCUに配線してDONEと区別できるようにする(基板改版)
  3. キック→自動再チャージのタイミング調整(ソレノイド電流が減衰するまで待つ等)
- DoneがFAULTでも1になるため、キックガードがFAULT時に誤って通る点に注意

## 調整パラメータ(kicker.c 先頭の#define)
- KICK_TIME_MS: キック通電時間(キック力)
- CHARGE_RESET_US: CHARGEトグルのLow時間(ラッチ解除)
