#include "app.h"

PwmOut LED1;
PwmOut LED2;
PwmOut LED3;
PwmOut LED4;

DigitalOut CAN_LED;
DigitalOut MD_SLEEP;

DigitalIn SW;

CanBus can;
CanData canRecvData;

Serial serial;

Timer can_send_interval_timer;

uint16_t adc_val[4];  // 0: Current, 1: BallSensor
#define MOTOR_CURRENT_IDX 0
#define BALL_SENSOR_IDX 1

#define CAN_SEND_INTERVAL_MS 10  // 10ms

// ===========================================================================
// 試験用ビルド設定
// ===========================================================================
// 1: ボール検知でドリブラを回す追従試験モード。CAN受信での駆動は行わない
// BS_IN の接続ピン。Makefile の BSIN_PC2=1 で切り替わる
#ifdef BSIN_PIN_PC2
#define BSIN_GPIO_PIN GPIO_PIN_2
#define BSIN_PIN_NAME "PC2 (ADC1_IN12)"
#else
#define BSIN_GPIO_PIN GPIO_PIN_3
#define BSIN_PIN_NAME "PC3 (ADC1_IN13)"
#endif

// 1: ボール検知でドリブラを回す追従試験モード (CAN受信での駆動は無効)
// 0: 通常動作。CAN(ID 0x20)で受けた速度でドリブラを回す
#define BALL_FOLLOW_TEST 0
// 回転速度[%] (MAX_SPEED_LEVEL に対する割合)
#define TEST_SPEED_PERCENT 50
// 1: 起動時の長い診断(モータ掃引/点滅テスト/実使用モニタ)を省略する
#define SKIP_SLOW_DIAG 1
// ベンチ安全対策: 検知が張り付いた場合に回し続けない上限[ms]
#define TEST_MAX_RUN_MS 30000

#define CAN_RECV_ID 0x20
#define CAN_SEND_ID 0x70

// ---------------------------------------------------------------------------
// 起動診断用
// ---------------------------------------------------------------------------
// 現在どのSetupステップにいるかを保持する。ハングやフォールト時に
// Error_Handler / HardFault_Handler から参照して停止箇所を特定する。
volatile uint32_t g_diag_step = 0;

// 各サブシステムの初期化結果。失敗してもSetupは止めず、最後にまとめて報告する。
static bool can_ok = false;
static bool adc_ok = false;
static bool can_core_ok = false;  // rawテストでCANコア自体が生きていたか
static bool can_core_tested = false;

// CAN受信の生存確認カウンタ。起動ログを取り逃しても実行中に状況が分かるようにする。
static volatile uint32_t can_rx_count = 0;       // 受信した全フレーム数
static volatile uint32_t can_rx_match_count = 0; // CAN_RECV_ID に一致した数
static volatile uint8_t can_last_level = 0;      // 最後に受け取った速度レベル

#define DIAG_STEP(n, msg)        \
  do {                           \
    g_diag_step = (n);           \
    printf("[STEP %u] " msg "\r\n", (unsigned)(n)); \
  } while (0)

// キャリブレーションが完了しない場合に無限ループしないための上限[ms]
#define CALIB_TIMEOUT_MS 10000

static void CanDumpMsr(const char *tag);

static uint32_t can_busoff_count = 0;
static uint32_t g_vdda_mv = 3300;  // AdcCoreDiag で実測して更新

// 2枚の基板を比較するための指標
static uint32_t g_bs_off_avg = 0;
static uint32_t g_bs_on_avg = 0;
static int32_t  g_bs_delta = 0;
static int      g_bs_pin_up = -1;
static int      g_bs_pin_dn = -1;

// CAN_ESR の LEC(Last Error Code) を名前で返す
static const char *CanLecName(uint32_t lec) {
  switch (lec) {
    case 0: return "None";
    case 1: return "Stuff";      // ビットスタッフ違反。ノイズ/ボーレート不一致
    case 2: return "Form";       // フレーム形式異常
    case 3: return "ACK";        // 誰もACKを返していない = 相手がいない/受信できていない
    case 4: return "BitRecess";  // レセッシブを送ったのにドミナントが読めた
    case 5: return "BitDom";     // ドミナントを送ったのに読めない = 配線/終端
    case 6: return "CRC";
    default: return "SW";
  }
}

// CAN_ESR を人が読める形で出す
static void CanDumpEsr(const char *tag) {
  uint32_t esr = CAN1->ESR;
  printf("%s ESR=0x%08lX TEC=%lu REC=%lu LEC=%s%s%s%s\r\n", tag,
         (unsigned long)esr, (unsigned long)((esr >> 16) & 0xFFU),
         (unsigned long)((esr >> 24) & 0xFFU),
         CanLecName((esr >> 4) & 7U),
         (esr & CAN_ESR_EWGF) ? " EWG" : "",
         (esr & CAN_ESR_EPVF) ? " EPV" : "",
         (esr & CAN_ESR_BOFF) ? " BUS-OFF" : "");
}

// バスオフからの復帰。ABOM=ENABLEなら自動復帰するが、
// 明示的に初期化モードを一巡させて確実に復帰シーケンスを起動する。
static void CanBusOffRecover(void) {
  CAN1->MCR |= CAN_MCR_INRQ;
  uint32_t t = HAL_GetTick();
  while (((CAN1->MSR & CAN_MSR_INAK) == 0U) && (HAL_GetTick() - t < 10U)) {
  }
  CAN1->MCR &= ~CAN_MCR_INRQ;
  t = HAL_GetTick();
  while ((CAN1->MSR & CAN_MSR_INAK) && (HAL_GetTick() - t < 50U)) {
  }
}

// Can_Init() は失敗時に Error_Handler() へ飛んで無言で停止するため、
// 切り分け用に「失敗しても戻ってくる」版をここに用意する。
static bool CanInitChecked(void) {
  can.hcan = &hcan1;
  can.myId = 0;

  HAL_StatusTypeDef st = HAL_CAN_Start(&hcan1);
  if (st != HAL_OK) {
    printf("  !! HAL_CAN_Start failed: st=%d State=%d ErrorCode=0x%08lX\r\n",
           (int)st, (int)hcan1.State, (unsigned long)hcan1.ErrorCode);
    printf("  !! CAN1 ESR=0x%08lX TSR=0x%08lX\r\n",
           (unsigned long)CAN1->ESR, (unsigned long)CAN1->TSR);
    CanDumpMsr("!!");
    printf("  -> INAKが降りない。CANトランシーバ未給電/未接続、\r\n");
    printf("     バス未終端、PB8(RX)がLOW固定 などを疑う\r\n");
    return false;
  }

  CAN_FilterTypeDef filter = {
      .FilterIdHigh = 0,
      .FilterIdLow = 0,
      .FilterMaskIdHigh = 0,
      .FilterMaskIdLow = 0,
      .FilterFIFOAssignment = CAN_FILTER_FIFO0,
      .FilterBank = 0,
      .FilterMode = CAN_FILTERMODE_IDMASK,
      .FilterScale = CAN_FILTERSCALE_32BIT,
      .FilterActivation = ENABLE,
      .SlaveStartFilterBank = 14,
  };
  if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) {
    printf("  !! HAL_CAN_ConfigFilter failed: ErrorCode=0x%08lX\r\n",
           (unsigned long)hcan1.ErrorCode);
    return false;
  }

  return true;
}

// CAN_MSR の中身を人が読める形で出す。bit11 RX は CAN1_RX ピンの現在レベル。
static void CanDumpMsr(const char *tag) {
  uint32_t msr = CAN1->MSR;
  printf("  %s MSR=0x%08lX INAK=%lu SLAK=%lu ERRI=%lu WKUI=%lu "
         "SAMP=%lu RX(pin)=%lu\r\n",
         tag, (unsigned long)msr, (unsigned long)((msr >> 0) & 1U),
         (unsigned long)((msr >> 1) & 1U), (unsigned long)((msr >> 2) & 1U),
         (unsigned long)((msr >> 3) & 1U), (unsigned long)((msr >> 10) & 1U),
         (unsigned long)((msr >> 11) & 1U));
}

// CAN1_RX(PB8)の電気的状態を調べる。
// CANコントローラは初期化モードを抜けるのに11連続レセッシブ(HIGH)を必要とするため、
// このピンがLOW固定だと HAL_CAN_Start() は必ずタイムアウトする。
static void CanPinDiag(void) {
  GPIO_InitTypeDef g = {0};
  g.Pin = GPIO_PIN_8;
  g.Mode = GPIO_MODE_INPUT;
  g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  g.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &g);
  HAL_Delay(2);
  int up = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_SET) ? 1 : 0;

  g.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &g);
  HAL_Delay(2);
  int dn = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_SET) ? 1 : 0;

  // AF9(CAN1_RX)に戻す
  g.Mode = GPIO_MODE_AF_PP;
  g.Pull = GPIO_NOPULL;
  g.Alternate = GPIO_AF9_CAN1;
  HAL_GPIO_Init(GPIOB, &g);

  printf("  PB8(CAN1_RX): pull-up=%d pull-down=%d\r\n", up, dn);
  if (up == 1 && dn == 0) {
    printf("  -> フローティング。トランシーバ未実装/未給電/RXD未配線\r\n");
  } else if (up == 0 && dn == 0) {
    printf("  -> 外部がLOW駆動。バスdominant固定/CANH-CANL短絡/PB8のGND短絡\r\n");
  } else if (up == 1 && dn == 1) {
    printf("  -> 外部がHIGH駆動(recessive)。ピン自体は正常\r\n");
  } else {
    printf("  -> 不定\r\n");
  }
}

// CANコアが本当に動いているかを、HALの状態機械を介さずレジスタ直叩きで確認する。
// HAL_CAN_DeInit() は MspDeInit でクロックを切った"後"に MCR.RESET を書くため
// マスターリセットが効かない。ここでは自前でリセットしてから試す。
static void CanRawDiag(void) {
  printf("  -- raw CAN test --\r\n");
  can_core_tested = true;
  printf("  SYSCLK=%luHz HCLK=%luHz PCLK1=%luHz\r\n",
         (unsigned long)HAL_RCC_GetSysClockFreq(),
         (unsigned long)HAL_RCC_GetHCLKFreq(),
         (unsigned long)HAL_RCC_GetPCLK1Freq());
  printf("  APB1ENR.CAN1EN=%lu\r\n",
         (unsigned long)((RCC->APB1ENR >> 25) & 1U));

  // ビットレートの実測値を計算 (1 + BS1 + BS2) x Prescaler / PCLK1
  uint32_t tq = 1U + 13U + 2U;
  uint32_t bitrate = HAL_RCC_GetPCLK1Freq() / (25U * tq);
  printf("  bit timing: %luTQ x presc25 -> %lubps\r\n", (unsigned long)tq,
         (unsigned long)bitrate);

  __HAL_RCC_CAN1_CLK_ENABLE();
  printf("  before: MCR=0x%08lX BTR=0x%08lX\r\n", (unsigned long)CAN1->MCR,
         (unsigned long)CAN1->BTR);
  CanDumpMsr("before:");

  // 自前のマスターリセット
  CAN1->MCR |= CAN_MCR_RESET;
  uint32_t t = HAL_GetTick();
  while ((CAN1->MCR & CAN_MCR_RESET) && (HAL_GetTick() - t < 100U)) {
  }
  printf("  master reset: %s\r\n",
         (CAN1->MCR & CAN_MCR_RESET) ? "RESETビットが降りない(クロック停止を疑う)"
                                     : "OK");
  CanDumpMsr("reset :");

  // DBF=1(リセット既定)だとデバッガ接続時にCANが凍結されうるので落としておく
  CAN1->MCR &= ~CAN_MCR_DBF;

  // 初期化モードへ (リセット直後はスリープなのでSLEEPも落とす)
  CAN1->MCR |= CAN_MCR_INRQ;
  CAN1->MCR &= ~CAN_MCR_SLEEP;
  t = HAL_GetTick();
  while ((((CAN1->MSR & CAN_MSR_INAK) == 0U) ||
          ((CAN1->MSR & CAN_MSR_SLAK) != 0U)) &&
         (HAL_GetTick() - t < 100U)) {
  }
  printf("  enter init: %lums\r\n", (unsigned long)(HAL_GetTick() - t));
  CanDumpMsr("init  :");

  // SILENT+LOOPBACK。RXピンは内部で切り離されるのでバスの状態に依存しない
  CAN1->BTR = (uint32_t)CAN_MODE_SILENT_LOOPBACK | (uint32_t)CAN_SJW_1TQ |
              (uint32_t)CAN_BS1_13TQ | (uint32_t)CAN_BS2_2TQ | (25U - 1U);
  printf("  BTR=0x%08lX LBKM=%lu SILM=%lu\r\n", (unsigned long)CAN1->BTR,
         (unsigned long)((CAN1->BTR & CAN_BTR_LBKM) ? 1U : 0U),
         (unsigned long)((CAN1->BTR & CAN_BTR_SILM) ? 1U : 0U));

  // 通常モードへ抜ける = 11連続レセッシブを数えられるかの試験
  CAN1->MCR &= ~CAN_MCR_INRQ;
  t = HAL_GetTick();
  while ((CAN1->MSR & CAN_MSR_INAK) && (HAL_GetTick() - t < 500U)) {
  }
  uint32_t elapsed = HAL_GetTick() - t;

  if (CAN1->MSR & CAN_MSR_INAK) {
    printf("  leave init: FAILED (500ms)\r\n");
    CanDumpMsr("fail  :");
    printf("  ESR=0x%08lX LEC=%lu TEC=%lu REC=%lu\r\n",
           (unsigned long)CAN1->ESR, (unsigned long)((CAN1->ESR >> 4) & 7U),
           (unsigned long)((CAN1->ESR >> 16) & 0xFFU),
           (unsigned long)((CAN1->ESR >> 24) & 0xFFU));
    printf("  => ループバックでもINAKが降りない。ただしクロックは生きている\r\n");
    printf("     (マスタリセット成功/レジスタ書換え可/PCLK1正常)ので\r\n");
    printf("     このテスト自体の妥当性を alt-pin test で確認する\r\n");
  } else {
    printf("  leave init: OK (%lums)\r\n", (unsigned long)elapsed);
    CanDumpMsr("ok    :");
    can_core_ok = true;
    printf("  => CANコアは正常。原因はPB8のLOW固定(基板外部)に確定\r\n");
  }

  // 後始末: 自前でリセットしてからNORMALで再初期化 (Startはしない)
  CAN1->MCR |= CAN_MCR_RESET;
  t = HAL_GetTick();
  while ((CAN1->MCR & CAN_MCR_RESET) && (HAL_GetTick() - t < 100U)) {
  }
  hcan1.State = HAL_CAN_STATE_RESET;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  HAL_CAN_Init(&hcan1);
}

// CANコアの生死を確定させる決定打。
// CAN1_RX は PB8 のほかに PA11(AF9) にも割り当てられる。PA11 は本基板で未使用なので、
// PB8 を切り離して PA11 を内部プルアップ付きの CAN1_RX にすれば、外部に何も繋がって
// いなくても recessive(HIGH) が供給される。これで NORMAL モードの起動が通れば、
// CANコアは完全に健全で、問題は PB8 とその先(トランシーバ/配線)に限定される。
static void CanAltPinDiag(void) {
  printf("  -- alt-pin test (CAN1_RX を PA11 へ付け替え) --\r\n");

  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef g = {0};

  // PB8 を完全に切り離す (アナログ入力にして CAN1_RX から外す)
  g.Pin = GPIO_PIN_8;
  g.Mode = GPIO_MODE_ANALOG;
  g.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &g);

  // PA11 を CAN1_RX(AF9) + 内部プルアップ に
  g.Pin = GPIO_PIN_11;
  g.Mode = GPIO_MODE_AF_PP;
  g.Pull = GPIO_PULLUP;
  g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  g.Alternate = GPIO_AF9_CAN1;
  HAL_GPIO_Init(GPIOA, &g);
  HAL_Delay(2);

  // CANを自前リセットしてNORMALモードで起動を試す
  CAN1->MCR |= CAN_MCR_RESET;
  uint32_t t = HAL_GetTick();
  while ((CAN1->MCR & CAN_MCR_RESET) && (HAL_GetTick() - t < 100U)) {
  }
  CAN1->MCR &= ~CAN_MCR_DBF;
  CAN1->MCR |= CAN_MCR_INRQ;
  CAN1->MCR &= ~CAN_MCR_SLEEP;
  t = HAL_GetTick();
  while ((((CAN1->MSR & CAN_MSR_INAK) == 0U) ||
          ((CAN1->MSR & CAN_MSR_SLAK) != 0U)) &&
         (HAL_GetTick() - t < 100U)) {
  }

  // 通常(NORMAL)モード。ループバックは使わない
  CAN1->BTR = (uint32_t)CAN_SJW_1TQ | (uint32_t)CAN_BS1_13TQ |
              (uint32_t)CAN_BS2_2TQ | (25U - 1U);
  CanDumpMsr("PA11  :");

  CAN1->MCR &= ~CAN_MCR_INRQ;
  t = HAL_GetTick();
  while ((CAN1->MSR & CAN_MSR_INAK) && (HAL_GetTick() - t < 500U)) {
  }
  uint32_t elapsed = HAL_GetTick() - t;

  if (CAN1->MSR & CAN_MSR_INAK) {
    printf("  PA11でもINAKが降りない (500ms)\r\n");
    CanDumpMsr("fail  :");
    printf("  => CAN1ペリフェラル自体が死んでいる可能性が高い。MCU交換を検討\r\n");
  } else {
    printf("  PA11でINAKが降りた (%lums)\r\n", (unsigned long)elapsed);
    CanDumpMsr("ok    :");
    printf("  => CANコアは完全に健全。ループバックテストの方が当てにならなかった\r\n");
    printf("     原因は PB8 とその先(トランシーバ/配線)に確定。ハード側\r\n");
  }

  // 後始末: PA11を切り離してPB8をCAN1_RXに戻す
  g.Pin = GPIO_PIN_11;
  g.Mode = GPIO_MODE_ANALOG;
  g.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &g);

  g.Pin = GPIO_PIN_8;
  g.Mode = GPIO_MODE_AF_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  g.Alternate = GPIO_AF9_CAN1;
  HAL_GPIO_Init(GPIOB, &g);

  CAN1->MCR |= CAN_MCR_RESET;
  t = HAL_GetTick();
  while ((CAN1->MCR & CAN_MCR_RESET) && (HAL_GetTick() - t < 100U)) {
  }
  hcan1.State = HAL_CAN_STATE_RESET;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  HAL_CAN_Init(&hcan1);
}

// dribbler.c 側の実体を診断から直接叩く
extern DigitalOut BS_LED;
extern DigitalOut BS_OUT;

// adc_val[idx] を n 回サンプルして min/avg/max を返す
static void AdcSample(uint8_t idx, uint16_t n, uint16_t *mn, uint32_t *avg,
                      uint16_t *mx) {
  uint32_t sum = 0;
  uint16_t lo = 0xFFFF, hi = 0;
  for (uint16_t i = 0; i < n; i++) {
    uint16_t v = adc_val[idx];
    sum += v;
    if (v < lo) lo = v;
    if (v > hi) hi = v;
    HAL_Delay(1);
  }
  *mn = lo;
  *mx = hi;
  *avg = sum / n;
}

// ADCコア自体が健全かを内蔵基準電圧(VREFINT)で確認する。
// VREFINTは約1.21Vの内部基準。これが正しく読めればADCとVDDAは正常で、
// フォトセンサの異常は外部(センサ/コネクタ/R15)に絞れる。
// 工場校正値 VREFINT_CAL は VDDA=3.3V, 30degC で測定されている。
// VREFINT_CAL_ADDR / VREFINT_CAL_VREF は HAL(stm32f4xx_ll_adc.h)の定義を使う。

static void AdcCoreDiag(void) {
  printf("  -- ADC core diag (VREFINT) --\r\n");

  HAL_ADC_Stop_DMA(&hadc1);

  // 一時的に単発1チャンネル変換へ
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    printf("  !! 一時再初期化に失敗\r\n");
  }

  ADC_ChannelConfTypeDef c = {0};
  c.Channel = ADC_CHANNEL_VREFINT;
  c.Rank = 1;
  c.SamplingTime = ADC_SAMPLETIME_480CYCLES;  // VREFINTは長い取込時間が必要
  if (HAL_ADC_ConfigChannel(&hadc1, &c) != HAL_OK) {
    printf("  !! VREFINTチャンネル設定に失敗\r\n");
  }
  HAL_Delay(1);  // 基準電圧の立ち上がり待ち

  uint32_t sum = 0;
  uint16_t n = 0;
  for (uint16_t i = 0; i < 16; i++) {
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
      sum += HAL_ADC_GetValue(&hadc1);
      n++;
    }
    HAL_ADC_Stop(&hadc1);
  }

  if (n == 0) {
    printf("  !! VREFINTを1回も変換できなかった -> ADCコアの異常\r\n");
  } else {
    uint32_t raw = sum / n;
    uint16_t cal = *VREFINT_CAL_ADDR;
    printf("  VREFINT raw=%lu (%u回平均)  VREFINT_CAL=%u\r\n",
           (unsigned long)raw, n, cal);

    // VDDA = 3.3V * CAL / raw
    if (raw > 0 && cal >= 1000 && cal <= 2000) {
      uint32_t vdda_mv = ((uint32_t)VREFINT_CAL_VREF * (uint32_t)cal) / raw;
      g_vdda_mv = vdda_mv;
      printf("  -> VDDA(3.3V_sub) = %lu mV\r\n", (unsigned long)vdda_mv);
      if (vdda_mv < 3000 || vdda_mv > 3600) {
        printf("  !! VDDAが規定外。レギュレータ/電源を疑う\r\n");
      } else {
        printf("  => ADCコアと基準電圧は正常。異常があれば外部回路側\r\n");
      }
    } else {
      printf("  !! CAL値が異常(%u)。校正値の読み出しアドレスを要確認\r\n", cal);
      // 校正値を使わない概算 (VREFINT typ 1.21V)
      if (raw > 0) {
        printf("  参考: VDDA = %lu mV (VREFINT=1.21V典型値から概算)\r\n",
               (unsigned long)((1210U * 4095U) / raw));
      }
    }
  }

  // 元の4ch連続DMA構成へ戻す
  MX_ADC1_Init();
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&adc_val, 4) != HAL_OK) {
    printf("  !! DMA再開に失敗\r\n");
  }
  HAL_Delay(50);
  printf("  DMA再開: adc_val=[%u,%u,%u,%u]\r\n", adc_val[0], adc_val[1],
         adc_val[2], adc_val[3]);
}

#if !SKIP_SLOW_DIAG
// 投光LEDと BS_OUT を振って、BS_IN が応答するかを見る。
// 応答が無ければ光学系かコネクタの問題で、MCUは無関係と分かる。
static void BallSensorDiag(void) {
  printf("  -- ball sensor diag --\r\n");
  printf("  BS_IN は R15(10k)でGNDプルダウン。センサ未接続なら0付近になるはず\r\n");
  printf("  LED OUT |  BS_IN min/avg/max\r\n");

  uint16_t mn, mx;
  uint32_t avg;
  uint32_t dark = 0, bright = 0;

  for (int led = 0; led <= 1; led++) {
    for (int out = 0; out <= 1; out++) {
      DigitalOut_Write(&BS_LED, led);
      DigitalOut_Write(&BS_OUT, out);
      HAL_Delay(100);
      AdcSample(BALL_SENSOR_IDX, 100, &mn, &avg, &mx);
      printf("  %3d %3d | %5u / %5lu / %5u\r\n", led, out, mn,
             (unsigned long)avg, mx);
      if (out == 1 && led == 0) dark = avg;
      if (out == 1 && led == 1) bright = avg;
    }
  }

  DigitalOut_Write(&BS_LED, 1);
  DigitalOut_Write(&BS_OUT, 1);

  int32_t delta = (int32_t)bright - (int32_t)dark;
  printf("  BS_LED ON/OFF の差 = %ld\r\n", (long)delta);
  if (delta > -50 && delta < 50) {
    printf("  !! 投光LEDを振ってもBS_INが動かない\r\n");
    printf("     -> 投光LED/受光素子/J4コネクタ/ケーブルを疑う。MCUは無関係\r\n");
  } else {
    printf("  => 光学系は生きている\r\n");
  }

  // 他チャンネルも併せて確認
  printf("  -- 全チャンネル --\r\n");
  const char *nm[4] = {"ch6  MD_SO (電流)", "BS_IN (フォト)",
                       "ch1  ENC_1", "ch2  ENC_2"};
  for (uint8_t i = 0; i < 4; i++) {
    AdcSample(i, 100, &mn, &avg, &mx);
    printf("  adc[%u] %-20s %5u / %5lu / %5u\r\n", i, nm[i], mn,
           (unsigned long)avg, mx);
  }
}
#endif


// 2枚の基板をログ同士で突き合わせるための要約。
// 全コネクタを外した状態なら、正常な基板は BS_IN / ENC_1 / ENC_2 が
// いずれも 0 付近になるはず (R15/R21/R22 の10kプルダウンのみが効くため)。
static void PrintFingerprint(void) {
  uint16_t mn, mx;
  uint32_t avg;
  uint32_t bs_mv = (g_bs_on_avg * g_vdda_mv) / 4095U;

  printf("\r\n=== BOARD FINGERPRINT ===\r\n");
  printf("  VDDA          = %lu mV\r\n", (unsigned long)g_vdda_mv);

  AdcSample(0, 50, &mn, &avg, &mx);
  printf("  MD_SO  (ch6)  = %lu\r\n", (unsigned long)avg);
  AdcSample(1, 50, &mn, &avg, &mx);
  printf("  BS_IN  (%s) = %lu  (%lu mV)\r\n", BSIN_PIN_NAME, (unsigned long)avg,
         (unsigned long)((avg * g_vdda_mv) / 4095U));
  AdcSample(2, 50, &mn, &avg, &mx);
  printf("  ENC_1  (ch1)  = %lu\r\n", (unsigned long)avg);
  AdcSample(3, 50, &mn, &avg, &mx);
  printf("  ENC_2  (ch2)  = %lu\r\n", (unsigned long)avg);

  if (g_bs_on_avg == 0 && g_bs_off_avg == 0) {
    printf("  BS_LED OFF/ON = (点滅テスト未実行のため無効)\r\n");
  } else {
    printf("  BS_LED OFF/ON = %lu / %lu  (差 %ld)\r\n",
           (unsigned long)g_bs_off_avg, (unsigned long)g_bs_on_avg,
           (long)g_bs_delta);
  }
  printf("  BS_IN pin     = pullup:%d pulldown:%d\r\n", g_bs_pin_up,
         g_bs_pin_dn);
  if (g_bs_on_avg != 0) {
    printf("  (参考) BS_IN  = %lu mV\r\n", (unsigned long)bs_mv);
  }
  printf("  期待値(全コネクタ外し): BS_IN/ENC_1/ENC_2 は 0 付近\r\n");
  printf("=========================\r\n\r\n");
}

#if !SKIP_SLOW_DIAG
// 移設先候補ピンの健全性チェック。
// 未接続ピンなので、健全なら内部プルアップで1・プルダウンで0になる。
// PC3のように保護構造が壊れていると、内部プルに逆らって固定される。
// pad 8/9/10 (PC0/PC1/PC2) と pad 17 (PA3) はいずれも基板上で未接続。
static void CandidatePinProbe(void) {
  struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    const char *name;
    uint8_t pad;
    const char *adc;
  } const cand[] = {
      {GPIOC, GPIO_PIN_0, "PC0", 8, "ADC1_IN10"},
      {GPIOC, GPIO_PIN_1, "PC1", 9, "ADC1_IN11"},
      {GPIOA, GPIO_PIN_3, "PA3", 17, "ADC1_IN3"},
  };

  printf("  -- 予備ピンの健全性 (PC2はBS_INに使用済み) --\r\n");
  printf("  健全な未接続ピン = pullup:1 pulldown:0\r\n");
  printf("  pin pad ADCch      up dn  判定\r\n");

  for (uint8_t i = 0; i < sizeof(cand) / sizeof(cand[0]); i++) {
    GPIO_InitTypeDef g = {0};
    g.Pin = cand[i].pin;
    g.Mode = GPIO_MODE_INPUT;
    g.Speed = GPIO_SPEED_FREQ_LOW;

    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(cand[i].port, &g);
    HAL_Delay(5);
    int up = (HAL_GPIO_ReadPin(cand[i].port, cand[i].pin) == GPIO_PIN_SET) ? 1 : 0;

    g.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(cand[i].port, &g);
    HAL_Delay(5);
    int dn = (HAL_GPIO_ReadPin(cand[i].port, cand[i].pin) == GPIO_PIN_SET) ? 1 : 0;

    // 触らない状態(アナログ)へ戻す
    g.Mode = GPIO_MODE_ANALOG;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(cand[i].port, &g);

    const char *verdict;
    if (up == 1 && dn == 0) {
      verdict = "OK 健全。移設先に使える";
    } else if (up == 1 && dn == 1) {
      verdict = "NG HIGH側に固定。PC3と同じ症状";
    } else if (up == 0 && dn == 0) {
      verdict = "NG LOW側に固定";
    } else {
      verdict = "?? 不定";
    }
    printf("  %s %3u %-10s %d  %d  %s\r\n", cand[i].name, cand[i].pad,
           cand[i].adc, up, dn, verdict);
  }
}
#endif


// BS_IN が本当に外部から駆動されているのか、単に浮いているのかを判定する。
// 一時的にデジタル入力にして内部プルアップ(約40k)/プルダウン(約40k)で引いてみる。
// 外部が低インピーダンスで駆動していれば内部プルでは動かない。
static void BsInPinProbe(void) {
  printf("  -- BS_IN(%s) ピン駆動源の判定 --\r\n", BSIN_PIN_NAME);

  GPIO_InitTypeDef g = {0};
  g.Pin = BSIN_GPIO_PIN;
  g.Mode = GPIO_MODE_INPUT;
  g.Speed = GPIO_SPEED_FREQ_LOW;

  g.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &g);
  HAL_Delay(5);
  int up = (HAL_GPIO_ReadPin(GPIOC, BSIN_GPIO_PIN) == GPIO_PIN_SET) ? 1 : 0;

  g.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &g);
  HAL_Delay(5);
  int dn = (HAL_GPIO_ReadPin(GPIOC, BSIN_GPIO_PIN) == GPIO_PIN_SET) ? 1 : 0;

  // ADC用のアナログモードへ戻す
  g.Mode = GPIO_MODE_ANALOG;
  g.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &g);
  HAL_Delay(5);

  g_bs_pin_up = up;
  g_bs_pin_dn = dn;
  printf("  内部プルアップ=%d 内部プルダウン=%d\r\n", up, dn);
  if (up == 1 && dn == 1) {
    printf("  -> 外部が低インピーダンスでHIGH側に駆動している\r\n");
    printf("     40kの内部プルダウンに勝っている = 数kオーム級の経路が実在\r\n");
  } else if (up == 1 && dn == 0) {
    printf("  -> 浮いている(高インピーダンス)\r\n");
    printf("     R15(10k)が未実装/断線の可能性。ADCの値は当てにならない\r\n");
  } else if (up == 0 && dn == 0) {
    printf("  -> LOW側に引かれている。R15は効いている\r\n");
    printf("     この場合ADCが3500を示すのは矛盾。要再確認\r\n");
  } else {
    printf("  -> 不定\r\n");
  }

  // 参考: R15が生きていれば、内部プルアップ(40k)との分圧で約1/5になるはず
  printf("  参考: R15(10k)健全なら内部プルアップ時は約660mV(=ADC 820)でLOW判定\r\n");
}

// BS_LED を1秒ごとにON/OFFして、BS_IN の時間変化を追う。
// 短い切り替えだと見落とす遅い応答や、手で覆ったときの変化を確認できる。
#define BS_BLINK_CYCLES 5
#define BS_PHASE_MS 1000
#define BS_SAMPLES 10  // 100msごと

#if !SKIP_SLOW_DIAG
static void BallSensorBlinkTest(void) {
  printf("  -- ball sensor blink test --\r\n");
  printf("  BS_LED を1秒ごとにON/OFF、%d往復します (約%d秒)\r\n",
         BS_BLINK_CYCLES, BS_BLINK_CYCLES * 2);
  printf("  1) 投光LEDが実際に点滅しているか目視してください\r\n");
  printf("  2) 途中でセンサを手で覆う/外すを試し、値が動くか見てください\r\n");
  printf("  cyc LED | 100ms毎のBS_IN                               | min/avg/max\r\n");

  DigitalOut_Write(&BS_OUT, 1);

  uint32_t off_sum = 0, on_sum = 0;
  uint16_t off_n = 0, on_n = 0;
  uint16_t all_mn = 0xFFFF, all_mx = 0;

  for (uint8_t c = 1; c <= BS_BLINK_CYCLES; c++) {
    for (int led = 0; led <= 1; led++) {
      DigitalOut_Write(&BS_LED, led);

      uint32_t sum = 0;
      uint16_t mn = 0xFFFF, mx = 0;
      printf("  %3u %s |", c, led ? "ON " : "OFF");
      for (uint8_t i = 0; i < BS_SAMPLES; i++) {
        HAL_Delay(BS_PHASE_MS / BS_SAMPLES);
        uint16_t v = adc_val[BALL_SENSOR_IDX];
        printf(" %4u", v);
        sum += v;
        if (v < mn) mn = v;
        if (v > mx) mx = v;
      }
      uint32_t avg = sum / BS_SAMPLES;
      printf(" | %4u/%4lu/%4u\r\n", mn, (unsigned long)avg, mx);

      if (mn < all_mn) all_mn = mn;
      if (mx > all_mx) all_mx = mx;
      if (led) {
        on_sum += avg;
        on_n++;
      } else {
        off_sum += avg;
        off_n++;
      }
    }
  }

  DigitalOut_Write(&BS_LED, 1);

  uint32_t off_avg = off_n ? (off_sum / off_n) : 0;
  uint32_t on_avg = on_n ? (on_sum / on_n) : 0;
  int32_t delta = (int32_t)on_avg - (int32_t)off_avg;
  g_bs_off_avg = off_avg;
  g_bs_on_avg = on_avg;
  g_bs_delta = delta;

  // 実測VDDAで電圧・電流に換算する
  uint32_t mv = (on_avg * g_vdda_mv) / 4095U;
  uint32_t ua = mv / 10U;  // R15 = 10k なので uA = mV/10

  printf("  === 集計 ===\r\n");
  printf("  LED OFF 平均 = %lu\r\n", (unsigned long)off_avg);
  printf("  LED ON  平均 = %lu\r\n", (unsigned long)on_avg);
  printf("  差          = %ld\r\n", (long)delta);
  printf("  全体レンジ  = %u .. %u\r\n", all_mn, all_mx);
  printf("  BS_IN ≒ %lu mV (VDDA=%lu mV基準) -> R15(10k)に約%lu uA 流入\r\n",
         (unsigned long)mv, (unsigned long)g_vdda_mv, (unsigned long)ua);

  if (delta > -50 && delta < 50) {
    printf("  !! 投光LEDに対してBS_INが応答しない\r\n");
    if (mv > (g_vdda_mv * 9U) / 10U) {
      printf("     BS_INがほぼVDDA。3.3Vへの短絡を疑う\r\n");
    } else if (mv < 100U) {
      printf("     BS_INがほぼ0V。センサ未接続/断線を疑う(R15で引かれている)\r\n");
    } else {
      printf("     中間電位で固定。受光素子が飽和(外光)か、常時導通を疑う\r\n");
      printf("     -> 手で覆っても値が動かないなら光学系ではなく電気的な固定\r\n");
    }
  } else {
    printf("  => 投光LEDに応答している。光学系は生きている\r\n");
  }
}
#endif


// ADCがDMAで実際に値を更新しているかを確認する。
static bool AdcStartChecked(void) {
  HAL_StatusTypeDef st = HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_val, 4);
  if (st != HAL_OK) {
    printf("  !! HAL_ADC_Start_DMA failed: st=%d State=0x%08lX Err=0x%08lX\r\n",
           (int)st, (unsigned long)hadc1.State, (unsigned long)hadc1.ErrorCode);
    return false;
  }

  HAL_Delay(100);  // DMAが数周するのを待つ

  printf("  adc_val = [%u, %u, %u, %u]\r\n", adc_val[0], adc_val[1], adc_val[2],
         adc_val[3]);

  bool updated = false;
  for (uint8_t i = 0; i < 4; i++) {
    if (adc_val[i] != 0) {
      updated = true;
    }
  }
  if (!updated) {
    printf("  !! ADCが全チャンネル0のまま。DMA未動作の可能性\r\n");
    printf("  !! hadc1.State=0x%08lX ErrorCode=0x%08lX\r\n",
           (unsigned long)hadc1.State, (unsigned long)hadc1.ErrorCode);
    return false;
  }

  return true;
}

// 実使用条件でのボールセンサ検証。
// 手で完全に覆う場合と違い、実際のボールは部分的にしか遮らない可能性があるため、
// 10Hzで連続記録し、5秒ごとにその区間の min/max を出す。
// 「5秒ボール無し → 5秒ボール有り」を繰り返せば、両状態の実測レンジが直接読める。
#define BS_MON_SEC 60
#define BS_MON_HZ 10
#define BS_MON_WINDOW_SEC 5

#if !SKIP_SLOW_DIAG
static void BallSensorLiveMonitor(void) {
  const uint16_t per_window = BS_MON_HZ * BS_MON_WINDOW_SEC;
  const uint16_t total = BS_MON_HZ * BS_MON_SEC;

  printf("  -- ボールセンサ実使用検証 (%d秒) --\r\n", BS_MON_SEC);
  printf("  実際のボールを入れる/抜くを %d秒ごとに繰り返してください\r\n",
         BS_MON_WINDOW_SEC);
  printf("  較正で決まった閾値 th=%lu\r\n",
         (unsigned long)Dribbler_GetPhotoThreshold());
  printf("  t[s]  raw   lpf   photo\r\n");

  uint16_t w_mn = 0xFFFF, w_mx = 0;
  uint16_t g_mn = 0xFFFF, g_mx = 0;
  uint16_t win = 1;

  for (uint16_t i = 0; i < total; i++) {
    HAL_Delay(1000 / BS_MON_HZ);

    uint16_t raw = adc_val[BALL_SENSOR_IDX];
    Dribbler_Update(raw, adc_val[MOTOR_CURRENT_IDX]);
    uint16_t lpf = Dribbler_GetFilteredPhoto();
    uint8_t photo = Dribbler_IsBallCapturedByPhoto();

    printf("  %4lu.%lu %5u %5u   %u\r\n", (unsigned long)(i / BS_MON_HZ),
           (unsigned long)(i % BS_MON_HZ), raw, lpf, photo);

    if (raw < w_mn) w_mn = raw;
    if (raw > w_mx) w_mx = raw;
    if (raw < g_mn) g_mn = raw;
    if (raw > g_mx) g_mx = raw;

    if (((i + 1) % per_window) == 0) {
      printf("  --- 区間%u (%d秒間): min=%u max=%u 幅=%u ---\r\n", win,
             BS_MON_WINDOW_SEC, w_mn, w_mx, (uint16_t)(w_mx - w_mn));
      w_mn = 0xFFFF;
      w_mx = 0;
      win++;
    }
  }

  printf("  === 全体 min=%u max=%u ===\r\n", g_mn, g_mx);
  printf("  現在の閾値 th=%lu (baseline比例)\r\n",
         (unsigned long)Dribbler_GetPhotoThreshold());
  printf("  参考: 遮蔽時の最大値と非遮蔽時の最小値の間に閾値を置くこと\r\n");
  printf("  参考: 全体maxの50%% = %u\r\n", (uint16_t)(g_mx / 2));
}
#endif


void Setup() {
  printf("\r\n==== Dribbler Setup Start ====\r\n");
#ifdef BSIN_PIN_PC2
  printf("*** 改造機ビルド: BS_IN = %s ***\r\n", BSIN_PIN_NAME);
  printf("*** PC3は損傷のため切り離し済み。通常機では動作しません ***\r\n");
#else
  printf("*** 通常機ビルド: BS_IN = %s ***\r\n", BSIN_PIN_NAME);
#endif

  DIAG_STEP(1, "PWM(LED) init");
  PwmOut_Init(&LED1, &htim8, TIM_CHANNEL_4);
  PwmOut_Init(&LED2, &htim8, TIM_CHANNEL_3);
  PwmOut_Init(&LED3, &htim8, TIM_CHANNEL_2);
  PwmOut_Init(&LED4, &htim8, TIM_CHANNEL_1);

#ifdef BSIN_PIN_PC2
  // 【改造機専用】PC3 は保護構造が損傷しており、基板側では切り離し済み。
  // デジタル入力バッファを働かせないようアナログに固定し、以後一切触らない。
  // 出力に設定すると損傷部に最大17.6mA流れて悪化するため絶対に禁止。
  {
    GPIO_InitTypeDef g = {0};
    g.Pin = GPIO_PIN_3;
    g.Mode = GPIO_MODE_ANALOG;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &g);
  }
#endif  // BSIN_PIN_PC2

  DIAG_STEP(2, "DigitalOut init");
  DigitalOut_Init(&CAN_LED, CAN_LED_GPIO_Port, CAN_LED_Pin);
  DigitalOut_Init(&MD_SLEEP, MD_nSLEEP_GPIO_Port, MD_nSLEEP_Pin);

  DIAG_STEP(3, "MD nSLEEP = 1 (wake motor driver)");
  DigitalOut_Write(&MD_SLEEP, 1);

  PwmOut_Write(&LED1, 1);  // デジタル入出力の初期化確認

  DIAG_STEP(4, "CAN init");
  can_ok = CanInitChecked();
  printf("  CAN init %s\r\n", can_ok ? "OK" : "FAILED (続行する)");
  if (!can_ok) {
    CanPinDiag();
    CanRawDiag();
    CanAltPinDiag();
  }
  HAL_CAN_DeactivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  PwmOut_Write(&LED2, 1);  // CANの初期化確認

  DIAG_STEP(5, "ADC DMA start");
  adc_ok = AdcStartChecked();
  printf("  ADC init %s\r\n", adc_ok ? "OK" : "FAILED (続行する)");

  PwmOut_Write(&LED3, 1);  // ADCの初期化確認

  DIAG_STEP(6, "Dribbler_Init (Motor PWM on htim3)");
  Dribbler_Init();

  DIAG_STEP(61, "ADC/ボールセンサ切り分け");
  AdcCoreDiag();
#if !SKIP_SLOW_DIAG
  BallSensorDiag();
  BallSensorBlinkTest();
  CandidatePinProbe();
#endif
  BsInPinProbe();
  PrintFingerprint();

#if !SKIP_SLOW_DIAG
  DIAG_STEP(7, "Motor spin sweep (CAN非依存)");
  // レベルを上げながら電流を測る。デューティに対して電流が動かなければ
  // 「モータが回っていない」か「電流センスが死んでいる」のどちらか。
  printf("  level | current min/avg/max\r\n");
  {
    const uint8_t levels[] = {0, 2, 4, 6, 8, 10};
    for (uint8_t i = 0; i < sizeof(levels); i++) {
      Motor_Drive(levels[i]);
      HAL_Delay(400);  // 立ち上がりを待つ

      uint32_t sum = 0;
      uint16_t mn = 0xFFFF, mx = 0;
      for (uint16_t n = 0; n < 100; n++) {
        uint16_t v = adc_val[MOTOR_CURRENT_IDX];
        sum += v;
        if (v < mn) mn = v;
        if (v > mx) mx = v;
        HAL_Delay(1);
      }
      printf("  %5u | %u / %lu / %u\r\n", levels[i], mn,
             (unsigned long)(sum / 100), mx);
    }
    Motor_Free();
    HAL_Delay(300);
    printf("  free  | %u\r\n", adc_val[MOTOR_CURRENT_IDX]);
  }

#endif  // !SKIP_SLOW_DIAG

  DIAG_STEP(8, "Motor base current calibration");
  uint32_t t0 = HAL_GetTick();
  while (!Motor_SetBaseCurrent(adc_val[MOTOR_CURRENT_IDX])) {
    if (HAL_GetTick() - t0 > CALIB_TIMEOUT_MS) {
      printf("  !! base current calibration TIMEOUT (%lums)\r\n",
             (unsigned long)(HAL_GetTick() - t0));
      Motor_Free();
      break;
    }
    HAL_Delay(1);
  }
  printf("  base current calibration done (%lums)\r\n",
         (unsigned long)(HAL_GetTick() - t0));

  DIAG_STEP(9, "Photo threshold calibration");
  t0 = HAL_GetTick();
  while (!Dribbler_SetPhotoThreshold(adc_val[BALL_SENSOR_IDX])) {
    if (HAL_GetTick() - t0 > CALIB_TIMEOUT_MS) {
      printf("  !! photo threshold calibration TIMEOUT (%lums)\r\n",
             (unsigned long)(HAL_GetTick() - t0));
      break;
    }
    HAL_Delay(1);
  }
  printf("  photo threshold calibration done (%lums)\r\n",
         (unsigned long)(HAL_GetTick() - t0));

#if !SKIP_SLOW_DIAG
  DIAG_STEP(91, "ボールセンサ実使用検証");
  BallSensorLiveMonitor();
#endif

  PwmOut_Write(&LED4, 1);  // モーターの初期化確認

  DIAG_STEP(10, "Timer init");
  Timer_Init(&can_send_interval_timer);

  PwmOut_Write(&LED1, 0);
  PwmOut_Write(&LED2, 0);
  PwmOut_Write(&LED3, 0);
  PwmOut_Write(&LED4, 0);

  DIAG_STEP(11, "Setup complete");
  printf("---- summary ----\r\n");
  printf("  CAN : %s\r\n", can_ok ? "OK" : "NG");
  printf("  ADC : %s\r\n", adc_ok ? "OK" : "NG");
  printf("=================\r\n");

#if BALL_FOLLOW_TEST
  printf("  [試験モード] CAN受信によるモータ駆動は無効です\r\n");
  (void)can_ok;
#else
  if (can_ok) {
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  } else {
    printf("  CANがNGのため受信割り込みは有効化しない\r\n");
    printf("  -> CAN経由の Motor_Drive() は来ない。モータは回らない\r\n");
  }
#endif
}

#if BALL_FOLLOW_TEST
// ボール追従試験。
// フォトセンサがボールを検知したらドリブラを TEST_SPEED_PERCENT で回し、
// 離したら停止する。CANは状態送信のみで、駆動指令は受け付けない。
static void BallFollowTest(void) {
  const uint8_t level =
      (uint8_t)((TEST_SPEED_PERCENT * MAX_SPEED_LEVEL + 50) / 100);
  const uint32_t th_on = Dribbler_GetPhotoThreshold();
  // 離脱判定は25%高い側で行う。閾値付近でのバタつきを防ぐ
  const uint32_t th_off = th_on + th_on / 4U;

  printf("\r\n======== ボール追従試験 ========\r\n");
  printf("  検知 -> レベル%u/%d (%d%%) で回転\r\n", level, MAX_SPEED_LEVEL,
         TEST_SPEED_PERCENT);
  printf("  離脱 -> 停止 (フリー)\r\n");
  printf("  ON閾値 =%lu (これ未満で検知)\r\n", (unsigned long)th_on);
  printf("  OFF閾値=%lu (これ超で離脱)\r\n", (unsigned long)th_off);
  printf("  連続回転の上限 %dms (ベンチ安全対策)\r\n", TEST_MAX_RUN_MS);
  if (th_on == 0) {
    printf("  !! 閾値が0。センサ異常のため回転させません\r\n");
  }
  printf("================================\r\n");

  bool running = false;
  bool timeout_latched = false;
  uint32_t started_at = 0;
  uint32_t last_log = HAL_GetTick();
  uint32_t detect_count = 0;

  while (1) {
    Dribbler_Update(adc_val[BALL_SENSOR_IDX], adc_val[MOTOR_CURRENT_IDX]);
    uint16_t lpf = Dribbler_GetFilteredPhoto();
    uint32_t now = HAL_GetTick();

    if (th_on > 0) {
      if (!running && !timeout_latched && lpf < th_on) {
        running = true;
        started_at = now;
        detect_count++;
        Motor_Drive(level);
        printf("[BALL] 検知 -> 回転開始  photo=%u (#%lu)\r\n", lpf,
               (unsigned long)detect_count);
      } else if (running && lpf > th_off) {
        running = false;
        Motor_Free();
        printf("[BALL] 離脱 -> 停止      photo=%u  保持%lums\r\n", lpf,
               (unsigned long)(now - started_at));
      } else if (running && (now - started_at) > TEST_MAX_RUN_MS) {
        running = false;
        timeout_latched = true;
        Motor_Free();
        printf("[BALL] !! 連続回転が上限に達したため停止\r\n");
        printf("       センサが検知状態で張り付いている可能性があります\r\n");
      }
    }

    // 検知が解除されたらタイムアウトのラッチも解除する
    if (timeout_latched && lpf > th_off) {
      timeout_latched = false;
      printf("[BALL] 検知解除。再び動作可能です\r\n");
    }

    // 状態ログ
    if (now - last_log >= 1000) {
      last_log = now;
      printf("[RUN] photo raw=%u lpf=%u th=%lu | motor=%s | cur=%u\r\n",
             adc_val[BALL_SENSOR_IDX], lpf, (unsigned long)th_on,
             running ? "回転中" : "停止", adc_val[MOTOR_CURRENT_IDX]);
    }

    // CANには状態のみ送る(駆動指令は受けない)
    if (Timer_ReadMs(&can_send_interval_timer) >= CAN_SEND_INTERVAL_MS) {
      DigitalOut_Write(&CAN_LED, 1);
      CanData data = {
          .stdId = CAN_SEND_ID,
          .data = {(uint8_t)(lpf < th_on), 0, (uint8_t)running},
      };
      Can_Send(&can, &data);
      Timer_Reset(&can_send_interval_timer);
    } else {
      DigitalOut_Write(&CAN_LED, 0);
    }

    PwmOut_Write(&LED1, (lpf < th_on) ? 1.0f : 0.0f);
    PwmOut_Write(&LED3, running ? 1.0f : 0.0f);
  }
}
#endif  // BALL_FOLLOW_TEST

void MainApp() {
#if BALL_FOLLOW_TEST
  BallFollowTest();
  return;
#endif

  // 前回送信した状態（変化検出用）。初回は必ず送信されるように-1で初期化
  static int16_t prev_by_photo = -1;
  static int16_t prev_by_current = -1;
  static int16_t prev_captured = -1;

  // 生存確認ログ。1秒ごとに現在値を吐く
  uint32_t last_log = HAL_GetTick();
  // CANがNGの間は5秒ごとに再診断する。起動ログを取り逃しても状況が読めるうえ、
  // 配線を挿し直した瞬間にPB8の変化がその場で見える。
  uint32_t last_can_retry = HAL_GetTick();

  while (1) {
    Dribbler_Update(adc_val[BALL_SENSOR_IDX], adc_val[MOTOR_CURRENT_IDX]);

    uint8_t by_photo = Dribbler_IsBallCapturedByPhoto();
    uint8_t by_current = Dribbler_IsBallCapturedByCurrent();
    uint8_t captured = Dribbler_IsBallCaptured();

    if (HAL_GetTick() - last_log >= 1000) {
      last_log = HAL_GetTick();
      printf("[RUN] adc[cur]=%u adc[photo]=%u photo=%u cur=%u cap=%u | "
             "CAN=%s rx=%lu match=%lu lvl=%u boff=%lu\r\n",
             adc_val[MOTOR_CURRENT_IDX], adc_val[BALL_SENSOR_IDX], by_photo,
             by_current, captured, can_ok ? "OK" : "NG",
             (unsigned long)can_rx_count, (unsigned long)can_rx_match_count,
             can_last_level, (unsigned long)can_busoff_count);
      CanDumpEsr("      ");

      // バスオフは自力で復帰させる。放置すると送受信とも永久に停止する
      if (can_ok && (CAN1->ESR & CAN_ESR_BOFF)) {
        can_busoff_count++;
        printf("      !! Bus-Off 検出 (%lu回目) -> 復帰を試行\r\n",
               (unsigned long)can_busoff_count);
        CanBusOffRecover();
      }
    }

    if (!can_ok && HAL_GetTick() - last_can_retry >= 5000) {
      last_can_retry = HAL_GetTick();
      printf("---- CAN re-diagnose ----\r\n");
      if (can_core_tested) {
        printf("  boot raw test: CANコアは%s\r\n",
               can_core_ok ? "正常(原因は基板外部)" : "異常(MCU側も疑い)");
      }
      CanPinDiag();  // 今この瞬間のPB8を測る。挿抜しながら見られる
      can_ok = CanInitChecked();
      printf("  retry: %s\r\n", can_ok ? "CAN復帰" : "まだNG");
      if (can_ok) {
        HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
      }
      printf("-------------------------\r\n");
      last_log = HAL_GetTick();
    }

    // いずれかの値が変化した瞬間、または10ms周期でCAN送信
    bool changed = (by_photo != prev_by_photo) ||
                   (by_current != prev_by_current) ||
                   (captured != prev_captured);
    bool interval_elapsed =
        Timer_ReadMs(&can_send_interval_timer) >= CAN_SEND_INTERVAL_MS;

    if (changed || interval_elapsed) {
      DigitalOut_Write(&CAN_LED, 1);
      CanData data = {
          .stdId = CAN_SEND_ID,
          .data =
              {
                  by_photo,
                  by_current,
                  captured,
              },
      };
      Can_Send(&can, &data);
      Timer_Reset(&can_send_interval_timer);

      prev_by_photo = by_photo;
      prev_by_current = by_current;
      prev_captured = captured;
    } else {
      DigitalOut_Write(&CAN_LED, 0);
    }

    PwmOut_Write(&LED1, Dribbler_IsBallCapturedByPhoto());
    PwmOut_Write(&LED2, Dribbler_IsBallCapturedByCurrent());
    PwmOut_Write(&LED3, Dribbler_IsBallCaptured());
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
  if (can.hcan == hcan) {
    Can_Recv(&can, &canRecvData);
    can_rx_count++;
    if (canRecvData.stdId == CAN_RECV_ID) {
      can_rx_match_count++;
      // 受信した0~255の速度データをレベルに変換
      uint8_t level = (canRecvData.data[0] * MAX_SPEED_LEVEL) / 255;
      can_last_level = level;
      Motor_Drive(level);
    }
  }
}
