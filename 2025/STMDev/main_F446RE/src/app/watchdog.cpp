#include "watchdog.h"

#include "main.h"

static const uint32_t PERSIST_MAGIC = 0x57444F47;  // "WDOG"

// .noinitは電源を切らないリセットでは消えない。電源投入直後は不定値なのでmagicで有効性を判定する
static uint32_t persistMagic __attribute__((section(".noinit")));
static uint32_t persistResetCount __attribute__((section(".noinit")));

static bool resetByWatchdog = false;

void watchdogCheckResetCause() {
      resetByWatchdog = (RCC->CSR & RCC_CSR_IWDGRSTF) != 0;
      RCC->CSR |= RCC_CSR_RMVF;  // 次回の判定のためにリセット要因フラグを消す

      if (persistMagic != PERSIST_MAGIC) {
            persistMagic = PERSIST_MAGIC;
            persistResetCount = 0;
      }
      if (resetByWatchdog) {
            persistResetCount++;
      }
}

bool watchdogCausedReset() {
      return resetByWatchdog;
}

uint32_t watchdogResetCount() {
      return persistResetCount;
}

void watchdogStart(uint32_t timeoutMs) {
      // デバッガでCPUを止めている間はウォッチドッグも止める(止めないとブレークポイントでリセットされる)
      DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;

      // LSI(約32kHz)を1/32して約1msで1カウント
      IWDG->KR = 0xCCCC;  // 開始(LSIも自動で起動する)
      IWDG->KR = 0x5555;  // PR, RLRへの書き込みを許可
      IWDG->PR = IWDG_PR_PR_1 | IWDG_PR_PR_0;  // 1/32
      IWDG->RLR = (timeoutMs > 0xFFF) ? 0xFFF : timeoutMs;
      while (IWDG->SR != 0) {
      }
      IWDG->KR = 0xAAAA;
}

void watchdogRefresh() {
      IWDG->KR = 0xAAAA;
}
