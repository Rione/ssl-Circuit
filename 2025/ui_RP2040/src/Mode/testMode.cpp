#include "testMode.hpp"

TestMode::TestMode(char letter, const char name[], UiKit *uiPtr,
                   MediaExecutor *mediaPtr)
    : Mode(letter, name, uiPtr, mediaPtr) {

  // x, y, w, h, text, bg, fg
  int startX = 80;
  int startY = 48;
  int gapX = 8;
  int gapY = 8;
  int bw = 110;
  int bh = 56;
  uint16_t btnBg =
      ui->display.sprite->color565(248, 249, 250); // ui->colSidebar equivalent

  btnDribbler = new TEXT_BUTTON(&ui->display, &ui->touch, startX, startY, bw,
                                bh, "Dribbler", btnBg, ui->colFg);
  btnKick = new TEXT_BUTTON(&ui->display, &ui->touch, startX + bw + gapX,
                            startY, bw, bh, "Kick", btnBg, ui->colFg);
  btnChipKick =
      new TEXT_BUTTON(&ui->display, &ui->touch, startX, startY + bh + gapY, bw,
                      bh, "Chip Kick", btnBg, ui->colFg);
  btnMotorTest = new TEXT_BUTTON(&ui->display, &ui->touch, startX + bw + gapX,
                                 startY + bh + gapY, bw, bh, "Motor Test",
                                 btnBg, ui->colFg);
  btnCharge = new TEXT_BUTTON(&ui->display, &ui->touch, startX,
                              startY + 2 * (bh + gapY), bw, bh, "Charge", btnBg,
                              ui->colFg);
  btnDischarge = new TEXT_BUTTON(&ui->display, &ui->touch, startX + bw + gapX,
                                 startY + 2 * (bh + gapY), bw, bh, "Discharge",
                                 btnBg, ui->colFg);
}

static bool isDribblerOn = false;
static bool isMotorTestOn = false;
static uint32_t motorTestStartTime = 0;
static uint32_t kickTime = 0;
static uint32_t chipKickTime = 0;

void TestMode::displaySet() {
  static bool prevChargeState = false;
  static uint8_t prevChargeVal = 255;

  if (ui->changeFlag_overMode) {
    drawUI();
  }

  uint32_t currentMillis = millis();

  // Motor test automatically stops after 3 seconds
  if (isMotorTestOn && (currentMillis - motorTestStartTime > 3000)) {
    isMotorTestOn = false;
  }

  uint16_t defaultBtnBg = ui->display.sprite->color565(248, 249, 250);

  bool needsRedrawDribbler = false;
  bool needsRedrawMotor = false;
  bool needsRedrawKick = false;
  bool needsRedrawChip = false;

  uint16_t expectedDribblerBg = isDribblerOn ? ui->colPrimary : defaultBtnBg;
  if (btnDribbler->bg != expectedDribblerBg) {
    btnDribbler->bg = expectedDribblerBg;
    btnDribbler->fg = isDribblerOn ? ui->colBg : ui->colFg;
    needsRedrawDribbler = true;
  }

  uint16_t expectedMotorBg = isMotorTestOn ? ui->colPrimary : defaultBtnBg;
  if (btnMotorTest->bg != expectedMotorBg) {
    btnMotorTest->bg = expectedMotorBg;
    btnMotorTest->fg = isMotorTestOn ? ui->colBg : ui->colFg;
    needsRedrawMotor = true;
  }

  bool isKickOn = (currentMillis - kickTime <= 750) && kickTime > 0;
  uint16_t expectedKickBg = isKickOn ? ui->colPrimary : defaultBtnBg;
  if (btnKick->bg != expectedKickBg) {
    btnKick->bg = expectedKickBg;
    btnKick->fg = isKickOn ? ui->colBg : ui->colFg;
    needsRedrawKick = true;
  }

  bool isChipOn = (currentMillis - chipKickTime <= 750) && chipKickTime > 0;
  uint16_t expectedChipBg = isChipOn ? ui->colPrimary : defaultBtnBg;
  if (btnChipKick->bg != expectedChipBg) {
    btnChipKick->bg = expectedChipBg;
    btnChipKick->fg = isChipOn ? ui->colBg : ui->colFg;
    needsRedrawChip = true;
  }

  if (needsRedrawDribbler && !ui->changeFlag_overMode) btnDribbler->draw(false);
  if (needsRedrawMotor && !ui->changeFlag_overMode) btnMotorTest->draw(false);
  if (needsRedrawKick && !ui->changeFlag_overMode) btnKick->draw(false);
  if (needsRedrawChip && !ui->changeFlag_overMode) btnChipKick->draw(false);

  // いずれかのボタンが再描画された場合は画面を更新する
  if ((needsRedrawDribbler || needsRedrawMotor || needsRedrawKick || needsRedrawChip) && !ui->changeFlag_overMode) {
    ui->display.publish(0, 0);
  }

  bool chargeStateChanged = (ui->info.capaData.chargeState != prevChargeState) || ui->changeFlag_overMode;
  bool chargeValChanged = (ui->info.capaData.chargeVal != prevChargeVal) || ui->changeFlag_overMode;

  if (chargeStateChanged) {
    prevChargeState = ui->info.capaData.chargeState;
    uint16_t defaultBtnBg = ui->display.sprite->color565(248, 249, 250);

    if (prevChargeState) {
      btnCharge->bg = ui->colError; // 充電中は赤色
      btnCharge->fg = ui->colBg;    // 白文字
      btnDischarge->bg = defaultBtnBg;
      btnDischarge->fg = ui->colFg;
    } else {
      btnCharge->bg = defaultBtnBg;
      btnCharge->fg = ui->colFg;
      btnDischarge->bg = ui->colPrimary; // 放電中（非充電中）は青色
      btnDischarge->fg = ui->colBg;      // 白文字
    }

    btnCharge->draw(false);
    btnDischarge->draw(false);
  }

  if (chargeValChanged || chargeStateChanged) {
    prevChargeVal = ui->info.capaData.chargeVal;
    
    // ボタンのテキストを動的に更新する
    static char chargeBtnText[16];
    snprintf(chargeBtnText, sizeof(chargeBtnText), "Chg %d%%", prevChargeVal);
    btnCharge->text = chargeBtnText;

    uint16_t color = ui->colSuccess;
    if (prevChargeVal < 80)
      color = ui->colWarning;
    if (prevChargeVal < 30)
      color = ui->colError;

    // もしボタンの背景が赤(colError)の場合、%の色が見えづらくなる可能性があるので白にする
    if (prevChargeState) {
        color = ui->colBg; // 白
    }
    
    btnCharge->fg = color;
    btnCharge->draw(false);
  }

  if (chargeStateChanged || chargeValChanged) {
      ui->display.publish(0, 0);
  }
}

void TestMode::drawUI() {
  ui->display.sprite->fillRect(72, 36, 248, 204, ui->colBg);

  btnDribbler->draw(false);
  btnKick->draw(false);
  btnChipKick->draw(false);
  btnMotorTest->draw(false);
  btnCharge->draw(false);
  btnDischarge->draw(false);

  ui->display.publish(0, 0);
}

void TestMode::determine() {
  // 状態の更新(はなされた時に戻す処理)
  btnDribbler->updateState();
  btnKick->updateState();
  btnChipKick->updateState();
  btnMotorTest->updateState();
  btnCharge->updateState();
  btnDischarge->updateState();

  if (btnDribbler->determine()) {
    isDribblerOn = !isDribblerOn;
    ui->info.testCommand = 3; // CMD_DRIBBLER
    ui->stmSendSerial(&ui->info);
    ui->info.testCommand = 0;
    media->setBuzzerType(playType::NOTIFY);
  }
  if (btnKick->determine()) {
    if (!ui->info.capaData.chargeState) {
      media->setBuzzerType(playType::ERROR);
    } else {
      kickTime = millis();
      ui->info.testCommand = 1; // CMD_KICK
      ui->stmSendSerial(&ui->info);
      ui->info.testCommand = 0;
      media->setBuzzerType(playType::NOTIFY);
    }
  }
  if (btnChipKick->determine()) {
    if (!ui->info.capaData.chargeState) {
      media->setBuzzerType(playType::ERROR);
    } else {
      chipKickTime = millis();
      ui->info.testCommand = 2; // CMD_CHIP_KICK
      ui->stmSendSerial(&ui->info);
      ui->info.testCommand = 0;
      media->setBuzzerType(playType::NOTIFY);
    }
  }
  if (btnMotorTest->determine()) {
    isMotorTestOn = true;
    motorTestStartTime = millis();
    ui->info.testCommand = 4; // CMD_MOTOR_TEST
    ui->stmSendSerial(&ui->info);
    ui->info.testCommand = 0;
    media->setBuzzerType(playType::NOTIFY);
  }
  if (btnCharge->determine()) {
    ui->info.modeStatus.charge_state = 1;
    ui->stmSendSerial(&ui->info);
    ui->info.modeStatus.charge_state = 0;
    media->setBuzzerType(playType::NOTIFY);
  }
  if (btnDischarge->determine()) {
    isDribblerOn = false;
    isMotorTestOn = false;
    ui->info.testCommand = 5; // CMD_DISCHARGE
    ui->stmSendSerial(&ui->info);
    ui->info.testCommand = 0;
    media->setBuzzerType(playType::NOTIFY);
  }
}
