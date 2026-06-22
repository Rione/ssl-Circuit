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

void TestMode::displaySet() {
  if (ui->changeFlag_overMode) {
    drawUI();
  }

  static uint8_t prevChargeVal = 255;
  uint8_t currentChargeVal = ui->info.capaData.chargeVal;

  uint16_t color = ui->colSuccess;
  if (currentChargeVal < 80) color = ui->colWarning;
  if (currentChargeVal < 30) color = ui->colError;

  uint16_t btnBg = ui->display.sprite->color565(248, 249, 250);
  ui->display.sprite->setTextColor(color, btnBg);
  ui->display.sprite->setTextDatum(MR_DATUM);
  ui->display.sprite->setTextFont(4);
  ui->display.sprite->setTextPadding(ui->display.sprite->textWidth("100%"));

  char chargeBuf[8];
  sprintf(chargeBuf, "%3d%%", currentChargeVal);
  String chargeStr = String(chargeBuf);

  ui->display.sprite->drawString(chargeStr, btnCharge->x + btnCharge->w - 12,
                                 btnCharge->y + btnCharge->h / 2);
  ui->display.sprite->setTextPadding(0);

  if (currentChargeVal != prevChargeVal || ui->changeFlag_overMode) {
    prevChargeVal = currentChargeVal;
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
  if (btnDribbler->determine()) {
    ui->info.testCommand = 3;
    ui->stmSendSerial(&ui->info);
    ui->info.testCommand = 0;
    media->setBuzzerType(playType::NOTIFY);
  }

  if (btnKick->determine()) {
    ui->info.modeStatus.kick = 1;
    ui->stmSendSerial(&ui->info);
    ui->info.modeStatus.kick = 0;
    media->setBuzzerType(playType::NOTIFY);
  }

  if (btnChipKick->determine()) {
    ui->info.testCommand = 2;
    ui->stmSendSerial(&ui->info);
    ui->info.testCommand = 0;
    media->setBuzzerType(playType::NOTIFY);
  }

  if (btnMotorTest->determine()) {
    ui->info.testCommand = 4;
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
    ui->info.testCommand = 5;
    ui->stmSendSerial(&ui->info);
    ui->info.testCommand = 0;
    media->setBuzzerType(playType::NOTIFY);
  }
}
