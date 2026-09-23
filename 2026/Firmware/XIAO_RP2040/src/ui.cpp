#include "ui.h"

#include "config.h"

namespace {

constexpr uint16_t Rgb(uint8_t r, uint8_t g, uint8_t b) {
  return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

// 配色 (ダークテーマ)
constexpr uint16_t kBg = Rgb(12, 14, 18);
constexpr uint16_t kPanel = Rgb(30, 34, 42);
constexpr uint16_t kPanelHi = Rgb(52, 58, 70);
constexpr uint16_t kText = Rgb(235, 238, 242);
constexpr uint16_t kMuted = Rgb(130, 138, 150);
constexpr uint16_t kAccent = Rgb(0, 190, 230);
constexpr uint16_t kOk = Rgb(60, 200, 100);
constexpr uint16_t kWarn = Rgb(250, 190, 40);
constexpr uint16_t kDanger = Rgb(235, 70, 70);
constexpr uint16_t kBall = Rgb(255, 140, 0);

// レイアウト
constexpr int16_t kW = 320;
constexpr int16_t kStatusH = 26;
constexpr int16_t kTabH = 34;
constexpr int16_t kTabY = 240 - kTabH;
constexpr int16_t kTop = kStatusH + 4;  // コンテンツ領域の上端
constexpr int16_t kTabW = kW / 5;

constexpr const char *kTabLabels[] = {"HOME", "DRIB", "KICK", "WHEEL", "IMU"};

// ステータスバーのロックボタン
constexpr int16_t kLockX = 2, kLockY = 2, kLockW = 66, kLockH = 22;

// DRIB ページ
constexpr int16_t kDribMinusX = 8, kDribPlusX = 132, kDribAdjY = 62, kDribAdjW = 56, kDribAdjH = 56;
constexpr int16_t kDribOnX = 200, kDribOnY = 56, kDribOnW = 112, kDribOnH = 68;

// KICK ページ
constexpr int16_t kChgX = 4, kChgY = 70, kChgW = 104, kChgH = 46;
constexpr int16_t kKpMinusX = 120, kKpPlusX = 262, kKpY = 70, kKpW = 54, kKpH = 46;
constexpr int16_t kKickY = 124, kKickH = 64;
constexpr int16_t kStraightX = 4, kChipX = 162, kKickW = 154;

// WHEEL ページ
constexpr int16_t kSelY = 32, kSelW = 38, kSelH = 34, kSelGap = 3;
constexpr int16_t kTgtY = 72, kTgtW = 46, kTgtH = 40;
constexpr int16_t kRunX = 212, kRunY = 32, kRunW = 104, kRunH = 80;

template <typename T>
T Clamp(T v, T lo, T hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

}  // namespace

void Ui::begin() {
  spr_.setColorDepth(16);
  if (spr_.createSprite(kW, 240) == nullptr) {
    // RAM 不足時は 8bit (RGB332) で妥協する
    spr_.setColorDepth(8);
    spr_.createSprite(kW, 240);
  }
  cmd_.kick_pct = KICK_DEFAULT_PCT;
  cmd_.dribble_pct = 50;
  cmd_.wheel_target_radps = WHEEL_TEST_DEFAULT_RADPS;
}

// ============================================================================
// 入力
// ============================================================================

bool Ui::tapped(int16_t x, int16_t y, int16_t w, int16_t h) const {
  return touch_.justPressed() && touch_.in(x, y, w, h);
}

void Ui::click(bool ok) {
#if BUZZER_ENABLE
  tone(PIN_BUZZER, ok ? BUZZER_CLICK_HZ : BUZZER_CLICK_HZ / 3, ok ? BUZZER_CLICK_MS : 120);
#else
  (void)ok;
#endif
}

bool Ui::canArm() const {
  return link_.connected() && !link_.telemetry().has(UI_TLM_F1_ROCK_LINK);
}

void Ui::lock() {
  armed_ = false;
  cmd_.dribble_on = false;
  cmd_.charge = false;
  cmd_.wheel_run = false;
}

void Ui::setPage(Page p) {
  if (p == page_) return;
  // ページを離れたら、そのページで動かしていたアクチュエータは止める
  if (page_ == kDribble) cmd_.dribble_on = false;
  if (page_ == kWheel) cmd_.wheel_run = false;
  page_ = p;
}

void Ui::updateSafety() {
  if (!armed_) return;
  bool idle_timeout = millis() - touch_.lastActivityMs() > TEST_AUTO_LOCK_MS;
  if (!canArm() || idle_timeout) lock();
}

void Ui::update() {
  touch_.update();

  handleStatusBar();
  handleTabs();
  switch (page_) {
    case kDribble: handleDribble(); break;
    case kKick: handleKick(); break;
    case kWheel: handleWheel(); break;
    default: break;
  }
  updateSafety();

  cmd_.test_enable = armed_;

  // キック ACK 検出 (MainBoard が今回の seq を実行したら表示を光らせる)
  uint8_t ack = link_.telemetry().kick_ack;
  if (ack != last_kick_ack_) {
    last_kick_ack_ = ack;
    if (ack == cmd_.kick_seq && cmd_.kick_type != UI_KICK_NONE) kick_flash_ms_ = millis();
  }
}

void Ui::handleStatusBar() {
  if (!tapped(kLockX, kLockY, kLockW, kLockH)) return;
  if (armed_) {
    lock();
    click();
  } else if (canArm()) {
    armed_ = true;
    click();
  } else {
    click(false);
  }
}

void Ui::handleTabs() {
  if (!touch_.justPressed() || touch_.y() < kTabY) return;
  Page p = (Page)Clamp<int>(touch_.x() / kTabW, 0, kPageCount - 1);
  if (p != page_) {
    setPage(p);
    click();
  }
}

void Ui::handleDribble() {
  if (tapped(kDribMinusX, kDribAdjY, kDribAdjW, kDribAdjH)) {
    cmd_.dribble_pct = (uint8_t)Clamp(cmd_.dribble_pct - DRIBBLE_STEP_PCT, 0, 100);
    click();
  }
  if (tapped(kDribPlusX, kDribAdjY, kDribAdjW, kDribAdjH)) {
    cmd_.dribble_pct = (uint8_t)Clamp(cmd_.dribble_pct + DRIBBLE_STEP_PCT, 0, 100);
    click();
  }
  if (tapped(kDribOnX, kDribOnY, kDribOnW, kDribOnH)) {
    if (armed_) {
      cmd_.dribble_on = !cmd_.dribble_on;
      click();
    } else {
      click(false);
    }
  }
}

void Ui::handleKick() {
  if (tapped(kChgX, kChgY, kChgW, kChgH)) {
    if (armed_) {
      cmd_.charge = !cmd_.charge;
      click();
    } else {
      click(false);
    }
  }
  if (tapped(kKpMinusX, kKpY, kKpW, kKpH)) {
    cmd_.kick_pct = (uint8_t)Clamp(cmd_.kick_pct - KICK_STEP_PCT, KICK_STEP_PCT, 100);
    click();
  }
  if (tapped(kKpPlusX, kKpY, kKpW, kKpH)) {
    cmd_.kick_pct = (uint8_t)Clamp(cmd_.kick_pct + KICK_STEP_PCT, KICK_STEP_PCT, 100);
    click();
  }

  uint8_t type = UI_KICK_NONE;
  if (tapped(kStraightX, kKickY, kKickW, kKickH)) type = UI_KICK_STRAIGHT;
  if (tapped(kChipX, kKickY, kKickW, kKickH)) type = UI_KICK_CHIP;
  if (type == UI_KICK_NONE) return;

  bool cooled = millis() - last_kick_ms_ >= KICK_MIN_INTERVAL_MS;
  if (armed_ && cooled) {
    cmd_.kick_type = type;
    cmd_.kick_seq++;
    last_kick_ms_ = millis();
    click();
  } else {
    click(false);
  }
}

void Ui::handleWheel() {
  for (int i = 0; i < 4; i++) {
    if (tapped(4 + i * (kSelW + kSelGap), kSelY, kSelW, kSelH)) {
      cmd_.wheel_mask ^= (1 << i);
      click();
    }
  }
  if (tapped(4 + 4 * (kSelW + kSelGap), kSelY, kSelW, kSelH)) {
    cmd_.wheel_mask = (cmd_.wheel_mask == 0x0F) ? 0 : 0x0F;
    click();
  }

  if (tapped(4, kTgtY, kTgtW, kTgtH)) {
    cmd_.wheel_target_radps = Clamp(cmd_.wheel_target_radps - WHEEL_TEST_STEP_RADPS,
                                    -WHEEL_TEST_MAX_RADPS, WHEEL_TEST_MAX_RADPS);
    click();
  }
  if (tapped(160, kTgtY, kTgtW, kTgtH)) {
    cmd_.wheel_target_radps = Clamp(cmd_.wheel_target_radps + WHEEL_TEST_STEP_RADPS,
                                    -WHEEL_TEST_MAX_RADPS, WHEEL_TEST_MAX_RADPS);
    click();
  }

  // デッドマン方式: RUN ボタンを押している間だけ回す
  bool holding = touch_.pressed() && touch_.in(kRunX, kRunY, kRunW, kRunH);
  if (holding && touch_.justPressed() && (!armed_ || cmd_.wheel_mask == 0)) click(false);
  cmd_.wheel_run = armed_ && holding && cmd_.wheel_mask != 0;
}

// ============================================================================
// 描画
// ============================================================================

void Ui::draw() {
  spr_.fillSprite(kBg);
  switch (page_) {
    case kHome: drawHome(); break;
    case kDribble: drawDribble(); break;
    case kKick: drawKick(); break;
    case kWheel: drawWheel(); break;
    case kImu: drawImu(); break;
    default: break;
  }
  drawStatusBar();
  drawTabs();
  spr_.pushSprite(0, 0);
}

void Ui::card(int16_t x, int16_t y, int16_t w, int16_t h, const char *title) {
  spr_.fillRoundRect(x, y, w, h, 6, kPanel);
  if (title) {
    spr_.setTextDatum(TL_DATUM);
    spr_.setTextColor(kMuted);
    spr_.drawString(title, x + 8, y + 6, 1);
  }
}

void Ui::button(int16_t x, int16_t y, int16_t w, int16_t h, const char *label, uint16_t bg,
                uint16_t fg, uint8_t font) {
  spr_.fillRoundRect(x, y, w, h, 6, bg);
  spr_.setTextDatum(MC_DATUM);
  spr_.setTextColor(fg);
  spr_.drawString(label, x + w / 2, y + h / 2 + 1, font);
}

void Ui::hbar(int16_t x, int16_t y, int16_t w, int16_t h, float ratio, uint16_t color) {
  ratio = Clamp(ratio, 0.0f, 1.0f);
  spr_.fillRoundRect(x, y, w, h, h / 2, kPanelHi);
  int16_t fw = (int16_t)(w * ratio);
  if (fw >= h) spr_.fillRoundRect(x, y, fw, h, h / 2, color);
}

void Ui::signedBar(int16_t x, int16_t y, int16_t w, int16_t h, float value, float full_scale,
                   uint16_t color, bool has_marker, float marker) {
  spr_.fillRect(x, y, w, h, kPanelHi);
  int16_t cx = x + w / 2;
  int16_t len = (int16_t)(Clamp(value / full_scale, -1.0f, 1.0f) * (w / 2));
  if (len > 0) spr_.fillRect(cx, y, len, h, color);
  if (len < 0) spr_.fillRect(cx + len, y, -len, h, color);
  spr_.drawFastVLine(cx, y - 1, h + 2, kMuted);
  if (has_marker) {
    int16_t mx = cx + (int16_t)(Clamp(marker / full_scale, -1.0f, 1.0f) * (w / 2));
    spr_.fillRect(mx - 1, y - 2, 3, h + 4, kWarn);
  }
}

uint16_t Ui::batteryColor(float v) const {
  if (v < BATTERY_LOW_V) return kDanger;
  if (v < BATTERY_WARN_V) return kWarn;
  return kOk;
}

void Ui::drawStatusBar() {
  const bool link = link_.connected();
  const Telemetry &t = link_.telemetry();
  char buf[24];

  spr_.fillRect(0, 0, kW, kStatusH, kPanel);

  button(kLockX, kLockY, kLockW, kLockH, armed_ ? "ARMED" : "LOCKED", armed_ ? kDanger : kPanelHi,
         kText, 2);

  // 接続/制御状態
  const char *state;
  uint16_t color;
  if (!link) {
    state = "NO LINK", color = kDanger;
  } else if (t.has(UI_TLM_F1_ROCK_LINK)) {
    state = "ROCK", color = kAccent;
  } else if (t.has(UI_TLM_F1_TEST_ACTIVE)) {
    state = "TEST", color = kWarn;
  } else {
    state = "IDLE", color = kOk;
  }
  spr_.fillCircle(79, 13, 4, color);
  spr_.setTextDatum(ML_DATUM);
  spr_.setTextColor(color);
  spr_.drawString(state, 87, 14, 2);

  // 電源電圧・昇圧電圧
  spr_.setTextDatum(MR_DATUM);
  if (link) {
    snprintf(buf, sizeof(buf), "%.2fV", t.battery_v);
    spr_.setTextColor(batteryColor(t.battery_v));
  } else {
    snprintf(buf, sizeof(buf), "--.--V");
    spr_.setTextColor(kMuted);
  }
  spr_.drawString(buf, 222, 14, 2);

  if (link) {
    snprintf(buf, sizeof(buf), "%dV", (int)t.cap_v);
    spr_.setTextColor(t.cap_v >= CAP_READY_V ? kWarn : kText);
  } else {
    snprintf(buf, sizeof(buf), "---V");
    spr_.setTextColor(kMuted);
  }
  spr_.drawString(buf, 280, 14, 2);

  // ボール
  bool ball = link && t.has(UI_TLM_F1_BALL_DETECTED);
  if (ball) {
    spr_.fillCircle(302, 13, 8, kBall);
  } else {
    spr_.drawCircle(302, 13, 8, kMuted);
  }
  if (link && t.has(UI_TLM_F1_BALL_HOLD)) spr_.drawCircle(302, 13, 11, kBall);
}

void Ui::drawTabs() {
  spr_.fillRect(0, kTabY, kW, kTabH, kPanel);
  for (int i = 0; i < kPageCount; i++) {
    bool active = i == page_;
    int16_t x = i * kTabW;
    if (active) spr_.fillRect(x + 6, kTabY, kTabW - 12, 3, kAccent);
    spr_.setTextDatum(MC_DATUM);
    spr_.setTextColor(active ? kText : kMuted);
    spr_.drawString(kTabLabels[i], x + kTabW / 2, kTabY + kTabH / 2 + 1, 2);
  }
}

void Ui::drawLockHint() {
  const char *msg = nullptr;
  if (!link_.connected()) {
    msg = "NO LINK TO MAINBOARD";
  } else if (link_.telemetry().has(UI_TLM_F1_ROCK_LINK)) {
    msg = "ROCK5A IN CONTROL - TEST DISABLED";
  } else if (!armed_) {
    msg = "TAP [LOCKED] TO ARM";
  }
  if (!msg) return;
  spr_.setTextDatum(BC_DATUM);
  spr_.setTextColor(kMuted);
  spr_.drawString(msg, kW / 2, kTabY - 2, 1);
}

void Ui::drawHome() {
  const bool link = link_.connected();
  const Telemetry &t = link_.telemetry();
  char buf[24];

  // 電源電圧
  card(4, kTop, 154, 82, "BATTERY");
  spr_.setTextDatum(TL_DATUM);
  spr_.setTextColor(link ? batteryColor(t.battery_v) : kMuted);
  if (link) snprintf(buf, sizeof(buf), "%.2f", t.battery_v);
  else snprintf(buf, sizeof(buf), "--.--");
  spr_.drawString(buf, 12, kTop + 18, 4);
  spr_.setTextColor(kMuted);
  spr_.drawString("V", 100, kTop + 26, 2);
  hbar(12, kTop + 58, 138, 10,
       link ? (t.battery_v - BATTERY_EMPTY_V) / (BATTERY_FULL_V - BATTERY_EMPTY_V) : 0,
       batteryColor(t.battery_v));

  // 昇圧電圧
  card(162, kTop, 154, 82, "BOOST");
  spr_.setTextDatum(TL_DATUM);
  spr_.setTextColor(link ? kText : kMuted);
  if (link) snprintf(buf, sizeof(buf), "%d", (int)t.cap_v);
  else snprintf(buf, sizeof(buf), "---");
  spr_.drawString(buf, 170, kTop + 18, 4);
  spr_.setTextColor(kMuted);
  spr_.drawString("V", 230, kTop + 26, 2);
  const char *chg = !link ? "" : t.has(UI_TLM_F1_CHARGE_DONE) ? "READY" : t.has(UI_TLM_F1_CHARGING) ? "CHARGING" : "DISCHG";
  spr_.setTextDatum(TR_DATUM);
  spr_.setTextColor(t.has(UI_TLM_F1_CHARGE_DONE) ? kWarn : kMuted);
  spr_.drawString(chg, 308, kTop + 6, 1);
  hbar(170, kTop + 58, 138, 10, link ? t.cap_v / CAP_MAX_V : 0,
       t.cap_v >= CAP_READY_V ? kWarn : kAccent);

  // ボール
  const int16_t y2 = kTop + 88;
  card(4, y2, 96, 84, "BALL");
  bool det = link && t.has(UI_TLM_F1_BALL_DETECTED);
  bool hold = link && t.has(UI_TLM_F1_BALL_HOLD);
  if (det) spr_.fillCircle(52, y2 + 40, 18, kBall);
  else spr_.drawCircle(52, y2 + 40, 18, kMuted);
  if (hold) spr_.drawCircle(52, y2 + 40, 22, kBall);
  spr_.setTextDatum(BC_DATUM);
  spr_.setTextColor(det ? kBall : kMuted);
  spr_.drawString(hold ? "HOLD" : det ? "DETECT" : "NONE", 52, y2 + 80, 1);

  // ホイール実角速度
  card(104, y2, 212, 84, "WHEEL [rad/s]");
  for (int i = 0; i < 4; i++) {
    int16_t ry = y2 + 20 + i * 15;
    snprintf(buf, sizeof(buf), "M%d", i + 1);
    spr_.setTextDatum(ML_DATUM);
    spr_.setTextColor(kMuted);
    spr_.drawString(buf, 112, ry + 4, 1);
    signedBar(130, ry, 122, 8, link ? t.wheel_radps[i] : 0, WHEEL_DISPLAY_MAX_RADPS, kAccent);
    if (link) snprintf(buf, sizeof(buf), "%.1f", t.wheel_radps[i]);
    else snprintf(buf, sizeof(buf), "--");
    spr_.setTextDatum(MR_DATUM);
    spr_.setTextColor(kText);
    spr_.drawString(buf, 310, ry + 4, 1);
  }
}

void Ui::drawDribble() {
  const bool link = link_.connected();
  const Telemetry &t = link_.telemetry();
  char buf[16];

  spr_.setTextDatum(TL_DATUM);
  spr_.setTextColor(kText);
  spr_.drawString("DRIBBLER", 8, kTop + 2, 4);

  // 出力設定
  button(kDribMinusX, kDribAdjY, kDribAdjW, kDribAdjH, "-", kPanelHi, kText, 4);
  button(kDribPlusX, kDribAdjY, kDribAdjW, kDribAdjH, "+", kPanelHi, kText, 4);
  snprintf(buf, sizeof(buf), "%d", cmd_.dribble_pct);
  spr_.setTextDatum(MC_DATUM);
  spr_.setTextColor(kText);
  spr_.drawString(buf, 98, kDribAdjY + kDribAdjH / 2 - 6, 4);
  spr_.setTextColor(kMuted);
  spr_.drawString("%", 98, kDribAdjY + kDribAdjH / 2 + 16, 2);

  // ON/OFF
  uint16_t bg = !armed_ ? kPanel : cmd_.dribble_on ? kOk : kPanelHi;
  button(kDribOnX, kDribOnY, kDribOnW, kDribOnH, cmd_.dribble_on ? "ON" : "OFF", bg,
         armed_ ? kText : kMuted, 4);

  // 状態
  card(8, 132, 304, 40, nullptr);
  bool det = link && t.has(UI_TLM_F1_BALL_DETECTED);
  bool hold = link && t.has(UI_TLM_F1_BALL_HOLD);
  if (det) spr_.fillCircle(30, 152, 10, kBall);
  else spr_.drawCircle(30, 152, 10, kMuted);
  spr_.setTextDatum(ML_DATUM);
  spr_.setTextColor(det ? kBall : kMuted);
  spr_.drawString(hold ? "BALL HOLD" : det ? "BALL DETECT" : "NO BALL", 48, 153, 2);
  spr_.setTextDatum(MR_DATUM);
  spr_.setTextColor(kText);
  if (link) snprintf(buf, sizeof(buf), "OUT %d%%", t.dribble_pct);
  else snprintf(buf, sizeof(buf), "OUT --");
  spr_.drawString(buf, 304, 153, 2);

  drawLockHint();
}

void Ui::drawKick() {
  const bool link = link_.connected();
  const Telemetry &t = link_.telemetry();
  char buf[24];

  // 昇圧電圧バー (CAP_READY_V に目盛り)
  spr_.setTextDatum(TL_DATUM);
  spr_.setTextColor(kMuted);
  spr_.drawString("BOOST", 8, kTop + 2, 1);
  spr_.setTextDatum(TR_DATUM);
  spr_.setTextColor(t.has(UI_TLM_F1_CHARGE_DONE) ? kWarn : kText);
  if (link) snprintf(buf, sizeof(buf), "%dV %s", (int)t.cap_v, t.has(UI_TLM_F1_CHARGE_DONE) ? "READY" : "");
  else snprintf(buf, sizeof(buf), "---V");
  spr_.drawString(buf, 312, kTop, 2);
  hbar(8, kTop + 18, 304, 14, link ? t.cap_v / CAP_MAX_V : 0,
       t.cap_v >= CAP_READY_V ? kWarn : kAccent);
  int16_t mark = 8 + (int16_t)(304 * CAP_READY_V / CAP_MAX_V);
  spr_.drawFastVLine(mark, kTop + 15, 20, kText);

  // 充電/放電
  uint16_t bg = !armed_ ? kPanel : cmd_.charge ? kWarn : kPanelHi;
  button(kChgX, kChgY, kChgW, kChgH, cmd_.charge ? "CHARGE" : "DISCHG", bg,
         !armed_ ? kMuted : cmd_.charge ? kBg : kText, 2);

  // キック強さ
  button(kKpMinusX, kKpY, kKpW, kKpH, "-", kPanelHi, kText, 4);
  button(kKpPlusX, kKpY, kKpW, kKpH, "+", kPanelHi, kText, 4);
  snprintf(buf, sizeof(buf), "%d%%", cmd_.kick_pct);
  spr_.setTextDatum(MC_DATUM);
  spr_.setTextColor(kText);
  spr_.drawString(buf, (kKpMinusX + kKpW + kKpPlusX) / 2, kKpY + 16, 4);
  spr_.setTextColor(kMuted);
  spr_.drawString("POWER", (kKpMinusX + kKpW + kKpPlusX) / 2, kKpY + 38, 1);

  // キックボタン (ACK 受信で 300ms 緑に光る)
  bool flash = millis() - kick_flash_ms_ < 300;
  bool ready = armed_ && millis() - last_kick_ms_ >= KICK_MIN_INTERVAL_MS;
  uint16_t fg = ready ? kText : kMuted;
  uint16_t sbg = flash && cmd_.kick_type == UI_KICK_STRAIGHT ? kOk : ready ? kDanger : kPanel;
  uint16_t cbg = flash && cmd_.kick_type == UI_KICK_CHIP ? kOk : ready ? kDanger : kPanel;
  button(kStraightX, kKickY, kKickW, kKickH, "STRAIGHT", sbg, fg, 4);
  button(kChipX, kKickY, kKickW, kKickH, "CHIP", cbg, fg, 4);

  drawLockHint();
}

void Ui::drawWheel() {
  const bool link = link_.connected();
  const Telemetry &t = link_.telemetry();
  char buf[24];

  // 対象ホイール選択
  for (int i = 0; i < 4; i++) {
    bool sel = cmd_.wheel_mask & (1 << i);
    snprintf(buf, sizeof(buf), "M%d", i + 1);
    button(4 + i * (kSelW + kSelGap), kSelY, kSelW, kSelH, buf, sel ? kAccent : kPanelHi,
           sel ? kBg : kText, 2);
  }
  button(4 + 4 * (kSelW + kSelGap), kSelY, kSelW, kSelH, "ALL",
         cmd_.wheel_mask == 0x0F ? kAccent : kPanelHi, cmd_.wheel_mask == 0x0F ? kBg : kText, 2);

  // 目標角速度
  button(4, kTgtY, kTgtW, kTgtH, "-", kPanelHi, kText, 4);
  button(160, kTgtY, kTgtW, kTgtH, "+", kPanelHi, kText, 4);
  snprintf(buf, sizeof(buf), "%+.0f", cmd_.wheel_target_radps);
  spr_.setTextDatum(MC_DATUM);
  spr_.setTextColor(kText);
  spr_.drawString(buf, 105, kTgtY + 13, 4);
  snprintf(buf, sizeof(buf), "rad/s  (%+.0f rpm)", cmd_.wheel_target_radps * 60.0f / (2.0f * PI));
  spr_.setTextColor(kMuted);
  spr_.drawString(buf, 105, kTgtY + 34, 1);

  // RUN (押している間だけ回転)
  uint16_t bg = !armed_ || cmd_.wheel_mask == 0 ? kPanel : cmd_.wheel_run ? kOk : kDanger;
  button(kRunX, kRunY, kRunW, kRunH, cmd_.wheel_run ? "RUNNING" : "HOLD", bg,
         armed_ ? kText : kMuted, 4);
  spr_.setTextDatum(BC_DATUM);
  spr_.setTextColor(armed_ ? kText : kMuted);
  if (!cmd_.wheel_run) spr_.drawString("TO RUN", kRunX + kRunW / 2, kRunY + kRunH - 6, 1);

  // 実角速度 (黄マーカー = 目標)
  for (int i = 0; i < 4; i++) {
    int16_t ry = 122 + i * 18;
    bool sel = cmd_.wheel_mask & (1 << i);
    snprintf(buf, sizeof(buf), "M%d", i + 1);
    spr_.setTextDatum(ML_DATUM);
    spr_.setTextColor(sel ? kAccent : kMuted);
    spr_.drawString(buf, 6, ry + 5, 2);
    float target = (cmd_.wheel_run && sel) ? cmd_.wheel_target_radps : 0;
    signedBar(34, ry, 212, 10, link ? t.wheel_radps[i] : 0, WHEEL_TEST_MAX_RADPS, kAccent, sel,
              target);
    if (link) snprintf(buf, sizeof(buf), "%+.1f", t.wheel_radps[i]);
    else snprintf(buf, sizeof(buf), "--");
    spr_.setTextDatum(MR_DATUM);
    spr_.setTextColor(kText);
    spr_.drawString(buf, 314, ry + 5, 2);
  }

  if (link && t.flags2 & UI_TLM_F2_WHEEL_EMG) {
    spr_.setTextDatum(BC_DATUM);
    spr_.setTextColor(kDanger);
    spr_.drawString("WHEEL UNIT EMG", kW / 2, kTabY - 2, 1);
  } else {
    drawLockHint();
  }
}

void Ui::drawImu() {
  const bool link = link_.connected();
  const Telemetry &t = link_.telemetry();
  char buf[32];

  // 方位ダイヤル (機体前方 = 上。yaw は反時計回り正)
  const int16_t cx = 84, cy = kTop + 86, r = 76;
  spr_.fillCircle(cx, cy, r, kPanel);
  spr_.drawCircle(cx, cy, r, kPanelHi);
  for (int deg = 0; deg < 360; deg += 30) {
    float a = deg * DEG_TO_RAD;
    int16_t len = (deg % 90 == 0) ? 10 : 5;
    spr_.drawLine(cx + (r - len) * sinf(a), cy - (r - len) * cosf(a), cx + r * sinf(a),
                  cy - r * cosf(a), kMuted);
  }
  if (link) {
    float a = -t.yaw_deg * DEG_TO_RAD;  // 反時計回り正 -> 画面上では左回り
    int16_t tx = cx + (r - 14) * sinf(a), ty = cy - (r - 14) * cosf(a);
    spr_.drawWideLine(cx, cy, tx, ty, 4, kAccent, kPanel);
    spr_.fillCircle(tx, ty, 5, kAccent);
  }
  spr_.fillCircle(cx, cy, 4, kText);

  // 数値
  struct Row {
    const char *label;
    const char *fmt;
    float value;
  } rows[] = {
      {"YAW", "%+.1f deg", t.yaw_deg},
      {"YAW RATE", "%+.1f dps", t.yaw_rate_dps},
      {"ACCEL X", "%+.3f g", t.accel_x_g},
      {"ACCEL Y", "%+.3f g", t.accel_y_g},
  };
  for (int i = 0; i < 4; i++) {
    int16_t y = kTop + 2 + i * 28;
    spr_.setTextDatum(TL_DATUM);
    spr_.setTextColor(kMuted);
    spr_.drawString(rows[i].label, 172, y, 1);
    if (link) snprintf(buf, sizeof(buf), rows[i].fmt, rows[i].value);
    else snprintf(buf, sizeof(buf), "--");
    spr_.setTextColor(kText);
    spr_.drawString(buf, 172, y + 10, 2);
  }

  // 加速度ベクトル (±1g で円周)
  const int16_t ax = 274, ay = kTop + 140, ar = 26;
  spr_.drawCircle(ax, ay, ar, kPanelHi);
  spr_.drawFastHLine(ax - ar, ay, ar * 2, kPanelHi);
  spr_.drawFastVLine(ax, ay - ar, ar * 2, kPanelHi);
  if (link) {
    // 機体座標 x=前方(画面上), y=左(画面左)
    int16_t px = ax - (int16_t)(Clamp(t.accel_y_g, -1.0f, 1.0f) * ar);
    int16_t py = ay - (int16_t)(Clamp(t.accel_x_g, -1.0f, 1.0f) * ar);
    spr_.fillCircle(px, py, 4, kWarn);
  }

  if (link && !t.has(UI_TLM_F1_IMU_READY)) {
    spr_.setTextDatum(BC_DATUM);
    spr_.setTextColor(kDanger);
    spr_.drawString("IMU NOT READY", kW / 2, kTabY - 2, 1);
  }
}
