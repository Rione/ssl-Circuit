// LCD 診断用ファーム (pio run -e lcd_diag -t upload)
// TFT_eSPI を使わず最小限の ILI9341 初期化を行い、SPI 速度を変えながら単色塗りつぶしを繰り返す。
// USB シリアル(115200)に現在の状態を出力する。'p' を送るとピントグルテストを行う。
#include <Arduino.h>
#include <SPI.h>

static constexpr uint8_t kSck = 2, kMosi = 3, kMiso = 4;
static constexpr uint8_t kCs = 1, kDc = 27, kRst = 26, kTouchCs = 6, kBacklight = 0;

static uint32_t g_freq = 1000000;

static void Cmd(uint8_t c, const uint8_t *data = nullptr, size_t n = 0) {
  SPI.beginTransaction(SPISettings(g_freq, MSBFIRST, SPI_MODE0));
  digitalWrite(kCs, LOW);
  digitalWrite(kDc, LOW);
  SPI.transfer(c);
  digitalWrite(kDc, HIGH);
  for (size_t i = 0; i < n; i++) SPI.transfer(data[i]);
  digitalWrite(kCs, HIGH);
  SPI.endTransaction();
}

static void InitLcd() {
  digitalWrite(kRst, HIGH);
  delay(5);
  digitalWrite(kRst, LOW);
  delay(20);
  digitalWrite(kRst, HIGH);
  delay(150);

  Cmd(0x01);  // SWRESET
  delay(150);
  Cmd(0x11);  // SLPOUT
  delay(150);
  const uint8_t pixfmt = 0x55;  // 16bit/pixel
  Cmd(0x3A, &pixfmt, 1);
  const uint8_t madctl = 0x48;
  Cmd(0x36, &madctl, 1);
  Cmd(0x29);  // DISPON
  delay(50);
}

static void Fill(uint16_t color) {
  const uint8_t col[] = {0, 0, 0, 239};
  const uint8_t row[] = {0, 0, 0x01, 0x3F};
  Cmd(0x2A, col, 4);
  Cmd(0x2B, row, 4);

  SPI.beginTransaction(SPISettings(g_freq, MSBFIRST, SPI_MODE0));
  digitalWrite(kCs, LOW);
  digitalWrite(kDc, LOW);
  SPI.transfer(0x2C);  // RAMWR
  digitalWrite(kDc, HIGH);
  for (uint32_t i = 0; i < 240UL * 320UL; i++) {
    SPI.transfer(color >> 8);
    SPI.transfer(color & 0xFF);
  }
  digitalWrite(kCs, HIGH);
  SPI.endTransaction();
}

static void PinToggleTest() {
  struct {
    uint8_t pin;
    const char *name;
  } pins[] = {{kCs, "CS (LCD pin3)"},   {kRst, "RESET (LCD pin4)"}, {kDc, "D/C (LCD pin5)"},
              {kMosi, "MOSI (LCD pin6)"}, {kSck, "SCK (LCD pin7)"},   {kBacklight, "LED (LCD pin8)"}};

  SPI.end();  // SCK/MOSI を GPIO に戻す
  for (auto &p : pins) pinMode(p.pin, OUTPUT);
  for (auto &p : pins) {
    Serial.printf("GPIO%-2u %-16s : 1秒周期で5回 H/L (テスタで LCD 側の端子を確認)\n", p.pin, p.name);
    for (int i = 0; i < 5; i++) {
      digitalWrite(p.pin, HIGH);
      delay(500);
      digitalWrite(p.pin, LOW);
      delay(500);
    }
  }
  digitalWrite(kCs, HIGH);
  digitalWrite(kRst, HIGH);
  digitalWrite(kBacklight, HIGH);
  SPI.begin();
  Serial.println("ピントグルテスト終了");
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {
  }

  pinMode(kBacklight, OUTPUT);
  digitalWrite(kBacklight, HIGH);
  pinMode(kTouchCs, OUTPUT);
  digitalWrite(kTouchCs, HIGH);  // タッチ IC をバスから切り離す
  pinMode(kCs, OUTPUT);
  digitalWrite(kCs, HIGH);
  pinMode(kDc, OUTPUT);
  pinMode(kRst, OUTPUT);

  SPI.setSCK(kSck);
  SPI.setTX(kMosi);
  SPI.setRX(kMiso);
  SPI.begin();

  Serial.println("LCD 診断開始 ('p' でピントグルテスト)");
}

void loop() {
  static const uint32_t kFreqs[] = {1000000, 8000000, 20000000, 40000000};
  static const struct {
    uint16_t color;
    const char *name;
  } kColors[] = {{0xF800, "赤"}, {0x07E0, "緑"}, {0x001F, "青"}};

  for (uint32_t f : kFreqs) {
    g_freq = f;
    InitLcd();
    for (auto &c : kColors) {
      if (Serial.available() && Serial.read() == 'p') PinToggleTest();
      Serial.printf("SPI %2lu MHz : %s\n", f / 1000000, c.name);
      Fill(c.color);
      delay(700);
    }
  }
}
