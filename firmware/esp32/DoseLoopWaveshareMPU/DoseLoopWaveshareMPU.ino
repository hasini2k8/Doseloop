/*
 * DoseLoop wearable for the Waveshare ESP32-S3-LCD-1.69, MPU6050 only, single file.
 *
 * Counts reps from an MPU6050 worn on the wrist, shows the count on the 1.69" screen, and
 * sends each rep over USB to the DoseLoop app's "Wearable" option (Chrome or Edge).
 *
 * Wiring (the board's I2C header, shared with its onboard IMU and clock):
 *   MPU6050 VCC -> 3V3 (never 5V)   GND -> GND
 *   MPU6050 SDA -> GPIO11           SCL -> GPIO10
 *   MPU6050 AD0 -> GND (address 0x68). XDA, XCL, INT: not connected.
 *   Do not use GPIO4-8 for anything: they drive the screen.
 *
 * Arduino IDE: board "ESP32S3 Dev Module", USB CDC On Boot: Enabled, Flash Size 16MB,
 *              Partition Scheme "16M Flash (3MB APP/9.9MB FATFS)" (not an "ESP SR" one).
 * Library: "GFX Library for Arduino" (moononournation). Nothing else to copy.
 *
 * Serial (115200):  board -> app  HELLO,DOSELOOP,1 | SET,START | REP,<n>,<tempo_ms>,<rom_deg>,<quality>
 *                                 PART,<rom_deg> | SET,STOP,<reps>,<partials> | WARN,MPU6050_NOT_FOUND
 *                   app -> board  EX,<name> | CFG,<rom_deg>,<tempo_ms>,<target_reps>,<hr_max> | START | STOP
 */
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Arduino_GFX_Library.h>

// Board pins (Waveshare ESP32-S3-LCD-1.69 schematic, V2 boards).
#define LCD_DC 4
#define LCD_CS 5
#define LCD_SCK 6
#define LCD_MOSI 7
#define LCD_RST 8
#define LCD_BL 15
#define I2C_SDA 11
#define I2C_SCL 10
#define PIN_BUTTON 0   // BOOT button: start / stop a set
#define PIN_BUZZER 42  // onboard buzzer (older boards: 33)
#define PIN_SYS_EN 41  // keeps the board on from its battery (older boards: 35)
#define MPU 0x68

Arduino_DataBus *bus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCK, LCD_MOSI, GFX_NOT_DEFINED);
Arduino_GFX *panel = new Arduino_ST7789(bus, LCD_RST, 0 /* rotation */, true /* IPS */, 240, 280, 0, 20, 0, 20);
Arduino_Canvas *canvas = new Arduino_Canvas(240, 280, panel);  // draw off-screen, then flush: no flicker
Arduino_GFX *gfx = canvas;  // what we draw on: the canvas, or the panel directly if the canvas cannot start
bool useCanvas = true;

// ======================= rep detector (DoseLoop core) =======================
#define DL_SAMPLE_HZ 50
#define ANGLE_ALPHA 0.2f
#define CAL_SAMPLES 25     // 0.5 s of stillness at the start of a set defines "rest"
#define ENTER_FRAC 0.35f
#define EXIT_FRAC 0.15f
#define VALID_FRAC 0.50f   // under half the range counts as a partial rep

struct RepConfig {
  float target_rom_deg = 80;
  uint16_t target_tempo_ms = 2000;
  uint16_t min_rep_ms = 300;
  uint16_t max_rep_ms = 10000;
};

struct Rep {
  uint16_t count, tempo_ms, rom_deg;
  uint8_t quality;  // 0-100: 50% range, 30% tempo, 20% smoothness
};

enum RepEvent { EV_NONE, EV_REP, EV_PARTIAL };

struct RepDetector {
  RepConfig cfg;
  float angle_f = 0, rest_deg = 0, cal_sum = 0, peak_deg = 0, rough_sum = 0;
  uint16_t cal_n = 0, rough_n = 0, count = 0, partials = 0;
  bool moving = false;
  uint32_t t_rest_ms = 0;

  void reset(const RepConfig &c) {
    *this = RepDetector();
    cfg = c;
  }

  static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

  uint8_t score(float peak, uint32_t tempo_ms, float roughness) const {
    float rom = clampf(peak / cfg.target_rom_deg, 0, 1) * 100;
    float r = (float)tempo_ms / cfg.target_tempo_ms, tempo;
    if (r < 0.7f) tempo = clampf((r - 0.3f) / 0.4f, 0, 1) * 100;
    else if (r > 1.5f) tempo = clampf((3.0f - r) / 1.5f, 0, 1) * 100;
    else tempo = 100;
    float smooth = clampf((0.5f - roughness) / 0.45f, 0, 1) * 100;
    return (uint8_t)(0.5f * rom + 0.3f * tempo + 0.2f * smooth + 0.5f);
  }

  RepEvent update(float ax, float ay, float az, uint32_t t_ms, Rep &out) {
    float mag = sqrtf(ax * ax + ay * ay + az * az);
    if (mag < 0.1f) return EV_NONE;
    float angle = atan2f(sqrtf(ay * ay + az * az), ax) * 57.29578f;

    if (cal_n < CAL_SAMPLES) {
      cal_sum += angle;
      cal_n++;
      rest_deg = angle_f = cal_sum / cal_n;
      t_rest_ms = t_ms;
      return EV_NONE;
    }
    angle_f += ANGLE_ALPHA * (angle - angle_f);
    float d = fabsf(angle_f - rest_deg), rom = cfg.target_rom_deg;

    if (!moving) {
      if (d < EXIT_FRAC * rom) t_rest_ms = t_ms;
      else if (d > ENTER_FRAC * rom) {
        moving = true;
        peak_deg = d;
        rough_sum = 0;
        rough_n = 0;
      }
      return EV_NONE;
    }
    if (d > peak_deg) peak_deg = d;
    rough_sum += fabsf(mag - 1.0f);
    rough_n++;
    if (d >= EXIT_FRAC * rom) return EV_NONE;

    moving = false;
    uint32_t tempo = t_ms - t_rest_ms;
    t_rest_ms = t_ms;
    if (tempo < cfg.min_rep_ms || tempo > cfg.max_rep_ms) return EV_NONE;
    out.tempo_ms = tempo;
    out.rom_deg = (uint16_t)(peak_deg + 0.5f);
    out.quality = score(peak_deg, tempo, rough_n ? rough_sum / rough_n : 0);
    if (peak_deg < VALID_FRAC * rom) {
      partials++;
      out.count = count;
      return EV_PARTIAL;
    }
    out.count = ++count;
    return EV_REP;
  }
};

// ======================= sketch =======================
RepDetector rd;
RepConfig cfg;
bool mpuOk = false, setActive = false;
int targetReps = 10;
char exercise[40] = "Bicep curl";
uint8_t lastQuality = 0;
uint16_t lastTempo = 0, lastRom = 0;
uint32_t buzzUntil = 0;

// Live sensor readings, shown on the screen all the time.
float liveAx = 0, liveAy = 0, liveAz = 0;
float liveTilt = 0;     // tilt of the sensor's X axis from vertical, degrees (smoothed)
bool liveFresh = false; // a reading arrived in the last half second
uint32_t lastReadMs = 0;

void send(const char *line) {
  Serial.print(line);
  Serial.print('\n');
}

void beep(uint32_t ms) {
  digitalWrite(PIN_BUZZER, HIGH);
  buzzUntil = millis() + ms;
}

void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool mpuInit() {
  Wire.beginTransmission(MPU);
  if (Wire.endTransmission() != 0) return false;
  mpuWrite(0x6B, 0x00);  // wake up
  mpuWrite(0x1A, 0x04);  // 20 Hz low-pass filter
  mpuWrite(0x1C, 0x08);  // +-4 g
  return true;
}

bool mpuRead(float &ax, float &ay, float &az) {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU, 6) != 6) return false;
  int16_t x = (Wire.read() << 8) | Wire.read();
  int16_t y = (Wire.read() << 8) | Wire.read();
  int16_t z = (Wire.read() << 8) | Wire.read();
  ax = x / 8192.0f;
  ay = y / 8192.0f;
  az = z / 8192.0f;
  return true;
}

void setStart() {
  rd.reset(cfg);
  setActive = true;
  lastQuality = 0;
  lastTempo = lastRom = 0;
  send("SET,START");
  beep(80);
}

void setStop() {
  char buf[40];
  setActive = false;
  snprintf(buf, sizeof(buf), "SET,STOP,%u,%u", rd.count, rd.partials);
  send(buf);
}

void handleLine(const char *line) {
  unsigned rom, tempo, reps, hr;
  if (strncmp(line, "EX,", 3) == 0) {  // exercise name, shown on the screen
    strncpy(exercise, line + 3, sizeof(exercise) - 1);
    exercise[sizeof(exercise) - 1] = 0;
  } else if (strcmp(line, "START") == 0) {
    if (!setActive) setStart();
  } else if (strcmp(line, "STOP") == 0) {
    if (setActive) setStop();
  } else if (sscanf(line, "CFG,%u,%u,%u,%u", &rom, &tempo, &reps, &hr) == 4 && rom >= 5 && rom <= 180 && tempo >= 300 && tempo <= 10000 && reps <= 500) {
    cfg.target_rom_deg = rom;
    cfg.target_tempo_ms = tempo;
    targetReps = reps;
  }
}

// ---------- screen ----------
const uint16_t BG = 0x0A06, INK = 0xFFFF, MUTED = 0xB5D6, ACCENT = 0xFD8F, GOOD = 0x5EB5, ALERT = 0xF9C7, TRACK = 0x2A49;

void centered(const char *s, int y, int size, uint16_t color) {
  int16_t x1, y1;
  uint16_t w, h;
  gfx->setTextSize(size);
  gfx->getTextBounds(s, 0, 0, &x1, &y1, &w, &h);
  gfx->setTextColor(color);
  gfx->setCursor((240 - w) / 2, y);
  gfx->print(s);
}

// One accelerometer axis: small label, value in g, and a bar from the centre (-1 g .. +1 g).
void axis(const char *label, float v, int x) {
  char buf[12];
  gfx->setTextSize(1);
  gfx->setTextColor(MUTED);
  gfx->setCursor(x, 186);
  gfx->print(label);
  snprintf(buf, sizeof(buf), "%+.2f", v);
  gfx->setTextSize(2);
  gfx->setTextColor(INK);
  gfx->setCursor(x, 197);
  gfx->print(buf);
  int len = (int)(constrain(v, -1.0f, 1.0f) * 30);
  gfx->fillRect(x, 216, 60, 4, TRACK);
  if (len > 0) gfx->fillRect(x + 30, 216, len, 4, ACCENT);
  else if (len < 0) gfx->fillRect(x + 30 + len, 216, -len, 4, ACCENT);
}

void drawScreen() {
  char buf[40];
  bool reached = targetReps && rd.count >= targetReps;
  gfx->fillScreen(BG);
  centered(exercise, 14, 2, INK);

  // Reps: the count, the dose and a progress bar.
  snprintf(buf, sizeof(buf), "%u", rd.count);
  centered(buf, 38, 7, reached ? GOOD : INK);
  snprintf(buf, sizeof(buf), "of %d reps", targetReps);
  centered(buf, 92, 2, MUTED);
  int fill = targetReps ? min(200, (int)(200L * rd.count / targetReps)) : 0;
  gfx->fillRoundRect(20, 114, 200, 8, 4, TRACK);
  if (fill) gfx->fillRoundRect(20, 114, fill, 8, 4, reached ? GOOD : ACCENT);

  if (!mpuOk || !liveFresh) {
    centered("No sensor data", 160, 2, ALERT);
    centered("SDA GPIO11  SCL GPIO10", 186, 1, MUTED);
  } else {
    // Movement: during a set, how far the arm is from its resting position compared with
    // the prescribed range (the green tick is half the range: a full rep must pass it).
    // Outside a set, the plain tilt angle.
    float move = setActive && rd.cal_n >= CAL_SAMPLES ? fabsf(rd.angle_f - rd.rest_deg) : liveTilt;
    snprintf(buf, sizeof(buf), setActive ? "Moved %d deg" : "Tilt %d deg", (int)(move + 0.5f));
    gfx->setTextSize(2);
    gfx->setTextColor(INK);
    gfx->setCursor(20, 136);
    gfx->print(buf);
    float full = setActive ? cfg.target_rom_deg : 180.0f;
    int w = (int)(200 * constrain(move / full, 0.0f, 1.0f));
    gfx->fillRoundRect(20, 158, 200, 12, 6, TRACK);
    if (w) gfx->fillRoundRect(20, 158, max(w, 12), 12, 6, setActive && move >= VALID_FRAC * full ? GOOD : ACCENT);
    if (setActive) gfx->fillRect(20 + (int)(200 * VALID_FRAC), 155, 2, 18, GOOD);

    // Raw accelerometer, in g.
    axis("X", liveAx, 20);
    axis("Y", liveAy, 90);
    axis("Z", liveAz, 160);
  }

  // Last rep: range, time and form score.
  if (lastQuality) {
    snprintf(buf, sizeof(buf), "%u deg  %.1fs  Form %u%%", lastRom, lastTempo / 1000.0f, lastQuality);
    centered(buf, 232, 1 + (strlen(buf) <= 18), lastQuality >= 70 ? GOOD : ACCENT);
  }
  const char *hint = !mpuOk ? "Sensor not found" : setActive ? (reached ? "Dose reached: stop" : "Set in progress") : "Press BOOT to start";
  centered(hint, 252, 2, !mpuOk ? ALERT : setActive ? ACCENT : MUTED);
  if (useCanvas) canvas->flush();
}

void setup() {
  pinMode(PIN_SYS_EN, OUTPUT);
  digitalWrite(PIN_SYS_EN, HIGH);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(LCD_BL, OUTPUT);
  analogWrite(LCD_BL, 140);  // backlight about 55%: less heat

  Serial.begin(115200);
  delay(300);
  send("HELLO,DOSELOOP,1");
  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  // The canvas needs 134 KB of memory; if it cannot get it, draw straight to the screen instead.
  if (canvas->begin()) {
    send("LCD,OK");
  } else {
    useCanvas = false;
    gfx = panel;
    send(panel->begin() ? "LCD,OK_DIRECT" : "WARN,LCD_INIT_FAILED");
  }

  mpuOk = mpuInit();
  rd.reset(cfg);
  if (!mpuOk) send("WARN,MPU6050_NOT_FOUND");
  drawScreen();
}

void loop() {
  static char rx[64];
  static uint8_t rxLen = 0;
  static uint32_t nextSample = millis(), nextDraw = 0, btnSince = 0;
  static bool btnDown = false;
  uint32_t now = millis();

  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (rxLen) {
        rx[rxLen] = 0;
        handleLine(rx);
      }
      rxLen = 0;
    } else if (rxLen < sizeof(rx) - 1) {
      rx[rxLen++] = ch;
    } else {
      rxLen = 0;
    }
  }

  bool pressed = digitalRead(PIN_BUTTON) == LOW;  // BOOT button, debounced
  if (pressed && !btnDown) {
    if (!btnSince) btnSince = now;
    else if (now - btnSince > 30) {
      btnDown = true;
      if (setActive) setStop();
      else setStart();
    }
  } else if (!pressed) {
    btnSince = 0;
    btnDown = false;
  }

  if (buzzUntil && (int32_t)(now - buzzUntil) >= 0) {
    digitalWrite(PIN_BUZZER, LOW);
    buzzUntil = 0;
  }

  // Sensor missing: look for it again every 2 seconds, so fixing the wiring works without a reset.
  static uint32_t nextRetry = 0;
  if (!mpuOk && (int32_t)(now - nextRetry) >= 0) {
    nextRetry = now + 2000;
    if ((mpuOk = mpuInit())) send("WARN,MPU6050_FOUND");
  }

  if ((int32_t)(now - nextSample) >= 0) {  // motion: 50 samples a second
    nextSample += 1000 / DL_SAMPLE_HZ;
    float ax, ay, az;
    bool got = mpuOk && mpuRead(ax, ay, az);
    if (got) {  // live readings for the screen, in or out of a set
      liveAx += 0.3f * (ax - liveAx);
      liveAy += 0.3f * (ay - liveAy);
      liveAz += 0.3f * (az - liveAz);
      float tilt = atan2f(sqrtf(ay * ay + az * az), ax) * 57.29578f;
      liveTilt += 0.3f * (tilt - liveTilt);
      lastReadMs = now;
    }
    liveFresh = lastReadMs && now - lastReadMs < 500;
    if (setActive && got) {
      Rep rep;
      RepEvent ev = rd.update(ax, ay, az, now, rep);
      char buf[48];
      if (ev == EV_REP) {
        snprintf(buf, sizeof(buf), "REP,%u,%u,%u,%u", rep.count, rep.tempo_ms, rep.rom_deg, rep.quality);
        send(buf);
        lastQuality = rep.quality;
        lastTempo = rep.tempo_ms;
        lastRom = rep.rom_deg;
        if (targetReps && rep.count == targetReps) beep(400);     // dose reached
        else if (targetReps && rep.count > targetReps) beep(60);  // past the dose
      } else if (ev == EV_PARTIAL) {
        snprintf(buf, sizeof(buf), "PART,%u", rep.rom_deg);
        send(buf);
      }
    }
  }

  if ((int32_t)(now - nextDraw) >= 0) {
    nextDraw = now + 200;  // 5 frames a second
    drawScreen();
  }
  delay(1);  // let the CPU idle between passes (less heat)
}
