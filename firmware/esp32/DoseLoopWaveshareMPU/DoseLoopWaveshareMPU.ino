/*
 * DoseLoop wearable for the Waveshare ESP32-S3-LCD-1.69 + MPU6050 + MAX30102, single file.
 *
 * Counts reps from an MPU6050 worn on the wrist, shows the count on the 1.69" screen, and
 * sends each rep over USB to the DoseLoop app's "Wearable" option (Chrome or Edge).
 *
 * Wiring (the board's I2C header, shared with its onboard IMU and clock):
 *   MPU6050 VCC -> 3V3 (never 5V)   GND -> GND
 *   MPU6050 SDA -> GPIO11           SCL -> GPIO10
 *   MPU6050 AD0 -> GND (address 0x68). XDA, XCL, INT: not connected.
 *   MAX30102 VIN -> 3V3, GND -> GND, SDA -> GPIO11, SCL -> GPIO10 (same pins as the MPU6050;
 *   address 0x57, INT not connected).
 *   Do not use GPIO4-8 for anything: they drive the screen.
 *
 * Arduino IDE: board "ESP32S3 Dev Module", USB CDC On Boot: Enabled, Flash Size 16MB,
 *              Partition Scheme "16M Flash (3MB APP/9.9MB FATFS)" (not an "ESP SR" one).
 * Libraries: "GFX Library for Arduino" (moononournation) and "SparkFun MAX3010x Pulse and
 * Proximity Sensor Library" (used only to read the sensor). Nothing else to copy. (The SparkFun library prints an
 * "I2C_BUFFER_LENGTH redefined" warning on ESP32: harmless.)
 * Tools -> PSRAM: OPI PSRAM (the screen buffer uses it).
 *
 * Wi-Fi (optional): fill in WIFI_SSID / WIFI_PASS below. The board then sends its readings
 * about once a second to DOSELOOP_URL/api/device, shown on the website's "Wearable" page.
 * 2.4 GHz networks only; university networks that need a username (eduroam) will not work,
 * so use a phone hotspot or home Wi-Fi.
 *
 * Serial (115200):  board -> app  HELLO,DOSELOOP,1 | SET,START | REP,<n>,<tempo_ms>,<rom_deg>,<quality>
 *                                 PART,<rom_deg> | HR,<bpm> | SPO2,<percent> | WARN,HR_HIGH
 *                                 SET,STOP,<reps>,<partials> | WARN,MPU6050_NOT_FOUND | WARN,MAX30102_NOT_FOUND
 *                   app -> board  EX,<name> | CFG,<rom_deg>,<tempo_ms>,<target_reps>,<hr_max> | START | STOP
 */
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Arduino_GFX_Library.h>
#include "MAX30105.h"

// ---------------- settings ----------------
const char *WIFI_SSID = "";  // your Wi-Fi name; leave empty to use USB only
const char *WIFI_PASS = "";
// The website the readings go to: the online one, or your computer running the app with
// HOST=0.0.0.0 (for example "http://192.168.1.20:4198").
const char *DOSELOOP_URL = "https://doseloop-tau.vercel.app";
volatile int wifiLastCode = 0;  // last HTTP status (200 = the website got it); <0 = network error

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

// ======================= pulse detector =======================
// Heart rate and SpO2 from MAX30102 red/IR samples at 100 samples a second.
//
// Finds each heartbeat as a peak in the filtered infrared signal, ignores stretches where the
// finger moves, rejects beats that do not fit the recent rhythm, and reports the median of
// recent beat-to-beat intervals. So the rate can be any value (72, 73, 74 ...) rather than
// the few steps the SparkFun/Maxim example produces. SpO2 uses the ratio of the red and
// infrared pulse sizes with Maxim's calibration curve.
//
// Same code as firmware/esp32/pulse_detector.h, which is tested on a computer
// (firmware/test/test_pulse.cpp).

struct PulseDetector {
  static const int HIST = 6;
  static constexpr float FINGER_IR = 50000;   // below this nothing is on the sensor
  static constexpr float SATURATED = 250000;  // above this the sensor is flooded (bright light)
  // A real pulse changes the light by roughly 0.3-2%. Sensor noise on an empty but bright
  // sensor is far smaller, so peaks under 0.08% of the level are not heartbeats.
  static constexpr float MIN_PULSE = 0.0008f;
  static const uint32_t NO_PULSE_MS = 4000;    // light but no pulse for this long: no finger

  // Outputs.
  int bpm = 0;    // 0 until 3 consistent beats
  int spo2 = 0;   // 0 until 3 beats with a usable red/IR ratio
  bool finger = false;   // something is on the sensor AND it has a pulse
  bool contact = false;  // enough light comes back (a finger, or just a reflection)
  bool moving = false;  // the last second was disturbed by movement

  // Filter state.
  float dcIr = 0, dcRed = 0;      // slow level (about 1 s), the "DC" in the SpO2 ratio
  float baseIr = 0, baseRed = 0;  // faster baseline (about 0.25 s): follows pressure changes
  float lpIr = 0, lpRed = 0, env = 0;
  uint16_t settle = 0;
  bool above = false;
  float peakVal = 0;
  uint32_t peakT = 0, lastPeakT = 0, quietFrom = 0;
  uint32_t contactT = 0, pulseT = 0;  // when contact began; the last pulse-sized peak
  // Sums over the current beat, for SpO2 with any straight-line drift removed:
  // x = infrared pulse, y = red pulse, k = sample number within the beat.
  float sX = 0, sY = 0, sK = 0, sXX = 0, sXY = 0, sKK = 0, sXK = 0, sYK = 0;
  uint16_t beatN = 0;
  uint16_t ibi[HIST] = {0};
  uint8_t nIbi = 0, rejects = 0;
  float amp[5] = {0};             // recent beat sizes, to recognise movement
  uint8_t nAmp = 0;
  float ratio[5] = {0};
  uint8_t nRatio = 0;

  void reset() { *this = PulseDetector(); }
  void clearBeat() { sX = sY = sK = sXX = sXY = sKK = sXK = sYK = 0; beatN = 0; }

  template <class T>
  static T median(const T *v, int n) {
    T s[8];
    for (int i = 0; i < n; i++) s[i] = v[i];
    for (int i = 1; i < n; i++)
      for (int j = i; j > 0 && s[j] < s[j - 1]; j--) {
        T x = s[j];
        s[j] = s[j - 1];
        s[j - 1] = x;
      }
    return n % 2 ? s[n / 2] : (s[n / 2 - 1] + s[n / 2]) / 2;
  }

  // One sample. Returns true when a heartbeat is accepted.
  bool update(uint32_t ir, uint32_t red, uint32_t t_ms) {
    if (ir < FINGER_IR || ir > SATURATED) {
      if (contact) reset();
      return false;
    }
    if (!contact) {  // something just arrived: start the filters at its level
      contact = true;
      contactT = t_ms;
      dcIr = baseIr = ir;
      dcRed = baseRed = red;
    }
    // A finger is only reported while pulses are being found (allowing a few seconds to find
    // the first). Light reflected off the room or the sensor cover has no pulse.
    finger = t_ms - contactT < NO_PULSE_MS || (pulseT && t_ms - pulseT < NO_PULSE_MS);
    if (!finger) bpm = spo2 = 0;
    dcIr += 0.01f * (ir - dcIr);
    dcRed += 0.01f * (red - dcRed);
    baseIr += 0.04f * (ir - baseIr);
    baseRed += 0.04f * (red - baseRed);
    // Inverted: blood arriving with each beat absorbs more light, so less comes back.
    lpIr += 0.3f * ((baseIr - ir) - lpIr);  // and smoothed above about 5 Hz
    lpRed += 0.3f * ((baseRed - red) - lpRed);
    if (settle < 150) {  // 1.5 s for the filters to settle after the finger arrives
      settle++;
      env = fabsf(lpIr);
      return false;
    }

    // Movement: a swing far bigger than recent heartbeats. Ignore the next 0.6 s and do not
    // count the gap as a long beat.
    float ampMed = nAmp >= 3 ? median(amp, nAmp) : 0;
    if (ampMed > 0 && fabsf(lpIr) > 1.5f * ampMed) {
      quietFrom = t_ms + 600;
      moving = true;
      pulseT = t_ms;  // a swing this big means a finger is there, just moving
      above = false;
      lastPeakT = 0;
      clearBeat();
      return false;
    }
    if ((int32_t)(t_ms - quietFrom) < 0) return false;
    if ((int32_t)(t_ms - quietFrom) > 1000) moving = false;

    float k = beatN++;
    sX += lpIr; sY += lpRed; sK += k;
    sXX += lpIr * lpIr; sXY += lpIr * lpRed; sKK += k * k; sXK += lpIr * k; sYK += lpRed * k;

    // Envelope of the pulse size, falling by half in about 1.4 s; a beat must reach 40% of it.
    env *= 0.995f;
    if (fabsf(lpIr) > env) env = fabsf(lpIr);
    float th = 0.4f * env;

    if (!above) {
      if (lpIr > th && (!lastPeakT || t_ms - lastPeakT > 330)) {  // at most about 180 bpm
        above = true;
        peakVal = lpIr;
        peakT = t_ms;
      }
      return false;
    }
    if (lpIr > peakVal) {
      peakVal = lpIr;
      peakT = t_ms;
    }
    if (lpIr > 0) return false;
    above = false;  // back below the middle: the peak just passed was a beat
    return beat();
  }

  bool beat() {
    uint32_t prev = lastPeakT;
    lastPeakT = peakT;
    // Covariances over the beat with the mean and a straight-line trend removed.
    float n = beatN, eIr = 0, eRI = 0;
    if (n > 10) {
      float cXX = sXX - sX * sX / n, cXY = sXY - sX * sY / n, cKK = sKK - sK * sK / n;
      float cXK = sXK - sX * sK / n, cYK = sYK - sY * sK / n;
      eIr = cXX - cXK * cXK / cKK;
      eRI = cXY - cXK * cYK / cKK;
    }
    clearBeat();
    if (peakVal < MIN_PULSE * dcIr) {  // far too small to be a heartbeat: sensor noise,
      lastPeakT = prev;                // so ignore it and keep timing from the last real beat
      return false;
    }
    pulseT = peakT;
    // Remember the beat size (for movement detection) even before the rhythm is known.
    for (int k = 4; k > 0; k--) amp[k] = amp[k - 1];
    amp[0] = peakVal;
    if (nAmp < 5) nAmp++;
    if (!prev) return false;
    uint32_t i = peakT - prev;
    if (i < 333 || i > 1500) {  // outside 40-180 bpm: a gap or a glitch, start again
      nIbi = 0;
      return false;
    }
    if (nIbi >= 3) {
      uint16_t med = median(ibi, nIbi);
      if (fabsf((float)i - med) > 0.25f * med) {  // does not fit the rhythm
        if (++rejects >= 3) nIbi = rejects = 0;   // three in a row: the rate really changed
        return false;
      }
    }
    rejects = 0;
    for (int k = HIST - 1; k > 0; k--) ibi[k] = ibi[k - 1];
    ibi[0] = (uint16_t)i;
    if (nIbi < HIST) nIbi++;
    if (nIbi >= 3) bpm = (int)(60000.0f / median(ibi, nIbi) + 0.5f);

    // SpO2 from the ratio of ratios R = (red AC / red DC) / (IR AC / IR DC), with AC measured
    // as how strongly the red pulse follows the infrared one over the whole beat: sensor
    // noise in the weaker red signal averages out instead of inflating its size.
    if (eIr > 0 && eRI > 0 && dcIr > 0 && dcRed > 0) {
      float R = (eRI / eIr) * dcIr / dcRed;
      if (R > 0.2f && R < 1.8f) {
        for (int k = 4; k > 0; k--) ratio[k] = ratio[k - 1];
        ratio[0] = R;
        if (nRatio < 5) nRatio++;
        if (nRatio >= 3) {
          float r = median(ratio, nRatio);
          float s = -45.060f * r * r + 30.354f * r + 94.845f;  // Maxim's calibration
          spo2 = s > 100 ? 100 : (s < 70 ? 0 : (int)(s + 0.5f));
        }
      }
    }
    return true;
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

// Heart rate and blood oxygen (MAX30102).
MAX30105 ox;
bool oxOk = false, finger = false;
int hr = 0, spo2 = 0;  // 0 = no reading yet
int hrMax = 130;       // the app sends the participant's limit in CFG

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

// MAX30102: 400 samples a second averaged in fours = 100 a second, read from the sensor's
// buffer and fed to the pulse detector with their own timing (10 ms apart), so the screen
// drawing cannot distort the beat intervals.
PulseDetector pulse;
uint32_t oxT = 0;
uint32_t lastIr = 0, lastRed = 0;  // latest raw readings, for the status line

void oxSetup() {
  if (!(oxOk = ox.begin(Wire, I2C_SPEED_FAST))) return;
  ox.setup(60 /* LED level */, 4 /* averaging */, 2 /* red + IR */, 400 /* samples/s */, 411 /* pulse width */, 4096 /* range */);
}

void oxPoll() {
  if (!oxOk) return;
  ox.check();
  while (ox.available()) {
    lastIr = ox.getFIFOIR();
    lastRed = ox.getFIFORed();
    pulse.update(lastIr, lastRed, oxT += 10);
    ox.nextSample();
  }
  finger = pulse.finger;
  hr = pulse.bpm;
  spo2 = pulse.spo2;
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
  unsigned rom, tempo, reps, hrLimit;
  if (strncmp(line, "EX,", 3) == 0) {  // exercise name, shown on the screen
    strncpy(exercise, line + 3, sizeof(exercise) - 1);
    exercise[sizeof(exercise) - 1] = 0;
  } else if (strcmp(line, "START") == 0) {
    if (!setActive) setStart();
  } else if (strcmp(line, "STOP") == 0) {
    if (setActive) setStop();
  } else if (sscanf(line, "CFG,%u,%u,%u,%u", &rom, &tempo, &reps, &hrLimit) == 4 && rom >= 5 && rom <= 180 && tempo >= 300 && tempo <= 10000 && reps <= 500) {
    cfg.target_rom_deg = rom;
    cfg.target_tempo_ms = tempo;
    targetReps = reps;
    hrMax = hrLimit;
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
  gfx->setCursor(x, 194);
  gfx->print(label);
  snprintf(buf, sizeof(buf), "%+.2f", v);
  gfx->setTextSize(2);
  gfx->setTextColor(INK);
  gfx->setCursor(x, 204);
  gfx->print(buf);
  int len = (int)(constrain(v, -1.0f, 1.0f) * 30);
  gfx->fillRect(x, 223, 60, 4, TRACK);
  if (len > 0) gfx->fillRect(x + 30, 223, len, 4, ACCENT);
  else if (len < 0) gfx->fillRect(x + 30 + len, 223, -len, 4, ACCENT);
}

void drawScreen() {
  char buf[40];
  bool reached = targetReps && rd.count >= targetReps;
  gfx->fillScreen(BG);
  // Wi-Fi state in words at the very top: is the website getting the readings?
  {
    char w[48];
    uint16_t c;
    if (!strlen(WIFI_SSID)) {
      snprintf(w, sizeof(w), "Wi-Fi off (USB only)");
      c = MUTED;
    } else if (WiFi.status() != WL_CONNECTED) {
      snprintf(w, sizeof(w), "Wi-Fi: connecting to %.20s...", WIFI_SSID);
      c = ACCENT;
    } else if (wifiLastCode == 200) {
      snprintf(w, sizeof(w), "Wi-Fi: connected, website OK");
      c = GOOD;
    } else if (wifiLastCode == 0) {
      snprintf(w, sizeof(w), "Wi-Fi: connected, sending...");
      c = ACCENT;
    } else {
      snprintf(w, sizeof(w), "Wi-Fi: connected, website error %d", wifiLastCode);
      c = ALERT;
    }
    centered(w, 4, 1, c);
  }
  centered(exercise, 18, 2, INK);

  // Reps: the count, the dose and a progress bar.
  snprintf(buf, sizeof(buf), "%u", rd.count);
  centered(buf, 38, 7, reached ? GOOD : INK);
  snprintf(buf, sizeof(buf), "of %d reps", targetReps);
  centered(buf, 92, 2, MUTED);
  int fill = targetReps ? min(200, (int)(200L * rd.count / targetReps)) : 0;
  gfx->fillRoundRect(20, 114, 200, 8, 4, TRACK);
  if (fill) gfx->fillRoundRect(20, 114, fill, 8, 4, reached ? GOOD : ACCENT);

  // Heart rate and oxygen.
  bool hrHigh = hrMax && hr > hrMax;
  if (!oxOk) {
    centered("Pulse sensor not found", 136, 1, ALERT);
  } else if (!finger) {
    centered("Put a finger on the pulse sensor", 136, 1, MUTED);
  } else if (!hr && pulse.moving) {
    centered("Hold your finger still", 136, 1, ACCENT);
  } else if (!hr) {
    centered("Reading pulse...", 136, 1, MUTED);
  } else {
    gfx->fillCircle(28, 139, 6, ALERT);
    snprintf(buf, sizeof(buf), hr ? "%d bpm" : "-- bpm", hr);
    gfx->setTextSize(2);
    gfx->setTextColor(hrHigh ? ALERT : INK);
    gfx->setCursor(40, 132);
    gfx->print(buf);
    snprintf(buf, sizeof(buf), spo2 ? "O2 %d%%" : "O2 --", spo2);
    gfx->setTextColor(spo2 && spo2 < 90 ? ALERT : INK);
    gfx->setCursor(144, 132);
    gfx->print(buf);
  }

  if (!mpuOk || !liveFresh) {
    centered("No motion data", 172, 2, ALERT);
    centered("SDA GPIO11  SCL GPIO10", 198, 1, MUTED);
  } else {
    // Movement: during a set, how far the arm is from its resting position compared with
    // the prescribed range (the green tick is half the range: a full rep must pass it).
    // Outside a set, the plain tilt angle.
    float move = setActive && rd.cal_n >= CAL_SAMPLES ? fabsf(rd.angle_f - rd.rest_deg) : liveTilt;
    snprintf(buf, sizeof(buf), setActive ? "Moved %d deg" : "Tilt %d deg", (int)(move + 0.5f));
    gfx->setTextSize(2);
    gfx->setTextColor(INK);
    gfx->setCursor(20, 156);
    gfx->print(buf);
    float full = setActive ? cfg.target_rom_deg : 180.0f;
    int w = (int)(200 * constrain(move / full, 0.0f, 1.0f));
    gfx->fillRoundRect(20, 176, 200, 10, 5, TRACK);
    if (w) gfx->fillRoundRect(20, 176, max(w, 10), 10, 5, setActive && move >= VALID_FRAC * full ? GOOD : ACCENT);
    if (setActive) gfx->fillRect(20 + (int)(200 * VALID_FRAC), 173, 2, 16, GOOD);

    // Raw accelerometer, in g.
    axis("X", liveAx, 20);
    axis("Y", liveAy, 90);
    axis("Z", liveAz, 160);
  }

  // Last rep: range, time and form score.
  if (lastQuality) {
    snprintf(buf, sizeof(buf), "%u deg  %.1fs  Form %u%%", lastRom, lastTempo / 1000.0f, lastQuality);
    centered(buf, 238, 1, lastQuality >= 70 ? GOOD : ACCENT);
  }
  const char *hint = hrHigh ? "Heart rate high: rest" : !mpuOk ? "Sensor not found" : setActive ? (reached ? "Dose reached: stop" : "Set in progress") : "Press BOOT to start";
  centered(hint, 256, 2, hrHigh || !mpuOk ? ALERT : setActive ? ACCENT : MUTED);
  if (useCanvas) canvas->flush();
}

// ---------------- Wi-Fi ----------------
// Runs on the other processor core, so a slow network never delays rep counting or the
// pulse sensor. The main loop writes a fresh JSON reading into wifiBody once a second;
// this task sends the newest one.
portMUX_TYPE wifiMux = portMUX_INITIALIZER_UNLOCKED;
char wifiBody[512];
volatile bool wifiFresh = false;
char deviceId[24];

void wifiTask(void *) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t lastTry = millis();
  String url = String(DOSELOOP_URL) + "/api/device";
  bool https = url.startsWith("https");
  WiFiClientSecure tls;
  tls.setInsecure();  // demo: the server certificate is not checked
  WiFiClient plain;
  char body[512];
  for (;;) {
    if (WiFi.status() != WL_CONNECTED) {
      if (millis() - lastTry > 10000) {  // keep trying every 10 s
        WiFi.disconnect();
        WiFi.begin(WIFI_SSID, WIFI_PASS);
        lastTry = millis();
      }
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }
    bool go = false;
    portENTER_CRITICAL(&wifiMux);
    if (wifiFresh) {
      memcpy(body, wifiBody, sizeof(body));
      wifiFresh = false;
      go = true;
    }
    portEXIT_CRITICAL(&wifiMux);
    if (go) {
      HTTPClient http;
      http.setReuse(true);
      http.setTimeout(3000);
      if (https) http.begin(tls, url);
      else http.begin(plain, url);
      http.addHeader("Content-Type", "application/json");
      wifiLastCode = http.POST((uint8_t *)body, strlen(body));
      http.end();
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// Called once a second from loop(): the latest readings as JSON for the website.
void queueReading(uint32_t now) {
  if (!strlen(WIFI_SSID)) return;
  char b[512];
  snprintf(b, sizeof(b),
           "{\"device\":\"%s\",\"motion\":%d,\"tilt\":%.1f,\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,"
           "\"pulse\":%d,\"finger\":%d,\"hr\":%d,\"spo2\":%d,\"ir\":%lu,\"hr_max\":%d,"
           "\"set_active\":%d,\"reps\":%u,\"target\":%d,\"quality\":%u,\"exercise\":\"%s\",\"rssi\":%d,\"uptime\":%lu}",
           deviceId, mpuOk && liveFresh ? 1 : 0, liveTilt, liveAx, liveAy, liveAz,
           oxOk ? 1 : 0, finger ? 1 : 0, hr, spo2, (unsigned long)lastIr, hrMax,
           setActive ? 1 : 0, rd.count, targetReps, lastQuality, exercise, (int)WiFi.RSSI(), (unsigned long)(now / 1000));
  portENTER_CRITICAL(&wifiMux);
  memcpy(wifiBody, b, sizeof(b));
  wifiFresh = true;
  portEXIT_CRITICAL(&wifiMux);
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
  oxSetup();
  rd.reset(cfg);
  if (!mpuOk) send("WARN,MPU6050_NOT_FOUND");
  if (!oxOk) send("WARN,MAX30102_NOT_FOUND");

  uint8_t mac[6];
  WiFi.macAddress(mac);
  snprintf(deviceId, sizeof(deviceId), "wearable-%02x%02x%02x", mac[3], mac[4], mac[5]);
  if (strlen(WIFI_SSID)) {
    xTaskCreatePinnedToCore(wifiTask, "wifi", 10240, nullptr, 1, nullptr, 0);
    char b[80];
    snprintf(b, sizeof(b), "WIFI,CONNECTING,%s", deviceId);
    send(b);
  }
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
  static uint32_t nextOxRetry = 0;
  if (!oxOk && (int32_t)(now - nextOxRetry) >= 0) {
    nextOxRetry = now + 3000;
    oxSetup();
    if (oxOk) send("WARN,MAX30102_FOUND");
  }

  // Heart rate and SpO2: sent every 2 seconds while a finger is on the sensor.
  static uint32_t lastHrSent = 0, lastWarn = 0;
  oxPoll();
  if (finger && hr && now - lastHrSent >= 2000) {
    char b[24];
    lastHrSent = now;
    snprintf(b, sizeof(b), "HR,%d", hr);
    send(b);
    if (spo2) {
      snprintf(b, sizeof(b), "SPO2,%d", spo2);
      send(b);
    }
    if (hrMax && hr > hrMax && now - lastWarn >= 10000) {
      lastWarn = now;
      send("WARN,HR_HIGH");
      beep(600);
    }
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

  // Status line once a second, so the Serial Monitor always shows the sensors working.
  // (The DoseLoop app ignores it.) ir/red are the raw light readings: about 50000 or more
  // means a finger is on the pulse sensor.
  static uint32_t nextStatus = 0;
  if ((int32_t)(now - nextStatus) >= 0) {
    nextStatus = now + 1000;
    queueReading(now);
    char st[200];
    snprintf(st, sizeof(st), "DATA,wifi=%s,motion=%s,tilt=%d,ax=%+.2f,ay=%+.2f,az=%+.2f,pulse=%s,ir=%lu,red=%lu,finger=%d,hr=%d,spo2=%d,reps=%u",
             !strlen(WIFI_SSID) ? "off" : WiFi.status() != WL_CONNECTED ? "connecting" : wifiLastCode == 200 ? "sent" : wifiLastCode == 0 ? "connected" : "error",
             mpuOk ? "ok" : "missing", (int)(liveTilt + 0.5f), liveAx, liveAy, liveAz, oxOk ? "ok" : "missing",
             (unsigned long)lastIr, (unsigned long)lastRed, finger ? 1 : 0, hr, spo2, rd.count);
    send(st);
  }

  if ((int32_t)(now - nextDraw) >= 0) {
    nextDraw = now + 200;  // 5 frames a second
    drawScreen();
  }
  delay(1);  // let the CPU idle between passes (less heat)
}
