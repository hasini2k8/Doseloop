/*
 * DoseLoop wearable for the Waveshare ESP32-S3-Touch-LCD-1.47 + MPU6050 + MAX30102, single file.
 *
 * Counts reps from an MPU6050 worn on the wrist, shows the count on the 1.47" screen, and
 * sends each rep over USB to the DoseLoop app's "Wearable" option (Chrome or Edge).
 *
 * Wiring (the board's I2C header, shared with its onboard IMU and clock):
 *   MPU6050 VCC -> 3V3 (never 5V)   GND -> GND
 *   MPU6050 SDA -> GPIO11           SCL -> GPIO10
 *   MPU6050 AD0 -> GND (address 0x68). XDA, XCL, INT: not connected.
 *   MAX30102 VIN -> 3V3, GND -> GND, SDA -> GPIO11, SCL -> GPIO10 (same pins as the MPU6050;
 *   address 0x57, INT not connected).
 *   Do not use GPIO21, 38-40, 45, 46 (screen) or 41, 42, 47, 48 (touch).
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
#include <Preferences.h>
#include <time.h>
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
volatile int wifiLastCode = 0;
volatile bool wifiWanted = true;  // the on-screen button turns Wi-Fi on and off (remembered)
Preferences prefs;  // last HTTP status (200 = the website got it); <0 = network error

// Board pins: Waveshare ESP32-S3-Touch-LCD-1.47 (Waveshare's Arduino example for the board).
#define LCD_DC 45
#define LCD_CS 21
#define LCD_SCK 38
#define LCD_MOSI 39
#define LCD_RST 40
#define LCD_BL 46
// The touch controller uses GPIO41, 42, 47 and 48: leave those alone.
#define I2C_SDA 11     // sensors (free header pins)
#define I2C_SCL 10
#define PIN_BUTTON 0   // BOOT button: start / stop a set
#define MPU 0x68

Arduino_DataBus *bus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCK, LCD_MOSI, GFX_NOT_DEFINED);
Arduino_GFX *panel = new Arduino_ST7789(bus, LCD_RST, 0 /* rotation */, false /* IPS */, 172, 320, 34, 0, 34, 0);
Arduino_Canvas *canvas = new Arduino_Canvas(172, 320, panel);  // draw off-screen, then flush: no flicker
Arduino_GFX *gfx = canvas;  // what we draw on: the canvas, or the panel directly if the canvas cannot start
bool useCanvas = true;

// The screen controller on this board is a JD9853. Arduino_GFX drives it as an ST7789, but it
// needs this start-up sequence first (from Waveshare's example for the board), or it stays dark.
void jd9853Init() {
  static const uint8_t ops[] = {
    BEGIN_WRITE, WRITE_COMMAND_8, 0x11, END_WRITE, DELAY, 120,  // wake up
    BEGIN_WRITE,
    WRITE_C8_D16, 0xDF, 0x98, 0x53,
    WRITE_C8_D8, 0xB2, 0x23,
    WRITE_COMMAND_8, 0xB7, WRITE_BYTES, 4, 0x00, 0x47, 0x00, 0x6F,
    WRITE_COMMAND_8, 0xBB, WRITE_BYTES, 6, 0x1C, 0x1A, 0x55, 0x73, 0x63, 0xF0,
    WRITE_C8_D16, 0xC0, 0x44, 0xA4,
    WRITE_C8_D8, 0xC1, 0x16,
    WRITE_COMMAND_8, 0xC3, WRITE_BYTES, 8, 0x7D, 0x07, 0x14, 0x06, 0xCF, 0x71, 0x72, 0x77,
    WRITE_COMMAND_8, 0xC4, WRITE_BYTES, 12, 0x00, 0x00, 0xA0, 0x79, 0x0B, 0x0A, 0x16, 0x79, 0x0B, 0x0A, 0x16, 0x82,
    WRITE_COMMAND_8, 0xC8, WRITE_BYTES, 32,
    0x3F, 0x32, 0x29, 0x29, 0x27, 0x2B, 0x27, 0x28, 0x28, 0x26, 0x25, 0x17, 0x12, 0x0D, 0x04, 0x00,
    0x3F, 0x32, 0x29, 0x29, 0x27, 0x2B, 0x27, 0x28, 0x28, 0x26, 0x25, 0x17, 0x12, 0x0D, 0x04, 0x00,
    WRITE_COMMAND_8, 0xD0, WRITE_BYTES, 5, 0x04, 0x06, 0x6B, 0x0F, 0x00,
    WRITE_C8_D16, 0xD7, 0x00, 0x30,
    WRITE_C8_D8, 0xE6, 0x14,
    WRITE_C8_D8, 0xDE, 0x01,
    WRITE_COMMAND_8, 0xB7, WRITE_BYTES, 5, 0x03, 0x13, 0xEF, 0x35, 0x35,
    WRITE_COMMAND_8, 0xC1, WRITE_BYTES, 3, 0x14, 0x15, 0xC0,
    WRITE_C8_D16, 0xC2, 0x06, 0x3A,
    WRITE_C8_D16, 0xC4, 0x72, 0x12,
    WRITE_C8_D8, 0xBE, 0x00,
    WRITE_C8_D8, 0xDE, 0x02,
    WRITE_COMMAND_8, 0xE5, WRITE_BYTES, 3, 0x00, 0x02, 0x00,
    WRITE_COMMAND_8, 0xE5, WRITE_BYTES, 3, 0x01, 0x02, 0x00,
    WRITE_C8_D8, 0xDE, 0x00,
    WRITE_C8_D8, 0x35, 0x00,
    WRITE_C8_D8, 0x3A, 0x05,
    WRITE_COMMAND_8, 0x2A, WRITE_BYTES, 4, 0x00, 0x22, 0x00, 0xCD,
    WRITE_COMMAND_8, 0x2B, WRITE_BYTES, 4, 0x00, 0x00, 0x01, 0x3F,
    WRITE_C8_D8, 0xDE, 0x02,
    WRITE_COMMAND_8, 0xE5, WRITE_BYTES, 3, 0x00, 0x02, 0x00,
    WRITE_C8_D8, 0xDE, 0x00,
    WRITE_C8_D8, 0x36, 0x00,
    WRITE_COMMAND_8, 0x21,
    END_WRITE, DELAY, 10,
    BEGIN_WRITE, WRITE_COMMAND_8, 0x29, END_WRITE  // screen on
  };
  bus->batchOperation(ops, sizeof(ops));
}

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
// Heart rate and SpO2 from MAX30102 red/IR samples (about 100 a second).
//
// Heart rate comes from the repeating rhythm of the last 8 seconds of the infrared signal
// (autocorrelation), not from picking individual peaks: a real pulse wave has a second,
// smaller bump after each beat, and peak picking counts it as an extra beat (reading double).
// Once a second the detector asks "after what delay does this signal best repeat itself?";
// that delay is one heartbeat. A clear repeat also tells it the reading is trustworthy, so
// movement, pressing and an empty sensor give no number rather than a wrong one.
//
// SpO2 comes from how strongly the red signal pulses compared with the infrared one over the
// same 8 seconds (the "ratio of ratios"), mapped with Maxim's calibration curve.
//
// Same code as firmware/esp32/pulse_detector.h, which is tested on a computer
// (firmware/test/test_pulse.cpp).

struct PulseDetector {
  static constexpr float FINGER_IR = 10000;   // below this nothing is on the sensor (empty reads ~1,300)
  static constexpr float SATURATED = 250000;  // above this the sensor is flooded (bright light)
  static const int N = 800;                   // 8 s window at 100 samples/s
  static const int MIN_N = 500;               // first estimate after 5 s
  static constexpr float MIN_QUALITY = 0.45f; // rhythm strength needed to report (0-1)
  // A real pulse changes the light by roughly 0.1-2%. Below this it is sensor noise.
  static constexpr float MIN_PULSE = 0.0004f;
  static const uint32_t NO_PULSE_MS = 6000;   // light but no pulse for this long: no finger

  // Outputs.
  int bpm = 0;           // 0 until a clear rhythm is found
  int spo2 = 0;          // 0 until a clear rhythm is found
  bool finger = false;   // something is on the sensor AND it has a pulse
  bool contact = false;  // enough light comes back (a finger, or just a reflection)
  bool moving = false;   // light changing but no clear rhythm: movement or pressing
  float quality = 0;     // rhythm strength of the last estimate, 0-1

  // State.
  float dcIr = 0, dcRed = 0, baseIr = 0, baseRed = 0, lpIr = 0, lpRed = 0;
  float x[N], y[N];      // filtered infrared and red; the newest is at head-1
  float cx[N], cy[N];    // the window being analysed, with movement bursts removed
  uint32_t tStamp[N];
  int head = 0, count = 0, sinceEstimate = 0;
  uint32_t contactT = 0, pulseT = 0, settleT = 0;
  int bpmHist[3] = {0}, nBpm = 0;
  float ratioHist[5] = {0};
  int nRatio = 0;

  void reset() {
    bpm = spo2 = 0;
    finger = contact = moving = false;
    quality = 0;
    head = count = sinceEstimate = 0;
    contactT = pulseT = settleT = 0;
    nBpm = nRatio = 0;
    lpIr = lpRed = 0;
  }

  template <class T>
  static T median(const T *v, int n) {
    T s[8];
    for (int i = 0; i < n; i++) s[i] = v[i];
    for (int i = 1; i < n; i++)
      for (int j = i; j > 0 && s[j] < s[j - 1]; j--) {
        T t = s[j];
        s[j] = s[j - 1];
        s[j - 1] = t;
      }
    return n % 2 ? s[n / 2] : (s[n / 2 - 1] + s[n / 2]) / 2;
  }

  // One sample. Returns true when a new heart-rate estimate was made.
  bool update(uint32_t ir, uint32_t red, uint32_t t_ms) {
    if (ir < FINGER_IR || ir > SATURATED) {
      if (contact) reset();
      return false;
    }
    if (!contact) {  // something just arrived: start the filters at its level
      contact = true;
      contactT = settleT = t_ms;
      dcIr = baseIr = ir;
      dcRed = baseRed = red;
    }
    // A sudden change of level (the finger shifted or was pressed) spoils the window: start again.
    if (fabsf(ir - dcIr) > 0.08f * dcIr) {
      dcIr = baseIr = ir;
      dcRed = baseRed = red;
      lpIr = lpRed = 0;
      head = count = sinceEstimate = 0;
      settleT = t_ms;
      moving = true;
    }
    finger = t_ms - contactT < NO_PULSE_MS || (pulseT && t_ms - pulseT < NO_PULSE_MS);
    if (!finger) bpm = spo2 = 0;

    dcIr += 0.01f * (ir - dcIr);  // level, about 1 s
    dcRed += 0.01f * (red - dcRed);
    baseIr += 0.04f * (ir - baseIr);  // baseline, about 0.25 s: follows pressure changes
    baseRed += 0.04f * (red - baseRed);
    lpIr += 0.3f * ((baseIr - ir) - lpIr);  // inverted (more blood = less light) and smoothed
    lpRed += 0.3f * ((baseRed - red) - lpRed);

    if (t_ms - settleT < 1500) return false;  // let the filters settle first
    x[head] = lpIr;
    y[head] = lpRed;
    tStamp[head] = t_ms;
    head = (head + 1) % N;
    if (count < N) count++;
    if (++sinceEstimate < 100 || count < MIN_N) return false;
    sinceEstimate = 0;
    return estimate();
  }


  // Heart rate and SpO2 from the samples in the window.
  bool estimate() {
    int n = count, start = (head - n + N) % N;
    // The real time between samples (the sensor's clock is not exactly 100 a second).
    float period = (float)(tStamp[(head - 1 + N) % N] - tStamp[start]) / (n - 1);
    float mx = 0, my = 0;
    for (int i = 0; i < n; i++) {
      cx[i] = x[(start + i) % N];
      cy[i] = y[(start + i) % N];
      mx += cx[i];
      my += cy[i];
    }
    mx /= n;
    my /= n;
    // Typical pulse size: the median of the size in each second of the window. A short jolt
    // only spoils one or two seconds, so it barely moves the median.
    float blk[8];
    int nb = 0;
    for (int b0 = 0; b0 + 100 <= n && nb < 8; b0 += 100, nb++) {
      float s = 0;
      for (int i = b0; i < b0 + 100; i++) s += (cx[i] - mx) * (cx[i] - mx);
      blk[nb] = sqrtf(s / 100);
    }
    float typical = median(blk, nb);
    if (period <= 0 || typical < MIN_PULSE * dcIr) {  // flat: nothing pulsing (or not skin)
      quality = 0;
      moving = false;
      return false;
    }
    // Samples far bigger than that are movement: leave them out (as zero) of both channels.
    float sxx = 0, sxy = 0;
    for (int i = 0; i < n; i++) {
      float a = cx[i] - mx, b = cy[i] - my;
      if (fabsf(a) > 3 * typical) a = b = 0;
      cx[i] = a;
      cy[i] = b;
      sxx += a * a;
      sxy += a * b;
    }
    float sd = sqrtf(sxx / n);

    // Autocorrelation for delays between 40 and 180 beats a minute; the strongest repeat wins.
    int lagMin = (int)(60000.0f / 180 / period), lagMax = (int)(60000.0f / 40 / period) + 1;
    if (lagMax > n / 2) lagMax = n / 2;
    float r[260];
    if (lagMax + 1 >= 260) lagMax = 258;
    for (int lag = lagMin - 1; lag <= lagMax + 1; lag++) {
      float s = 0;
      for (int i = 0; i + lag < n; i++) s += cx[i] * cx[i + lag];
      r[lag] = s / ((n - lag) * sd * sd);
    }
    float best = -1;
    int bestLag = 0;
    for (int lag = lagMin; lag <= lagMax; lag++)
      if (r[lag] > r[lag - 1] && r[lag] >= r[lag + 1] && r[lag] > best) {
        best = r[lag];
        bestLag = lag;
      }
    // Half-rate check: a slow answer whose half-length also repeats strongly is really two
    // beats (the strongest repeat skipped one). Use the half-length beat instead.
    if (bestLag) {
      int half = (bestLag + 1) / 2;
      float rh = -1;
      int hl = 0;
      for (int lag = half - 2; lag <= half + 2; lag++)
        if (lag >= lagMin && lag <= lagMax && r[lag] > rh) {
          rh = r[lag];
          hl = lag;
        }
      if (hl && hl > lagMin && hl < lagMax && r[hl] >= r[hl - 1] && r[hl] >= r[hl + 1] && rh > 0.75f * best) {
        bestLag = hl;
        best = rh;
      }
    }
    quality = best > 0 ? best : 0;
    moving = quality < 0.25f;
    if (quality < MIN_QUALITY || !bestLag) return false;
    pulseT = tStamp[(head - 1 + N) % N];

    // Refine the beat length between samples with a parabola through the peak.
    float a = r[bestLag - 1], b = r[bestLag], c = r[bestLag + 1], den = a - 2 * b + c;
    float shift = den < 0 ? 0.5f * (a - c) / den : 0;
    int est = (int)(60000.0f / ((bestLag + shift) * period) + 0.5f);
    for (int k = 2; k > 0; k--) bpmHist[k] = bpmHist[k - 1];
    bpmHist[0] = est;
    if (nBpm < 3) nBpm++;
    bpm = median(bpmHist, nBpm);

    // SpO2: R = (red pulse / red level) / (infrared pulse / infrared level), with the red pulse
    // measured as how strongly it follows the infrared one (sensor noise averages out).
    float R = (sxy / sxx) * dcIr / dcRed;
    if (R > 0.2f && R < 1.8f) {
      for (int k = 4; k > 0; k--) ratioHist[k] = ratioHist[k - 1];
      ratioHist[0] = R;
      if (nRatio < 5) nRatio++;
      float rr = median(ratioHist, nRatio);
      float s = -45.060f * rr * rr + 30.354f * rr + 94.845f;  // Maxim's calibration
      spo2 = s > 100 ? 100 : (s < 70 ? 0 : (int)(s + 0.5f));
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

// (This board has no buzzer or spare LED, so feedback is on the screen only.)
void flash(uint8_t, uint8_t, uint8_t, uint32_t) {}

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
uint32_t lastIr = 0, lastRed = 0;  // latest raw readings, for the status line
uint32_t oxLastSample = 0;          // when the pulse sensor last delivered a reading
uint16_t oxRestarts = 0;
bool rawStream = false;  // RAW,1 over USB: print every pulse-sensor reading (for checking the signal)

bool oxReg(uint8_t reg, uint8_t &val);

void oxSetup() {
  oxLastSample = millis();
  if (!(oxOk = ox.begin(Wire, I2C_SPEED_STANDARD))) return;
  ox.setup(36 /* LED level: about 7 mA. Brighter put fingertips near the sensor's maximum, flattening the pulse */, 4 /* averaging */, 2 /* red + IR */, 400 /* samples/s */, 411 /* pulse width */, 4096 /* range */);
  uint8_t clear;
  oxReg(0x00, clear);  // clear the power-up flag, so a later one means a real power dip
}

// Reads the sensor's own 32-reading buffer directly. (The SparkFun library keeps only 4
// readings, so whenever the screen is being redrawn some would be overwritten and lost,
// which throws off the beat timing.) Readings the sensor itself had to drop are counted
// from its overflow register, so the time between readings stays right.
#define OX_ADDR 0x57
uint32_t oxLost = 0;
// The sensor's clock is not exactly 100 readings a second (this one runs at about 103.5), so
// the real time between readings is measured and used, keeping heart rate accurate.
double oxPeriod = 10.0, oxTf = 0;
uint32_t oxCalStart = 0, oxCalCount = 0;

bool oxReg(uint8_t reg, uint8_t &val) {
  Wire.beginTransmission(OX_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(OX_ADDR, 1) != 1) return false;
  val = Wire.read();
  return true;
}

void oxSample(uint32_t red, uint32_t ir) {
  lastIr = ir;
  lastRed = red;
  if (!oxCalStart) oxCalStart = millis();
  if (++oxCalCount >= 500) {  // every ~5 s: readings per millisecond since the last check
    double p = (double)(millis() - oxCalStart) / oxCalCount;
    if (p > 8.5 && p < 11.5) oxPeriod = 0.7 * oxPeriod + 0.3 * p;
    oxCalStart = millis();
    oxCalCount = 0;
  }
  oxTf += oxPeriod;
  pulse.update(ir, red, (uint32_t)oxTf);
  if (rawStream) {
    char r[32];
    snprintf(r, sizeof(r), "R,%lu,%lu", (unsigned long)ir, (unsigned long)red);
    send(r);
  }
}

void oxPoll() {
  if (!oxOk) return;
  // 100 readings a second are expected. None for a second means the connection dropped
  // (usually a loose SDA/SCL wire): forget the stale reading and restart the sensor.
  if (millis() - oxLastSample > 1000) {
    pulse.reset();
    finger = false;
    hr = spo2 = 0;
    lastIr = lastRed = 0;
    oxRestarts++;
    uint8_t st;
    if (!oxReg(0x00, st)) send("WARN,MAX30102_RESTART,reason=no_answer");  // wire or connection
    else if (st & 0x01) send("WARN,MAX30102_RESTART,reason=power_dip");  // the sensor lost power and reset
    else send("WARN,MAX30102_RESTART,reason=stalled");
    oxSetup();  // if the sensor does not answer, oxOk goes false and loop() retries every 3 s
    return;
  }
  uint8_t wr, ovf, rp;
  if (!oxReg(0x04, wr) || !oxReg(0x05, ovf) || !oxReg(0x06, rp)) return;
  int n = (wr - rp) & 0x1F;
  ovf &= 0x1F;
  if (ovf) {  // the buffer filled up and the sensor dropped this many readings
    n = 32;
    oxTf += oxPeriod * ovf;
    oxCalStart = 0;  // readings were dropped: restart the rate measurement
    oxLost += ovf;
  }
  if (!n) return;
  oxLastSample = millis();
  Wire.beginTransmission(OX_ADDR);
  Wire.write(0x07);  // FIFO data: 3 bytes red, then 3 bytes infrared, per reading
  if (Wire.endTransmission(false) != 0) return;
  while (n > 0) {
    int k = n > 5 ? 5 : n;  // 30 bytes at a time fits the I2C buffer
    if (Wire.requestFrom(OX_ADDR, k * 6) != k * 6) return;
    for (int i = 0; i < k; i++) {
      uint32_t red = ((uint32_t)Wire.read() << 16) | ((uint32_t)Wire.read() << 8) | Wire.read();
      uint32_t ir = ((uint32_t)Wire.read() << 16) | ((uint32_t)Wire.read() << 8) | Wire.read();
      oxSample(red & 0x3FFFF, ir & 0x3FFFF);
    }
    n -= k;
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
  flash(0, 0, 60, 120);  // blue: set started
}

void setStop() {
  char buf[40];
  setActive = false;
  snprintf(buf, sizeof(buf), "SET,STOP,%u,%u", rd.count, rd.partials);
  send(buf);
}

void handleLine(const char *line) {
  unsigned rom, tempo, reps, hrLimit;
  if (strncmp(line, "BL,", 3) == 0) {  // backlight brightness 0-255 (for testing)
    analogWrite(LCD_BL, constrain(atoi(line + 3), 0, 255));
    return;
  }
  if (strcmp(line, "RAW,1") == 0 || strcmp(line, "RAW,0") == 0) {  // raw pulse readings on/off
    rawStream = line[4] == '1';
    return;
  }
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

// ---------- screen (172 x 320, portrait) ----------
const int SW = 172, SH = 320;
char deviceId[24] = "";
const uint16_t BG = 0x0A06, INK = 0xFFFF, MUTED = 0xB5D6, ACCENT = 0xFD8F, GOOD = 0x5EB5, ALERT = 0xF9C7, TRACK = 0x2A49;

void centered(const char *s, int y, int size, uint16_t color) {
  int16_t x1, y1;
  uint16_t w, h;
  gfx->setTextSize(size);
  gfx->getTextBounds(s, 0, 0, &x1, &y1, &w, &h);
  gfx->setTextColor(color);
  gfx->setCursor(w < SW ? (SW - w) / 2 : 0, y);
  gfx->print(s);
}

// Large text if it fits across the screen, otherwise small.
void centeredFit(const char *s, int y, uint16_t color) {
  centered(s, y, strlen(s) * 12 <= SW - 8 ? 2 : 1, color);
}

// One accelerometer axis as a row: label, value in g, and a bar from the centre (-1..+1 g).
void axisRow(const char *label, float v, int y) {
  char buf[12];
  gfx->setTextSize(1);
  gfx->setTextColor(MUTED);
  gfx->setCursor(10, y);
  gfx->print(label);
  snprintf(buf, sizeof(buf), "%+.2f", v);
  gfx->setTextColor(INK);
  gfx->setCursor(22, y);
  gfx->print(buf);
  int len = (int)(constrain(v, -1.0f, 1.0f) * 50);
  gfx->fillRect(60, y + 2, 102, 4, TRACK);
  if (len > 0) gfx->fillRect(111, y + 2, len, 4, ACCENT);
  else if (len < 0) gfx->fillRect(111 + len, y + 2, -len, 4, ACCENT);
}

// Wi-Fi state in words at the very top; on the exercise screen the time comes first.
void wifiLine(bool withTime) {
  char w[40], t[8] = "";
  uint16_t c;
  struct tm now;
  if (withTime && getLocalTime(&now, 0)) strftime(t, sizeof(t), "%H:%M ", &now);
  if (!strlen(WIFI_SSID)) {
    snprintf(w, sizeof(w), "%sWi-Fi not set up", t);
    c = MUTED;
  } else if (!wifiWanted) {
    snprintf(w, sizeof(w), "%sWi-Fi off", t);
    c = MUTED;
  } else if (WiFi.status() != WL_CONNECTED) {
    snprintf(w, sizeof(w), "%sWi-Fi: connecting...", t);
    c = ACCENT;
  } else if (wifiLastCode == 200) {
    snprintf(w, sizeof(w), "%sWi-Fi: online", t);
    c = GOOD;
  } else if (wifiLastCode == 0) {
    snprintf(w, sizeof(w), "%sWi-Fi: sending...", t);
    c = ACCENT;
  } else {
    snprintf(w, sizeof(w), "%sWi-Fi error %d", t, wifiLastCode);
    c = ALERT;
  }
  centered(w, 6, 1, c);
}

// A heart shape centred on (cx, cy), about 2*s wide.
void drawHeart(int cx, int cy, int s, uint16_t c) {
  gfx->fillCircle(cx - s / 2, cy - s / 4, s / 2 + 1, c);
  gfx->fillCircle(cx + s / 2, cy - s / 4, s / 2 + 1, c);
  gfx->fillTriangle(cx - s - 1, cy - s / 8, cx + s + 1, cy - s / 8, cx, cy + s, c);
}

// The on-screen Wi-Fi button on the watch face (tap to go online or offline).
const int BTN_X = 12, BTN_Y = 250, BTN_W = 148, BTN_H = 38;

// Watch face, shown whenever no set is running: the time, a heart that beats at the
// measured rate, the heart rate large, blood oxygen, and the Wi-Fi button.
void drawWatch() {
  char buf[40];
  bool hrHigh = hrMax && hr > hrMax;
  gfx->fillScreen(BG);
  wifiLine(false);

  struct tm now;
  if (getLocalTime(&now, 0)) {  // set from the internet once online; keeps running offline
    strftime(buf, sizeof(buf), "%H:%M", &now);
    centered(buf, 22, 4, INK);
    strftime(buf, sizeof(buf), "%a %d %b", &now);
    centered(buf, 58, 2, MUTED);
  } else {
    centered("--:--", 22, 4, TRACK);
    centered(strlen(WIFI_SSID) ? "Go online to set the time" : "No Wi-Fi set up", 60, 1, MUTED);
  }

  // Heart: grows briefly at each beat, at the measured rate.
  bool big = hr && (millis() % (60000UL / hr)) < 180;
  drawHeart(SW / 2, 98, big ? 17 : 13, hr ? ALERT : TRACK);

  snprintf(buf, sizeof(buf), hr ? "%d" : "--", hr);
  centered(buf, 124, 6, hrHigh ? ALERT : INK);
  centered("BPM", 170, 2, hrHigh ? ALERT : MUTED);

  snprintf(buf, sizeof(buf), spo2 ? "O2 %d%%" : "O2 --", spo2);
  centered(buf, 194, 2, spo2 && spo2 < 90 ? ALERT : INK);

  const char *hint = !oxOk ? "Pulse sensor not found"
                   : hrHigh ? "Heart rate high: rest"
                   : !finger ? "Rest a finger on the sensor"
                   : !hr && pulse.moving ? "Hold your finger still"
                   : !hr ? "Reading your pulse..."
                   : "BOOT: start exercise";
  centered(hint, 222, 1, hrHigh || !oxOk ? ALERT : !hr ? ACCENT : MUTED);

  // Wi-Fi button.
  if (strlen(WIFI_SSID)) {
    bool online = wifiWanted && WiFi.status() == WL_CONNECTED;
    uint16_t fill = !wifiWanted ? ACCENT : online ? GOOD : TRACK;
    gfx->fillRoundRect(BTN_X, BTN_Y, BTN_W, BTN_H, 12, fill);
    const char *label = !wifiWanted ? "Go online" : online ? "Online" : "Connecting";
    centered(label, BTN_Y + 11, 2, !wifiWanted || online ? BG : INK);
    centered(!wifiWanted ? "tap to connect to Wi-Fi" : "tap to disconnect", BTN_Y + BTN_H + 6, 1, MUTED);
  }
  if (useCanvas) canvas->flush();
}

// Exercise screen, during a set.
void drawWorkout() {
  char buf[40];
  bool reached = targetReps && rd.count >= targetReps;
  bool hrHigh = hrMax && hr > hrMax;
  gfx->fillScreen(BG);
  wifiLine(true);
  centeredFit(exercise, 22, INK);

  // Reps: the count, the dose and a progress bar.
  snprintf(buf, sizeof(buf), "%u", rd.count);
  centered(buf, 42, 8, reached ? GOOD : INK);
  snprintf(buf, sizeof(buf), "of %d reps", targetReps);
  centered(buf, 104, 2, MUTED);
  int fill = targetReps ? min(152, (int)(152L * rd.count / targetReps)) : 0;
  gfx->fillRoundRect(10, 126, 152, 8, 4, TRACK);
  if (fill) gfx->fillRoundRect(10, 126, fill, 8, 4, reached ? GOOD : ACCENT);

  // Heart rate and oxygen.
  if (!oxOk) {
    centered("Pulse sensor not found", 152, 1, ALERT);
  } else if (!finger) {
    centered("Rest a finger on the sensor", 152, 1, MUTED);
  } else if (!hr && pulse.moving) {
    centered("Hold your finger still", 152, 1, ACCENT);
  } else if (!hr) {
    centered("Reading pulse...", 152, 1, MUTED);
  } else {
    snprintf(buf, sizeof(buf), "%d bpm", hr);
    drawHeart(22, 152, 8, ALERT);
    centered(buf, 146, 2, hrHigh ? ALERT : INK);
    snprintf(buf, sizeof(buf), spo2 ? "O2 %d%%" : "O2 --", spo2);
    centered(buf, 166, 2, spo2 && spo2 < 90 ? ALERT : INK);
  }

  if (!mpuOk || !liveFresh) {
    centered("No motion data", 196, 2, ALERT);
    centered("SDA GPIO11  SCL GPIO10", 218, 1, MUTED);
  } else {
    // How far the arm is from where it started, against the prescribed range (the green
    // tick is half the range: a full rep must pass it).
    float move = rd.cal_n >= CAL_SAMPLES ? fabsf(rd.angle_f - rd.rest_deg) : 0;
    snprintf(buf, sizeof(buf), "Moved %d deg", (int)(move + 0.5f));
    centered(buf, 194, 2, INK);
    float full = cfg.target_rom_deg;
    int w = (int)(152 * constrain(move / full, 0.0f, 1.0f));
    gfx->fillRoundRect(10, 214, 152, 10, 5, TRACK);
    if (w) gfx->fillRoundRect(10, 214, max(w, 10), 10, 5, move >= VALID_FRAC * full ? GOOD : ACCENT);
    gfx->fillRect(10 + (int)(152 * VALID_FRAC), 211, 2, 16, GOOD);

    // Raw accelerometer, in g.
    axisRow("X", liveAx, 236);
    axisRow("Y", liveAy, 248);
    axisRow("Z", liveAz, 260);
  }

  // Last rep: range, time and form score.
  if (lastQuality) {
    snprintf(buf, sizeof(buf), "%udeg %.1fs Form %u%%", lastRom, lastTempo / 1000.0f, lastQuality);
    centered(buf, 278, 1, lastQuality >= 70 ? GOOD : ACCENT);
  }
  const char *hint = hrHigh ? "Heart rate high: rest" : !mpuOk ? "Sensor not found" : reached ? "Dose reached" : "BOOT: stop the set";
  centeredFit(hint, 296, hrHigh || !mpuOk ? ALERT : reached ? GOOD : ACCENT);
  if (useCanvas) canvas->flush();
}


// ---------------- touch screen (AXS5106L) ----------------
// Its own I2C connection, separate from the sensors. A touch is read by asking register 0x01
// for the number of fingers and the first finger's position.
#define TP_SDA 42
#define TP_SCL 41
#define TP_RST 47
#define TP_INT 48
#define TP_ADDR 0x63
bool touchOk = false;

void touchInit() {
  Wire1.begin(TP_SDA, TP_SCL, 400000);
  pinMode(TP_RST, OUTPUT);
  digitalWrite(TP_RST, LOW);
  delay(200);
  digitalWrite(TP_RST, HIGH);
  delay(300);
  pinMode(TP_INT, INPUT_PULLUP);
  Wire1.beginTransmission(TP_ADDR);
  touchOk = Wire1.endTransmission() == 0;
}

// True while a finger is on the screen; x, y in screen pixels.
bool touchRead(int &x, int &y) {
  if (!touchOk) return false;
  uint8_t d[14];
  Wire1.beginTransmission(TP_ADDR);
  Wire1.write(0x01);
  if (Wire1.endTransmission() != 0) return false;
  if (Wire1.requestFrom(TP_ADDR, 14) != 14) return false;
  for (int i = 0; i < 14; i++) d[i] = Wire1.read();
  if (d[1] == 0 || d[1] > 5) return false;
  x = SW - 1 - (((d[2] & 0x0F) << 8) | d[3]);
  y = ((d[4] & 0x0F) << 8) | d[5];
  return true;
}

// A tap (finger down) on the watch face's Wi-Fi button turns Wi-Fi on or off, remembered.
void handleTap(int x, int y) {
  char b[48];
  snprintf(b, sizeof(b), "TOUCH,%d,%d", x, y);
  send(b);
  if (setActive || !strlen(WIFI_SSID)) return;
  if (y >= BTN_Y - 10 && y <= BTN_Y + BTN_H + 16) {
    wifiWanted = !wifiWanted;
    prefs.putBool("wifi", wifiWanted);
    send(wifiWanted ? "WIFI,ON" : "WIFI,OFF");
  }
}

// The screen: the watch face between sets, the exercise screen during a set.
void drawScreen() {
  if (setActive) drawWorkout();
  else drawWatch();
}

// ---------------- Wi-Fi ----------------
// Runs on the other processor core, so a slow network never delays rep counting or the
// pulse sensor. The main loop writes a fresh JSON reading into wifiBody once a second;
// this task sends the newest one.
portMUX_TYPE wifiMux = portMUX_INITIALIZER_UNLOCKED;
char wifiBody[512];
volatile bool wifiFresh = false;

void wifiTask(void *) {
  bool started = false;
  uint32_t lastTry = 0;
  String url = String(DOSELOOP_URL) + "/api/device";
  bool https = url.startsWith("https");
  WiFiClientSecure tls;
  tls.setInsecure();  // demo: the server certificate is not checked
  WiFiClient plain;
  char body[512];
  for (;;) {
    if (!wifiWanted) {  // switched off with the on-screen button
      if (started) {
        WiFi.disconnect(true);  // also turns the radio off (saves power and heat)
        started = false;
        wifiLastCode = 0;
      }
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }
    if (!started) {
      WiFi.mode(WIFI_STA);
      WiFi.begin(WIFI_SSID, WIFI_PASS);
      started = true;
      lastTry = millis();
    }
    if (WiFi.status() != WL_CONNECTED) {
      if (millis() - lastTry > 10000) {  // keep trying every 10 s
        WiFi.disconnect();
        WiFi.begin(WIFI_SSID, WIFI_PASS);
        lastTry = millis();
      }
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }
    static bool timeSet = false;
    if (!timeSet) {
      configTzTime("EST5EDT,M3.2.0,M11.1.0", "pool.ntp.org", "time.nist.gov");
      timeSet = true;
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
  if (!strlen(WIFI_SSID) || !wifiWanted) return;
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
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(LCD_BL, OUTPUT);
  analogWrite(LCD_BL, 140);  // backlight about 55%: less heat

  Serial.begin(115200);
  delay(300);
  send("HELLO,DOSELOOP,1");
  Wire.begin(I2C_SDA, I2C_SCL, 100000);  // 100 kHz: copes better with jumper wires
  pinMode(LCD_RST, OUTPUT);  // reset the screen
  digitalWrite(LCD_RST, LOW);
  delay(10);
  digitalWrite(LCD_RST, HIGH);
  delay(10);
  // The canvas needs 110 KB of memory; if it cannot get it, draw straight to the screen instead.
  bool canvasOk = canvas->begin();
  jd9853Init();
  if (canvasOk) {
    send("LCD,OK");
  } else {
    useCanvas = false;
    gfx = panel;
    bool ok = panel->begin();
    jd9853Init();
    send(ok ? "LCD,OK_DIRECT" : "WARN,LCD_INIT_FAILED");
  }

  mpuOk = mpuInit();
  oxSetup();
  rd.reset(cfg);
  if (!mpuOk) send("WARN,MPU6050_NOT_FOUND");
  if (!oxOk) send("WARN,MAX30102_NOT_FOUND");

  // A name unique to this board, from the chip's factory ID (works before Wi-Fi starts).
  snprintf(deviceId, sizeof(deviceId), "wearable-%06lx", (unsigned long)((ESP.getEfuseMac() >> 24) & 0xFFFFFF));
  prefs.begin("doseloop", false);
  wifiWanted = prefs.getBool("wifi", true);
  touchInit();
  send(touchOk ? "TOUCH,OK" : "WARN,TOUCH_NOT_FOUND");
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
    // (nothing to switch off)
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
      flash(80, 0, 0, 600);  // red: heart rate over the limit
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
        if (targetReps && rep.count == targetReps) flash(0, 80, 0, 600);  // green, long: dose reached
        else if (targetReps && rep.count > targetReps) flash(80, 40, 0, 150);  // amber: past the dose
        else flash(0, 40, 0, 120);  // green: a rep
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
    snprintf(st, sizeof(st), "DATA,wifi=%s,motion=%s,tilt=%d,ax=%+.2f,ay=%+.2f,az=%+.2f,pulse=%s,ir=%lu,red=%lu,finger=%d,hr=%d,spo2=%d,ox_restarts=%u,ox_lost=%lu,reps=%u",
             !strlen(WIFI_SSID) || !wifiWanted ? "off" : WiFi.status() != WL_CONNECTED ? "connecting" : wifiLastCode == 200 ? "sent" : wifiLastCode == 0 ? "connected" : "error",
             mpuOk ? "ok" : "missing", (int)(liveTilt + 0.5f), liveAx, liveAy, liveAz, oxOk ? "ok" : "missing",
             (unsigned long)lastIr, (unsigned long)lastRed, finger ? 1 : 0, hr, spo2, oxRestarts, (unsigned long)oxLost, rd.count);
    send(st);
  }

  static uint32_t nextTouch = 0;
  static bool wasTouched = false;
  if ((int32_t)(now - nextTouch) >= 0) {
    nextTouch = now + 33;
    int tx, ty;
    bool touched = touchRead(tx, ty);
    if (touched && !wasTouched) {
      handleTap(tx, ty);
      nextDraw = now;  // show the change straight away
    }
    wasTouched = touched;
  }

  if ((int32_t)(now - nextDraw) >= 0) {
    nextDraw = now + 100;  // 10 frames a second (the watch face heart beats)
    drawScreen();
  }
  delay(1);  // let the CPU idle between passes (less heat)
}
