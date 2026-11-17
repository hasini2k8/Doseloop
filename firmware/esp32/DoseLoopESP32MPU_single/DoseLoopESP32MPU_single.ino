/*
 * DoseLoop wearable, MPU6050 only, for a standard ESP32 dev board ("ESP32 Dev Module").
 *
 * Counts reps from the MPU6050 worn on the wrist and sends them over USB to the DoseLoop
 * app's "Wearable" option (Chrome or Edge), which saves each set to the participant's record.
 *
 * Wiring:
 *   MPU6050 VCC -> 3V3 (not 5V)    GND -> GND
 *   MPU6050 SDA -> GPIO21          SCL -> GPIO22
 *   MPU6050 AD0 -> GND (address 0x68). XDA, XCL, INT: not connected.
 *   Optional active buzzer: + to GPIO25, - to GND.
 *
 * Arduino IDE: board "ESP32 Dev Module". No extra libraries.
 * Single file: the DoseLoop rep detector (firmware/core) is included below, nothing else to copy.
 *
 * Serial (115200), one line per message:
 *   board -> app  HELLO,DOSELOOP,1 | SET,START | REP,<n>,<tempo_ms>,<rom_deg>,<quality>
 *                 PART,<rom_deg> | SET,STOP,<reps>,<partials> | WARN,MPU6050_NOT_FOUND
 *   app -> board  EX,<exercise name> | CFG,<rom_deg>,<tempo_ms>,<target_reps>,<hr_max> | START | STOP
 */
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// ======================= DoseLoop core (from firmware/core/doseloop_core.h/.c) =======================
/*
 * DoseLoop core - portable signal processing for the wearable tracker.
 *
 * No HAL dependencies: this file builds for the STM32 target and for the host
 * unit tests (firmware/test/test_core.c) unchanged.
 */
#ifndef DOSELOOP_CORE_H
#define DOSELOOP_CORE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" { /* usable from the Arduino (C++) sketch */
#endif

#define DL_SAMPLE_HZ 50

/* ---------- Repetition detector (accelerometer tilt) ---------- */

typedef struct {
    float target_rom_deg;     /* prescribed range of motion for the exercise */
    uint16_t target_tempo_ms; /* prescribed time under movement per rep */
    uint16_t min_rep_ms;      /* anything faster is treated as a bump, not a rep */
    uint16_t max_rep_ms;      /* anything slower is a posture change, not a rep */
} rd_config_t;

typedef struct {
    uint16_t count;    /* running count of valid reps in this set */
    uint16_t tempo_ms; /* time under movement of this rep */
    uint16_t rom_deg;  /* peak displacement from rest */
    uint8_t quality;   /* 0-100: 50% range, 30% tempo, 20% smoothness */
} rd_rep_t;

typedef enum { RD_EVENT_NONE = 0, RD_EVENT_REP, RD_EVENT_PARTIAL } rd_event_t;

typedef struct {
    rd_config_t cfg;
    float angle_f;      /* low-pass filtered tilt angle, degrees */
    float rest_deg;     /* tilt at rest, captured at start of set */
    float cal_sum;
    uint16_t cal_n;
    bool moving;
    float peak_deg;
    float rough_sum;    /* accumulated |‖a‖ - 1g| while moving */
    uint16_t rough_n;
    uint32_t t_rest_ms; /* last time the limb was at rest */
    uint16_t count;
    uint16_t partials;
} rd_t;

void rd_default_config(rd_config_t *cfg);
void rd_init(rd_t *s, const rd_config_t *cfg);
/* Feed one accelerometer sample (in g). Fills *out for REP and PARTIAL events. */
rd_event_t rd_update(rd_t *s, float ax, float ay, float az, uint32_t t_ms, rd_rep_t *out);

/* ---------- Heart-rate detector (analog pulse sensor) ---------- */

typedef struct {
    float dc;
    float env;
    bool primed;
    bool above;
    uint32_t last_beat_ms;
    uint16_t ibi[4];
    uint8_t n;
    uint8_t idx;
    uint8_t bpm; /* 0 until two consecutive plausible beats are seen */
} hr_t;

void hr_init(hr_t *s);
/* Feed one ADC sample. Returns true on a detected beat (s->bpm is then current). */
bool hr_update(hr_t *s, uint16_t adc, uint32_t t_ms);

/* ---------- Host -> device command parsing ---------- */

typedef struct {
    uint16_t target_rom_deg;
    uint16_t target_tempo_ms;
    uint16_t target_reps;
    uint16_t hr_max;
} dl_cfg_cmd_t;

typedef enum { DL_CMD_NONE = 0, DL_CMD_CFG, DL_CMD_START, DL_CMD_STOP } dl_cmd_t;

/* Parses one line (no trailing newline). Fills *cfg for DL_CMD_CFG. */
dl_cmd_t dl_parse_line(const char *line, dl_cfg_cmd_t *cfg);

#ifdef __cplusplus
}
#endif

#endif

#include <math.h>
#include <stdio.h>
#include <string.h>

#define RAD_TO_DEG 57.29578f
#define ANGLE_ALPHA 0.2f   /* EMA coefficient, ~90 ms time constant at 50 Hz */
#define CAL_SAMPLES 25     /* 0.5 s of stillness defines the rest angle */
#define ENTER_FRAC 0.35f   /* leave rest when past 35% of prescribed range */
#define EXIT_FRAC 0.15f    /* back at rest when within 15% */
#define VALID_FRAC 0.50f   /* below half range the rep is logged as partial */

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* Angle between the sensor X axis and gravity, 0..180 degrees. */
static float tilt_deg(float ax, float ay, float az)
{
    return atan2f(sqrtf(ay * ay + az * az), ax) * RAD_TO_DEG;
}

void rd_default_config(rd_config_t *cfg)
{
    cfg->target_rom_deg = 80.0f;
    cfg->target_tempo_ms = 2000;
    cfg->min_rep_ms = 300;
    cfg->max_rep_ms = 10000;
}

void rd_init(rd_t *s, const rd_config_t *cfg)
{
    memset(s, 0, sizeof(*s));
    s->cfg = *cfg;
}

static uint8_t score_rep(const rd_t *s, float peak, uint32_t tempo_ms, float roughness)
{
    float rom = clampf(peak / s->cfg.target_rom_deg, 0.0f, 1.0f) * 100.0f;

    /* Full marks from 0.7x to 1.5x of prescribed tempo, falling to zero at 0.3x and 3x. */
    float r = (float)tempo_ms / (float)s->cfg.target_tempo_ms;
    float tempo;
    if (r < 0.7f) {
        tempo = clampf((r - 0.3f) / 0.4f, 0.0f, 1.0f) * 100.0f;
    } else if (r > 1.5f) {
        tempo = clampf((3.0f - r) / 1.5f, 0.0f, 1.0f) * 100.0f;
    } else {
        tempo = 100.0f;
    }

    /* Mean deviation from 1 g: <=0.05 g is controlled, >=0.5 g is jerky. */
    float smooth = clampf((0.5f - roughness) / 0.45f, 0.0f, 1.0f) * 100.0f;

    return (uint8_t)(0.5f * rom + 0.3f * tempo + 0.2f * smooth + 0.5f);
}

rd_event_t rd_update(rd_t *s, float ax, float ay, float az, uint32_t t_ms, rd_rep_t *out)
{
    float mag = sqrtf(ax * ax + ay * ay + az * az);
    if (mag < 0.1f) {
        return RD_EVENT_NONE; /* free fall or dead sensor: no usable gravity vector */
    }
    float angle = tilt_deg(ax, ay, az);

    if (s->cal_n < CAL_SAMPLES) {
        s->cal_sum += angle;
        s->cal_n++;
        s->rest_deg = s->cal_sum / (float)s->cal_n;
        s->angle_f = s->rest_deg;
        s->t_rest_ms = t_ms;
        return RD_EVENT_NONE;
    }

    s->angle_f += ANGLE_ALPHA * (angle - s->angle_f);
    float d = fabsf(s->angle_f - s->rest_deg);
    float rom = s->cfg.target_rom_deg;

    if (!s->moving) {
        if (d < EXIT_FRAC * rom) {
            s->t_rest_ms = t_ms;
        } else if (d > ENTER_FRAC * rom) {
            s->moving = true;
            s->peak_deg = d;
            s->rough_sum = 0.0f;
            s->rough_n = 0;
        }
        return RD_EVENT_NONE;
    }

    if (d > s->peak_deg) {
        s->peak_deg = d;
    }
    s->rough_sum += fabsf(mag - 1.0f);
    s->rough_n++;

    if (d >= EXIT_FRAC * rom) {
        return RD_EVENT_NONE;
    }

    /* Returned to rest: classify the excursion. */
    s->moving = false;
    uint32_t tempo = t_ms - s->t_rest_ms;
    s->t_rest_ms = t_ms;
    if (tempo < s->cfg.min_rep_ms || tempo > s->cfg.max_rep_ms) {
        return RD_EVENT_NONE;
    }

    float roughness = s->rough_n ? s->rough_sum / (float)s->rough_n : 0.0f;
    out->tempo_ms = (uint16_t)tempo;
    out->rom_deg = (uint16_t)(s->peak_deg + 0.5f);
    out->quality = score_rep(s, s->peak_deg, tempo, roughness);

    if (s->peak_deg < VALID_FRAC * rom) {
        s->partials++;
        out->count = s->count;
        return RD_EVENT_PARTIAL;
    }
    s->count++;
    out->count = s->count;
    return RD_EVENT_REP;
}

/* ---------- Heart rate ---------- */

#define HR_DC_ALPHA 0.02f
#define HR_ENV_DECAY 0.995f
#define HR_ENV_FLOOR 20.0f /* ADC counts; below this the sensor is off-skin */
#define HR_REFRACTORY_MS 300
#define HR_MAX_IBI_MS 1500

void hr_init(hr_t *s)
{
    memset(s, 0, sizeof(*s));
}

bool hr_update(hr_t *s, uint16_t adc, uint32_t t_ms)
{
    float x = (float)adc;
    if (!s->primed) {
        s->dc = x;
        s->primed = true;
        return false;
    }
    s->dc += HR_DC_ALPHA * (x - s->dc);
    float ac = x - s->dc;

    s->env *= HR_ENV_DECAY;
    if (ac > s->env) {
        s->env = ac;
    }
    if (s->env < HR_ENV_FLOOR) {
        s->above = false;
        return false;
    }

    float thr = 0.5f * s->env;
    if (s->above) {
        if (ac < 0.5f * thr) {
            s->above = false;
        }
        return false;
    }
    if (ac <= thr || (s->last_beat_ms && t_ms - s->last_beat_ms < HR_REFRACTORY_MS)) {
        return false;
    }

    s->above = true;
    uint32_t ibi = s->last_beat_ms ? t_ms - s->last_beat_ms : 0;
    s->last_beat_ms = t_ms;
    if (ibi == 0 || ibi > HR_MAX_IBI_MS) {
        s->n = 0; /* first beat, or a gap: restart the average */
        return true;
    }
    s->ibi[s->idx] = (uint16_t)ibi;
    s->idx = (uint8_t)((s->idx + 1) % 4);
    if (s->n < 4) {
        s->n++;
    }
    if (s->n >= 2) {
        uint32_t sum = 0;
        for (uint8_t i = 0; i < s->n; i++) {
            sum += s->ibi[i];
        }
        s->bpm = (uint8_t)((60000u * s->n + sum / 2) / sum);
    }
    return true;
}

/* ---------- Commands ---------- */

dl_cmd_t dl_parse_line(const char *line, dl_cfg_cmd_t *cfg)
{
    if (strcmp(line, "START") == 0) {
        return DL_CMD_START;
    }
    if (strcmp(line, "STOP") == 0) {
        return DL_CMD_STOP;
    }
    unsigned rom, tempo, reps, hr;
    if (sscanf(line, "CFG,%u,%u,%u,%u", &rom, &tempo, &reps, &hr) == 4) {
        if (rom < 5 || rom > 180 || tempo < 300 || tempo > 10000 || reps > 500 || hr > 250) {
            return DL_CMD_NONE;
        }
        cfg->target_rom_deg = (uint16_t)rom;
        cfg->target_tempo_ms = (uint16_t)tempo;
        cfg->target_reps = (uint16_t)reps;
        cfg->hr_max = (uint16_t)hr;
        return DL_CMD_CFG;
    }
    return DL_CMD_NONE;
}
// ======================= sketch =======================

#define I2C_SDA 21
#define I2C_SCL 22
#define PIN_LED 2      // onboard blue LED
#define PIN_BUTTON 0   // BOOT button: start / stop a set
#define PIN_BUZZER 25  // optional active buzzer

#define MPU 0x68

rd_t rd;
rd_config_t rdCfg;
bool mpuOk = false, setActive = false;
int targetReps = 10;
uint32_t buzzUntil = 0;

void send(const char *line) {
  Serial.print(line);
  Serial.print('\n');
}

void beep(uint32_t ms) {
  digitalWrite(PIN_BUZZER, HIGH);
  buzzUntil = millis() + ms;
}

// ---------- MPU6050 ----------
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

// ---------- sets ----------
void setStart() {
  rd_init(&rd, &rdCfg);
  setActive = true;
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
  if (strncmp(line, "EX,", 3) == 0) return;  // exercise name: nothing to show without a screen
  dl_cfg_cmd_t c;
  switch (dl_parse_line(line, &c)) {
    case DL_CMD_CFG:
      rdCfg.target_rom_deg = c.target_rom_deg;
      rdCfg.target_tempo_ms = c.target_tempo_ms;
      targetReps = c.target_reps;
      break;
    case DL_CMD_START:
      if (!setActive) setStart();
      break;
    case DL_CMD_STOP:
      if (setActive) setStop();
      break;
    default:
      break;
  }
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  mpuOk = mpuInit();

  rd_default_config(&rdCfg);
  rd_init(&rd, &rdCfg);
  send("HELLO,DOSELOOP,1");
  if (!mpuOk) send("WARN,MPU6050_NOT_FOUND");
}

void loop() {
  static char rx[64];
  static uint8_t rxLen = 0;
  static uint32_t nextSample = millis(), btnSince = 0;
  static bool btnDown = false;
  uint32_t now = millis();

  // Commands from the app.
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
      rxLen = 0;  // overlong line: drop it
    }
  }

  // BOOT button, debounced: start or stop a set.
  bool pressed = digitalRead(PIN_BUTTON) == LOW;
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

  // Motion: 50 samples a second into the rep detector.
  if ((int32_t)(now - nextSample) >= 0) {
    nextSample += 1000 / DL_SAMPLE_HZ;
    float ax, ay, az;
    if (setActive && mpuOk && mpuRead(ax, ay, az)) {
      rd_rep_t rep;
      rd_event_t ev = rd_update(&rd, ax, ay, az, now, &rep);
      char buf[48];
      if (ev == RD_EVENT_REP) {
        snprintf(buf, sizeof(buf), "REP,%u,%u,%u,%u", rep.count, rep.tempo_ms, rep.rom_deg, rep.quality);
        send(buf);
        if (targetReps && rep.count == targetReps) beep(400);     // dose reached
        else if (targetReps && rep.count > targetReps) beep(60);  // past the dose
      } else if (ev == RD_EVENT_PARTIAL) {
        snprintf(buf, sizeof(buf), "PART,%u", rep.rom_deg);
        send(buf);
      }
    }
  }

  // LED: off when idle, slow blink during a set, solid once the dose is reached,
  // fast blink if the sensor is missing.
  bool led = !mpuOk ? (now / 120) % 2 : !setActive ? false : (targetReps && rd.count >= targetReps) ? true : (now / 500) % 2;
  digitalWrite(PIN_LED, led);

  delay(1);  // let the CPU idle between passes (less heat)
}
