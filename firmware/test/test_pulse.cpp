// Host test for firmware/esp32/pulse_detector.h: synthetic MAX30102 signals at 100 samples/s.
// Build: g++ -std=c++17 -O2 -I../esp32 test_pulse.cpp -o test_pulse && ./test_pulse
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "pulse_detector.h"

static float gauss() {  // standard normal noise
  float u = (rand() + 1.0f) / (RAND_MAX + 2.0f), v = (rand() + 1.0f) / (RAND_MAX + 2.0f);
  return sqrtf(-2 * logf(u)) * cosf(6.2831853f * v);
}

// Pulse shape over one beat, phase 0..1: a systolic peak and a smaller dicrotic wave.
static float pulseShape(float ph) {
  float a = expf(-powf((ph - 0.18f) / 0.07f, 2)), b = 0.35f * expf(-powf((ph - 0.45f) / 0.08f, 2));
  return a + b;
}

struct Result { int bpm, spo2; };

// Simulates 30 s; R sets the true SpO2 (R=0.593 -> 97% on Maxim's curve).
static Result run(float trueBpm, float R, float noise, bool motion, unsigned seed) {
  srand(seed);
  PulseDetector pd;
  float ph = 0, irDC = 110000, redDC = 90000, amp = 0.004f;  // AC is 0.4% of DC
  for (int n = 0; n < 3000; n++) {
    uint32_t t = 10 * n;
    float rate = trueBpm * (1 + 0.03f * sinf(n * 0.0021f));  // breathing-related variation
    ph += rate / 60.0f / 100.0f;
    if (ph >= 1) ph -= 1;
    float p = pulseShape(ph);
    float drift = 1 + 0.01f * sinf(n * 0.004f);  // slow change in finger pressure
    float ir = irDC * drift * (1 - amp * p) + noise * gauss();
    float red = redDC * drift * (1 - amp * R * p) + noise * gauss();
    if (motion && (n % 700) > 600 && (n % 700) < 640) {  // a 0.4 s jolt every 7 s
      ir += 2500 * sinf(n * 0.9f);
      red += 2000 * sinf(n * 0.9f);
    }
    pd.update((uint32_t)ir, (uint32_t)red, t);
  }
  return {pd.bpm, pd.spo2};
}

int main() {
  int fails = 0;
  // tol: allowed SpO2 error in percentage points. A very noisy signal (3x a typical sensor)
  // reads slightly high, because noise in the infrared signal shrinks the red/IR ratio.
  struct Case { float bpm, R, noise; bool motion; const char *name; int tol; } cases[] = {
      {72, 0.593f, 40, false, "resting 72", 2},   {55, 0.593f, 40, false, "slow 55", 2},
      {100, 0.593f, 40, false, "100", 2},         {140, 0.593f, 40, false, "exercise 140", 2},
      {72, 0.593f, 120, false, "noisy 72", 3},    {80, 0.593f, 60, true, "80 with movement", 2},
      {72, 0.75f, 40, false, "72, lower SpO2", 2},
  };
  for (auto &c : cases) {
    float expectSpo2 = -45.060f * c.R * c.R + 30.354f * c.R + 94.845f;
    for (unsigned seed = 1; seed <= 5; seed++) {
      Result r = run(c.bpm, c.R, c.noise, c.motion, seed);
      bool ok = abs(r.bpm - (int)c.bpm) <= 3 && abs(r.spo2 - (int)(expectSpo2 + 0.5f)) <= c.tol;
      if (!ok) fails++;
      if (seed == 1 || !ok)
        printf("%-18s true %3.0f bpm %3.0f%% -> measured %3d bpm %3d%%  %s\n", c.name, c.bpm, expectSpo2, r.bpm, r.spo2, ok ? "ok" : "FAIL");
    }
  }
  // No finger: nothing reported.
  PulseDetector pd;
  for (int n = 0; n < 1000; n++) pd.update(3000, 2500, 10 * n);
  if (pd.bpm || pd.spo2 || pd.finger) fails++, printf("no finger: FAIL\n");
  else printf("no finger          -> nothing reported  ok\n");
  printf(fails ? "%d FAILED\n" : "all pulse tests passed\n", fails);
  return fails != 0;
}
