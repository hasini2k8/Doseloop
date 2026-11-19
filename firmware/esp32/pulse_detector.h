// Heart rate and SpO2 from MAX30102 red/IR samples at 100 samples a second.
//
// Finds each heartbeat as a peak in the filtered infrared signal, ignores stretches where the
// finger moves, rejects beats that do not fit the recent rhythm, and reports the median of
// recent beat-to-beat intervals. So the rate can be any value (72, 73, 74 ...) rather than
// the few steps the SparkFun/Maxim example produces. SpO2 uses the ratio of the red and
// infrared pulse sizes with Maxim's calibration curve.
//
// Plain C++ with no Arduino dependency, so it is tested on a computer (firmware/test).
#pragma once
#include <stdint.h>
#include <math.h>

struct PulseDetector {
  static const int HIST = 6;
  static constexpr float FINGER_IR = 50000;  // below this nothing is on the sensor

  // Outputs.
  int bpm = 0;    // 0 until 3 consistent beats
  int spo2 = 0;   // 0 until 3 beats with a usable red/IR ratio
  bool finger = false;
  bool moving = false;  // the last second was disturbed by movement

  // Filter state.
  float dcIr = 0, dcRed = 0;      // slow level (about 1 s), the "DC" in the SpO2 ratio
  float baseIr = 0, baseRed = 0;  // faster baseline (about 0.25 s): follows pressure changes
  float lpIr = 0, lpRed = 0, env = 0;
  uint16_t settle = 0;
  bool above = false;
  float peakVal = 0;
  uint32_t peakT = 0, lastPeakT = 0, quietFrom = 0;
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
    if (ir < FINGER_IR) {
      if (finger) reset();
      return false;
    }
    if (!finger) {  // finger just arrived: start the filters at its level
      finger = true;
      dcIr = baseIr = ir;
      dcRed = baseRed = red;
    }
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
