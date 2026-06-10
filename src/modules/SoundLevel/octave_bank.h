// Multirate 1/3-octave sound-level meter engine.
//
// Signal flow per processing block (BLOCK input samples @ 48 kHz):
//   level 0 = input                                   (BLOCK samples)
//   level d = elliptic-LP + downsample(level d-1)     (BLOCK >> d samples)
// Each band runs its 6th-order Butterworth band-pass (3x DF2T biquad) on the
// level whose local rate keeps fc/fs mid-band (OCT_BAND_DECIM[b]), then the
// per-band squared samples are summed into a per-band energy accumulator. At an
// interval boundary the host freezes the accumulator and reports per-band
// **Leq** (equivalent-continuous level over the interval) + synthesised
// LAeq/LCeq.
//
// Precision split (the S3 FPU is single-precision; double is software-emulated
// at ~26x and must never touch the 48 kHz path):
//   - hot 48 kHz loop: float  (one `acc += x*x` per sample)
//   - per-block (~187 Hz): fold the float block-sum into a `double` interval
//     accumulator -> exact over minute-scale windows, ~6k double-adds/s (free).
//
// Concurrency: process() and takeInterval() run on the producer (audio) task
// only — single owner, no locks. takeInterval() copies the interval totals OUT
// by value, so the consumer works from its own copy and never aliases live
// accumulator memory (no torn reads, no shared bank). See main.cpp.
#pragma once
#include <Arduino.h>
#include <math.h>
#include "octave_coeffs.h"

// Block of input samples processed at once. Must be a multiple of 2^OCT_MAX_DECIM
// so every decimation level yields a whole number of samples.
#define OCT_BLOCK 256
static_assert(OCT_BLOCK % (1 << OCT_MAX_DECIM) == 0, "BLOCK must divide by 2^MAX_DECIM");

// dBFS(rms) -> dB SPL offset. NOMINAL for the Infineon IM72D128VV01 (-36 dBFS @
// 94 dB SPL): 94 - (-36) = 130. *** Must be field-calibrated for real SPL. ***
// (XIAO Sense MSM261 onboard mic is ~-26 dBFS -> 120.)
#ifndef MIC_CAL_OFFSET_DB
#define MIC_CAL_OFFSET_DB 130.0f
#endif

class OctaveBank {
 public:
  void begin() {
    memset(bandState_, 0, sizeof(bandState_));
    memset(decState_, 0, sizeof(decState_));
    memset(acc_, 0, sizeof(acc_));
    blocks_ = 0;
  }

  // Process exactly OCT_BLOCK int16 samples. Accumulates per-band energy into the
  // active interval bank and bumps the block count. Producer-thread only.
  void process(const int16_t* in) {
    // level 0: normalise to [-1,1)
    float* l0 = level_[0];
    for (int i = 0; i < OCT_BLOCK; ++i) l0[i] = (float)in[i] * (1.0f / 32768.0f);

    // build decimated levels 1..MAX
    int len = OCT_BLOCK;
    for (int d = 1; d <= OCT_MAX_DECIM; ++d) {
      decimate(level_[d - 1], len, level_[d], decState_[d - 1]);
      len >>= 1;
    }

    // Per-band band-pass + sum of squares. A plain scalar loop: on the LX7 this
    // beat a band-interleaved version (which spilled FP registers) — see
    // docs/optimization_roadmap.md. The squared sum stays float in the hot loop;
    // it is folded into the double interval accumulator once per band per block.
    for (int b = 0; b < OCT_NUM_BANDS; ++b) {
      const int d = OCT_BAND_DECIM[b];
      const float* src = level_[d];
      const int n = OCT_BLOCK >> d;
      float (*st)[2] = bandState_[b];
      float bsum = 0.0f;
      for (int i = 0; i < n; ++i) {
        float x = src[i];
        // three cascaded DF2T biquads
        for (int s = 0; s < OCT_BP_SECTIONS; ++s) {
          const float* c = OCT_BAND_SOS[b][s];   // b0,b1,b2,a1,a2
          float y = c[0] * x + st[s][0];
          st[s][0] = c[1] * x - c[3] * y + st[s][1];
          st[s][1] = c[2] * x - c[4] * y;
          x = y;
        }
        bsum += x * x;
      }
      acc_[b] += (double)bsum;   // per-block fold into the wide accumulator
    }
    ++blocks_;
  }

  // ---- interval hand-off (producer thread only) -----------------------------
  uint32_t blocks() const { return blocks_; }

  // Copy this interval's per-band energy totals OUT by value, then reset for the
  // next interval. Producer-thread only. The caller owns `out`, so the consumer
  // never aliases live accumulator memory — no torn reads, no shared bank.
  void takeInterval(double out[OCT_NUM_BANDS], uint32_t* blocksOut) {
    *blocksOut = blocks_;
    memcpy(out, acc_, sizeof(acc_));
    memset(acc_, 0, sizeof(acc_));
    blocks_ = 0;
  }

  // ---- readouts (compute from a caller-owned interval copy) -----------------
  static int numBands() { return OCT_NUM_BANDS; }
  static float nominalHz(int b) { return OCT_BAND_NOMINAL[b]; }

  // Per-band interval Leq in dB SPL.
  static float bandLeqDb(const double* snap, uint32_t blocks, int b) {
    return 10.0f * log10f((float)msFromSnap(snap, blocks, b) + 1e-20f) + MIC_CAL_OFFSET_DB;
  }
  // Broadband A/C-weighted equivalent levels, synthesised from the band energies.
  static float dBAeq(const double* snap, uint32_t blocks) { return weightedEq(snap, blocks, OCT_BAND_AOFF); }
  static float dBCeq(const double* snap, uint32_t blocks) { return weightedEq(snap, blocks, OCT_BAND_COFF); }

 private:
  // Mean-square of band b over the interval = (sum of squares) / (sample count).
  // Sample count is exact: blocks * (samples this band contributes per block).
  static double msFromSnap(const double* snap, uint32_t blocks, int b) {
    if (blocks == 0) return 0.0;
    double N = (double)blocks * (double)(OCT_BLOCK >> OCT_BAND_DECIM[b]);
    return snap[b] / N;
  }
  // dB = CAL + 10log10( sum_b ms_b * 10^(off_b/10) )  (interval readout only).
  static float weightedEq(const double* snap, uint32_t blocks, const float* off) {
    double acc = 0.0;
    for (int b = 0; b < OCT_NUM_BANDS; ++b)
      acc += msFromSnap(snap, blocks, b) * pow(10.0, off[b] * 0.1);
    return 10.0f * log10f((float)acc + 1e-20f) + MIC_CAL_OFFSET_DB;
  }

  // Elliptic low-pass (OCT_DECIM_SOS) over `n` samples, keep every 2nd -> out[n/2].
  static void decimate(const float* in, int n, float* out,
                       float state[OCT_DECIM_SECTIONS][2]) {
    int o = 0;
    for (int i = 0; i < n; ++i) {
      float x = in[i];
      for (int s = 0; s < OCT_DECIM_SECTIONS; ++s) {
        const float* c = OCT_DECIM_SOS[s];
        float y = c[0] * x + state[s][0];
        state[s][0] = c[1] * x - c[3] * y + state[s][1];
        state[s][1] = c[2] * x - c[4] * y;
        x = y;
      }
      if (i & 1) out[o++] = x;   // decimate by 2
    }
  }

  float level_[OCT_MAX_DECIM + 1][OCT_BLOCK];      // per-level work buffers
  float bandState_[OCT_NUM_BANDS][OCT_BP_SECTIONS][2];
  float decState_[OCT_MAX_DECIM][OCT_DECIM_SECTIONS][2];

  double acc_[OCT_NUM_BANDS];   // per-band interval energy accumulator (producer only)
  uint32_t blocks_;             // blocks accumulated this interval (producer only)
};
