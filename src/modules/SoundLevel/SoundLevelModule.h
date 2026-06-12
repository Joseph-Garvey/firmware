#pragma once
#include "SinglePortModule.h"
#include "concurrency/OSThread.h"
#include "octave_bank.h"
#if HAS_SCREEN
#include <OLEDDisplay.h>
#include <OLEDDisplayUi.h>
#endif

/**
 * SoundLevelModule — a real-time 1/3-octave sound level meter that broadcasts its
 * spectrum onto the mesh (for MQTT uplink) instead of over serial.
 *
 * A high-priority FreeRTOS task captures PDM audio at 48 kHz and runs the multirate
 * 1/3-octave filter bank (OctaveBank, src/modules/SoundLevel/octave_bank.h). It folds
 * each ~1 s "base interval" of per-band energy (Sigma y^2) into a small FIFO. This
 * OSThread (runOnce) drains that FIFO into its own running accumulator — band energies
 * are additive, so the averaging window is just "everything since the last transmit",
 * at O(1) memory.
 *
 * The window therefore equals the transmit spacing, which is governed by the LoRa
 * duty cycle: we only transmit (and reset the accumulator) when AirTime says it's
 * allowed AND our own stricter self-imposed duty cap (SLM_MAX_DUTY_PCT) is satisfied.
 * Each packet carries the exact interval Leq over the real elapsed window.
 *
 * See src/modules/SoundLevel/SoundLevelModule.cpp for the wire format and the
 * duty-cycle math.
 */
class SoundLevelModule : public SinglePortModule,
#if HAS_SCREEN
                         public Observable<const UIFrameEvent *>,
#endif
                         private concurrency::OSThread
{
  public:
    SoundLevelModule();

  protected:
    virtual int32_t runOnce() override;

#if HAS_SCREEN
    // Live on-device sound-level meter: a dBA bar plus the 1/3-octave spectrum.
    virtual bool wantUIFrame() override { return captureOk; }
    virtual Observable<const UIFrameEvent *> *getUIFrameObservable() override { return this; }
    virtual void drawFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y) override;
#endif

  private:
    bool startCapture();   // one-time I2S + task bring-up (lazy, on first runOnce)
    void drainBaseIntervals();
    bool dutyAllows();
    void sendSpectrum();

    bool started = false;
    bool captureOk = false;       // mic is up; gates the UI frame
    uint32_t windowStartMs = 0;   // millis() when the current averaging window opened

#if HAS_SCREEN
    // Most-recent base interval (~1 s), kept for the live meter. Owned by the
    // consumer thread (runOnce/drawFrame both run on the main loop, no locks).
    double dispE[OCT_NUM_BANDS] = {0};
    uint32_t dispBlocks = 0;
#endif
};

extern SoundLevelModule *soundLevelModule;
