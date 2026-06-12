#include "configuration.h"
#if defined(ARCH_ESP32) && defined(SLM_ENABLED)

#include "SoundLevelModule.h"
#include "MeshService.h"
#include "airtime.h"
#include "main.h"
#include "mesh/generated/meshtastic/portnums.pb.h"

// ESP-IDF I2S PDM-RX driver (the Arduino ESP_I2S wrapper isn't in Meshtastic's
// trimmed framework include path; the IDF driver always is).
#include "driver/i2s_pdm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "octave_bank.h"   // the DSP engine, copied verbatim from the SLM repo

#if HAS_SCREEN
#include "graphics/Screen.h"
#include "graphics/ScreenFonts.h"
#include "graphics/SharedUIDisplay.h"
#endif

// ---------------------------------------------------------------------------
// Build-time configuration (override in the variant's platformio.ini build_flags)
// ---------------------------------------------------------------------------

// External PDM mic pins. NOTE: on the seeed_xiao_s3 variant Meshtastic's I2C bus
// (variant.h I2C_SDA=5 / I2C_SCL=6 — the QMI8658 IMU and the OLED live there, NOT
// the Arduino SDA/SCL=47/48 in pins_arduino.h) is on GPIO5/6, the SX1262 radio owns
// 7,8,9 / 38,39,40 / 41,42, and GPS is on 43/44 (with GPIO1 = GPS standby). The only
// free header pins are D1/D2/D3 (GPIO2/3/4), so the mic defaults to D1/D2 (GPIO2/3).
// Do NOT use D4/D5 (GPIO5/6): that collides with the I2C bus and corrupts the IMU/OLED.
#ifndef SLM_PDM_CLK_PIN
#define SLM_PDM_CLK_PIN 2
#endif
#ifndef SLM_PDM_DATA_PIN
#define SLM_PDM_DATA_PIN 3
#endif

// Base accumulation interval (ms) the audio task folds into the FIFO. The real
// averaging window is a whole multiple of this (quantization of the window).
#ifndef SLM_BASE_INTERVAL_MS
#define SLM_BASE_INTERVAL_MS 1000
#endif

// Minimum averaging window / transmit spacing (s). Bounds the packet rate.
#ifndef SLM_TMIN_S
#define SLM_TMIN_S 15
#endif

// Self-imposed duty-cycle cap (% of the last hour we allow OURSELVES to transmit),
// applied on top of Meshtastic's own gates. Meshtastic's isTxAllowedAirUtil() already
// enforces a "polite" cap of effectiveDutyCycle * polite_duty_cycle_percent / 100
// (e.g. 10% * 50% = 5% on EU_868) against utilizationTXPercent() — a tally SHARED
// across all of this node's traffic (text, position, telemetry, routing, SLM, ...).
// Without our own cap, SLM could consume that entire shared 5% budget by itself and
// starve normal mesh traffic. SLM_MAX_DUTY_PCT reserves a fair share of it for SLM,
// leaving the rest for everything else.
#ifndef SLM_MAX_DUTY_PCT
#define SLM_MAX_DUTY_PCT 2.0f
#endif

// Which core the audio capture/DSP task is pinned to, and its priority. Core 0 is
// chosen so the DSP never contends with Meshtastic's cooperative loop on the
// Arduino core. If THIS node also runs WiFi (e.g. it's the MQTT gateway too),
// revisit — WiFi lives on core 0. The task blocks on the I2S DMA read, so it
// yields the core whenever no audio is pending.
#ifndef SLM_AUDIO_CORE
#define SLM_AUDIO_CORE 0
#endif
#ifndef SLM_AUDIO_PRIO
#define SLM_AUDIO_PRIO 5
#endif

// FIFO depth in base-interval snapshots. runOnce drains every base interval, so a
// couple of slots is plenty; the extra are margin against a late drain.
#ifndef SLM_SNAP_QUEUE_DEPTH
#define SLM_SNAP_QUEUE_DEPTH 8
#endif

SoundLevelModule *soundLevelModule;

// ---------------------------------------------------------------------------
// Producer side (audio task) — sole owner of `bank`. Mirrors the standalone SLM
// firmware: capture -> process -> once per base interval, hand the per-band energy
// totals BY VALUE to the consumer through a FreeRTOS queue. No shared accumulator.
// ---------------------------------------------------------------------------

namespace
{
i2s_chan_handle_t s_rxChan = nullptr;
OctaveBank s_bank;
int16_t s_block[OCT_BLOCK];
QueueHandle_t s_snapQ;

// Base interval expressed in whole blocks so it is exact in samples.
const uint32_t kBaseIntervalBlocks =
    (uint32_t)((SLM_BASE_INTERVAL_MS * (double)OCT_FS0 / 1000.0) / OCT_BLOCK + 0.5);

// One base interval's per-band energy totals, carried by value through the queue.
struct BaseSnap {
    double snap[OCT_NUM_BANDS];   // per-band Sigma y^2 for the base interval
    uint32_t blocks;              // blocks accumulated this base interval
#ifdef SLM_AUDIO_DIAG
    double rawSumSq;              // raw (pre-filter) Sigma x^2 over the interval, x in [-1,1)
    uint32_t rawN;               // raw samples summed (= blocks * OCT_BLOCK)
#endif
};

// Consumer-owned running accumulator (the averaging window). Energy is additive, so
// summing successive base snapshots == accumulating over the whole window. O(1) RAM.
double s_accE[OCT_NUM_BANDS];
uint32_t s_accBlocks;

#ifdef SLM_AUDIO_DIAG
// Capture-integrity diagnostics (enable with -DSLM_AUDIO_DIAG). Mirrors the standalone
// SLM test firmware's AUDIO_DIAG: count I2S DMA receive-queue overflows (= samples
// physically lost, the prime suspect when the level reads high then self-recovers under
// load) and a producer heartbeat so the consumer can print the real capture rate.
volatile uint32_t g_ovf = 0;   // DMA overflow events (ISR-incremented)
uint32_t g_blocks = 0;         // cumulative blocks processed (producer heartbeat)
uint32_t g_startMs = 0;        // millis() at the first block (heartbeat anchor)
double s_rawSumSq = 0;         // raw Sigma x^2 for the in-progress base interval (producer-only)
uint32_t s_rawN = 0;           // raw samples in the in-progress base interval (producer-only)

bool IRAM_ATTR onI2sOvf(i2s_chan_handle_t, i2s_event_data_t *, void *)
{
    g_ovf++;
    return false;   // no higher-priority task to wake
}
#endif

bool readBlock()
{
    size_t got = 0;
    const size_t want = sizeof(s_block);
    while (got < want) {
        size_t n = 0;
        if (i2s_channel_read(s_rxChan, (uint8_t *)s_block + got, want - got, &n, portMAX_DELAY) != ESP_OK || n == 0)
            return false;
        got += n;
    }
    return true;
}

void audioTask(void *)
{
    for (;;) {
        if (!readBlock()) {
            vTaskDelay(1);
            continue;
        }
#ifdef SLM_AUDIO_DIAG
        if (g_blocks == 0)
            g_startMs = millis();   // anchor the heartbeat at the first block
        g_blocks++;
        for (int i = 0; i < OCT_BLOCK; ++i) {
            double x = (double)s_block[i] * (1.0 / 32768.0);
            s_rawSumSq += x * x;
        }
        s_rawN += OCT_BLOCK;
#endif
        s_bank.process(s_block);
        if (s_bank.blocks() >= kBaseIntervalBlocks) {
            BaseSnap m;
            s_bank.takeInterval(m.snap, &m.blocks);   // copy out by value, reset bank
#ifdef SLM_AUDIO_DIAG
            m.rawSumSq = s_rawSumSq;
            m.rawN = s_rawN;
            s_rawSumSq = 0;
            s_rawN = 0;
#endif
            // Non-blocking send; the consumer drains every base interval so the FIFO
            // should never fill. If it ever did, dropping the oldest base interval
            // loses a sliver of energy but can never stall capture.
            if (xQueueSend(s_snapQ, &m, 0) != pdTRUE) {
                BaseSnap drop;
                xQueueReceive(s_snapQ, &drop, 0);
                xQueueSend(s_snapQ, &m, 0);
            }
        }
    }
}

// Quantize a dB value to 0.5 dB/LSB in a uint8 (range 0..127.5 dB).
uint8_t dbToU8(float db)
{
    int v = (int)lroundf(db * 2.0f);
    if (v < 0)
        v = 0;
    if (v > 255)
        v = 255;
    return (uint8_t)v;
}

// Tear down whatever startCapture() managed to bring up, in reverse order. Safe to
// call at any partial-init point: each step is guarded by its own handle.
// `enabled` says whether the PDM RX channel was already i2s_channel_enable()d.
void teardownCapture(bool enabled)
{
    if (s_snapQ) {
        vQueueDelete(s_snapQ);
        s_snapQ = nullptr;
    }
    if (s_rxChan) {
        if (enabled)
            i2s_channel_disable(s_rxChan);
        i2s_del_channel(s_rxChan);
        s_rxChan = nullptr;
    }
}
} // namespace

// ---------------------------------------------------------------------------
// Module
// ---------------------------------------------------------------------------

SoundLevelModule::SoundLevelModule()
    : SinglePortModule("soundlevel", meshtastic_PortNum_PRIVATE_APP), OSThread("SoundLevel")
{
}

bool SoundLevelModule::startCapture()
{
    i2s_chan_config_t chanCfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    if (i2s_new_channel(&chanCfg, nullptr, &s_rxChan) != ESP_OK) {
        LOG_ERROR("SoundLevel: i2s_new_channel failed");
        return false;
    }
    i2s_pdm_rx_config_t pdmCfg = {
        .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG((uint32_t)OCT_FS0),
        .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .clk = (gpio_num_t)SLM_PDM_CLK_PIN,
            .din = (gpio_num_t)SLM_PDM_DATA_PIN,
            .invert_flags = {.clk_inv = false},
        },
    };
    // If init succeeds but enable fails, the channel exists but is not enabled;
    // pass enabled=false so teardown only deletes it. If both succeed we fall
    // through with the channel enabled.
    if (i2s_channel_init_pdm_rx_mode(s_rxChan, &pdmCfg) != ESP_OK) {
        LOG_ERROR("SoundLevel: I2S PDM init failed (clk=%d data=%d)", SLM_PDM_CLK_PIN, SLM_PDM_DATA_PIN);
        teardownCapture(false);
        return false;
    }
#ifdef SLM_AUDIO_DIAG
    // Must register while the channel is in READY (not RUNNING), i.e. before enable.
    {
        i2s_event_callbacks_t cbs = {};
        cbs.on_recv_q_ovf = onI2sOvf;
        if (i2s_channel_register_event_callback(s_rxChan, &cbs, nullptr) != ESP_OK)
            LOG_WARN("SoundLevel: DIAG overflow callback registration failed");
    }
#endif
    if (i2s_channel_enable(s_rxChan) != ESP_OK) {
        LOG_ERROR("SoundLevel: I2S PDM init failed (clk=%d data=%d)", SLM_PDM_CLK_PIN, SLM_PDM_DATA_PIN);
        teardownCapture(false);
        return false;
    }
    s_bank.begin();
    memset(s_accE, 0, sizeof(s_accE));
    s_accBlocks = 0;

    s_snapQ = xQueueCreate(SLM_SNAP_QUEUE_DEPTH, sizeof(BaseSnap));
    if (!s_snapQ) {
        LOG_ERROR("SoundLevel: queue alloc failed");
        teardownCapture(true);
        return false;
    }
    if (xTaskCreatePinnedToCore(audioTask, "slmAudio", 4096, nullptr, SLM_AUDIO_PRIO, nullptr, SLM_AUDIO_CORE) != pdPASS) {
        LOG_ERROR("SoundLevel: audio task create failed");
        teardownCapture(true);
        return false;
    }
    LOG_INFO("SoundLevel: capturing @%.0f Hz, base %lu ms, Tmin %ds, self-duty %.2f%%", (double)OCT_FS0,
             (unsigned long)SLM_BASE_INTERVAL_MS, (int)SLM_TMIN_S, (double)SLM_MAX_DUTY_PCT);
#if HAS_SCREEN
    // Mic is live: advertise the meter frame and ask the screen to rebuild its frameset.
    captureOk = true;
    UIFrameEvent e;
    e.action = UIFrameEvent::Action::REGENERATE_FRAMESET;
    notifyObservers(&e);
#endif
    return true;
}

void SoundLevelModule::drainBaseIntervals()
{
    BaseSnap m;
    bool gotOne = false;
    while (s_snapQ && xQueueReceive(s_snapQ, &m, 0) == pdTRUE) {
        for (int b = 0; b < OCT_NUM_BANDS; ++b)
            s_accE[b] += m.snap[b];
        s_accBlocks += m.blocks;
        gotOne = true;
#ifdef SLM_AUDIO_DIAG
        // Per-interval capture report. `raw` is the pre-filter broadband level in dBFS
        // (directly comparable to the test firmware's "# MIC PROBE: ... dBFS"); a jump in
        // `ovf` or `hz` sagging below 48000 is the DMA-starvation signature we're hunting.
        double rawMs = m.rawN ? m.rawSumSq / m.rawN : 0.0;
        float rawDbfs = 10.0f * log10f((float)rawMs + 1e-20f);   // = 20*log10(rms), rms in FS units
        float la = OctaveBank::dBAeq(m.snap, m.blocks);
        uint32_t elapsed = millis() - g_startMs + 1;
        float hz = (float)((double)g_blocks * OCT_BLOCK * 1000.0 / elapsed);
        LOG_INFO("SLM DIAG: raw %.1f dBFS | LAeq %.1f | ovf:%u hz:%.0f", (double)rawDbfs, (double)la,
                 (unsigned)g_ovf, (double)hz);
#endif
    }
#if HAS_SCREEN
    // Keep the most recent ~1 s interval for the live on-device meter (the long TX
    // window keeps growing in s_accE, but the screen wants a fresh, short reading).
    if (gotOne) {
        memcpy(dispE, m.snap, sizeof(dispE));
        dispBlocks = m.blocks;
    }
#else
    (void)gotOne;
#endif
}

bool SoundLevelModule::dutyAllows()
{
    if (!airTime)
        return false;
    // Meshtastic's own polite gates (region duty cycle + channel utilization), then our
    // own fair-share cap measured against the firmware's TX airtime tally for the hour.
    return airTime->isTxAllowedAirUtil() && airTime->isTxAllowedChannelUtil(true) &&
           airTime->utilizationTXPercent() < SLM_MAX_DUTY_PCT;
}

void SoundLevelModule::sendSpectrum()
{
    // Exact averaging window from the accumulated block count (sample-accurate).
    const double windowS = (double)s_accBlocks * OCT_BLOCK / OCT_FS0;
    uint16_t windowU16 = windowS >= 65535.0 ? 65535 : (uint16_t)lround(windowS);

    // Wire format v2 (35 bytes = 4 + OCT_NUM_BANDS): the band centers are the fixed
    // IEC base-10 set, so only the levels travel. The MQTT bridge knows the band
    // table and re-derives A/C-weighted broadband levels from the bands, so no
    // synthesized LAeq/LCeq is shipped (avoids the double pow() weighting here).
    //   [0]      version (0x02)
    //   [1]      nBands (OCT_NUM_BANDS = 31)
    //   [2..3]   window_seconds, uint16 little-endian
    //   [4..34]  per-band Leq, uint8 each, 0.5 dB/LSB (OCT_NUM_BANDS entries)
    uint8_t buf[4 + OCT_NUM_BANDS];
    buf[0] = 0x02;
    buf[1] = (uint8_t)OCT_NUM_BANDS;
    buf[2] = (uint8_t)(windowU16 & 0xFF);
    buf[3] = (uint8_t)(windowU16 >> 8);
    for (int b = 0; b < OCT_NUM_BANDS; ++b)
        buf[4 + b] = dbToU8(OctaveBank::bandLeqDb(s_accE, s_accBlocks, b));

    meshtastic_MeshPacket *p = allocDataPacket();   // stamps our port, to=BROADCAST
    p->to = NODENUM_BROADCAST;
    p->want_ack = false;
    p->priority = meshtastic_MeshPacket_Priority_BACKGROUND;
    p->decoded.payload.size = sizeof(buf);
    memcpy(p->decoded.payload.bytes, buf, sizeof(buf));

    LOG_INFO("SoundLevel: TX %us window, 31-band spectrum (1kHz=%.1f dB)", windowU16,
             (double)OctaveBank::bandLeqDb(s_accE, s_accBlocks, 17));
    service->sendToMesh(p);

    // Reset the window.
    memset(s_accE, 0, sizeof(s_accE));
    s_accBlocks = 0;
    windowStartMs = millis();
}

int32_t SoundLevelModule::runOnce()
{
    if (!started) {
        if (!startCapture())
            return disable();   // bail out cleanly if the mic/queue can't come up
        started = true;
        windowStartMs = millis();
        return SLM_BASE_INTERVAL_MS;
    }

    drainBaseIntervals();   // always fold pending base intervals into the window

    const bool windowReady = (millis() - windowStartMs) >= (uint32_t)SLM_TMIN_S * 1000;
    if (windowReady && s_accBlocks > 0 && dutyAllows())
        sendSpectrum();

    // Re-poll every base interval: cheap, keeps the FIFO drained, and lets the window
    // simply grow while the duty-cycle gate holds us silent (no energy is lost).
    return SLM_BASE_INTERVAL_MS;
}

#if HAS_SCREEN
// ---------------------------------------------------------------------------
// On-device UI: a live dBA bar meter plus the 31-band 1/3-octave spectrum,
// computed from the most recent ~1 s base interval (dispE/dispBlocks). Runs on
// the main loop thread, same as runOnce(), so the display copy needs no lock.
// ---------------------------------------------------------------------------
void SoundLevelModule::drawFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y)
{
    display->clear();
    display->setFont(FONT_SMALL);
    display->setTextAlignment(TEXT_ALIGN_LEFT);

    const char *titleStr = (graphics::currentResolution == graphics::ScreenResolution::High) ? "Sound Level" : "SLM";
    graphics::drawCommonHeader(display, x, y, titleStr);

    const int w = SCREEN_WIDTH;
    const int h = SCREEN_HEIGHT;
    const int line1 = graphics::getTextPositions(display)[1];

    if (dispBlocks == 0) {
        display->drawString(x, line1, "Warming up...");
        return;
    }

    // Shared dB->screen mapping for the meter and the spectrum bars.
    const float dbFloor = 20.0f, dbCeil = 100.0f;
    auto frac = [&](float db) -> float {
        float f = (db - dbFloor) / (dbCeil - dbFloor);
        return f < 0.0f ? 0.0f : (f > 1.0f ? 1.0f : f);
    };

    // --- top line: broadband dBA (left) + loudest band (right) ---
    const float dBA = OctaveBank::dBAeq(dispE, dispBlocks);
    char buf[24];
    snprintf(buf, sizeof(buf), "%.1f dBA", (double)dBA);
    display->drawString(x, line1, buf);

    int peak = 0;
    float peakDb = -1e9f;
    for (int b = 0; b < OCT_NUM_BANDS; ++b) {
        float db = OctaveBank::bandLeqDb(dispE, dispBlocks, b);
        if (db > peakDb) {
            peakDb = db;
            peak = b;
        }
    }
    const float peakHz = OctaveBank::nominalHz(peak);
    char pbuf[24];
    if (peakHz >= 1000.0f)
        snprintf(pbuf, sizeof(pbuf), "%.1fk", (double)(peakHz / 1000.0f));
    else
        snprintf(pbuf, sizeof(pbuf), "%dHz", (int)lroundf(peakHz));
    display->setTextAlignment(TEXT_ALIGN_RIGHT);
    display->drawString(w, line1, pbuf);

    // --- top line (center): ETA to the next broadcast. Two gates have predictable
    // timing and we show whichever is further out: the SLM_TMIN_S averaging window
    // (seconds), and the duty-cycle TX-percent budget freeing up (whole minutes, via
    // AirTime::getSilentMinutes — the same estimate Router uses for "send again in N
    // mins"). The remaining gate, channel utilization, depends on other radios' traffic
    // and is not predictable, so once both known gates clear we show "TX". ---
    const uint32_t windowMs = millis() - windowStartMs;
    const uint32_t tminMs = (uint32_t)SLM_TMIN_S * 1000;
    long etaS = (windowMs >= tminMs) ? 0 : (long)((tminMs - windowMs + 999) / 1000);

    if (airTime) {
        // Binding TX-percent threshold = stricter of Meshtastic's polite air-util cap
        // (effectiveDutyCycle * polite_duty_cycle_percent[=50] / 100) and our own
        // SLM_MAX_DUTY_PCT. Mirrors dutyAllows() / isTxAllowedAirUtil().
        const float effDuty = getEffectiveDutyCycle();
        const float meshCap = (!config.lora.override_duty_cycle && effDuty < 100.0f) ? effDuty * 0.5f : 100.0f;
        const float cap = meshCap < SLM_MAX_DUTY_PCT ? meshCap : SLM_MAX_DUTY_PCT;
        const long dutyS = (long)airTime->getSilentMinutes(airTime->utilizationTXPercent(), cap) * 60;
        if (dutyS > etaS)
            etaS = dutyS;
    }

    char cbuf[16];
    if (etaS <= 0)
        snprintf(cbuf, sizeof(cbuf), "TX");
    else if (etaS >= 60)
        snprintf(cbuf, sizeof(cbuf), "%ldm", (etaS + 59) / 60); // duty-limited: whole minutes
    else
        snprintf(cbuf, sizeof(cbuf), "%lds", etaS);
    display->setTextAlignment(TEXT_ALIGN_CENTER);
    display->drawString(w / 2, line1, cbuf);
    display->setTextAlignment(TEXT_ALIGN_LEFT);

    // --- 1/3-octave spectrum (with a reserved row of frequency anchor labels) ---
    const int specTop = line1 + FONT_HEIGHT_SMALL + 2;
    const int labelH = FONT_HEIGHT_SMALL;
    const int specBottom = h - 1 - labelH; // baseline; labels live below it
    const int specH = specBottom - specTop;
    if (specH < 4)
        return; // no vertical room left (tiny display)

    const int n = OCT_NUM_BANDS;
    int barW = w / n;
    if (barW < 1)
        barW = 1;
    const int gap = (barW >= 3) ? 1 : 0;
    display->drawLine(x, specBottom, x + barW * n, specBottom); // spectrum baseline
    for (int b = 0; b < n; ++b) {
        float db = OctaveBank::bandLeqDb(dispE, dispBlocks, b);
        int bh = (int)(specH * frac(db));
        if (bh > 0)
            display->fillRect(x + b * barW, specBottom - bh, barW - gap, bh);
    }

    // Frequency anchors: a tick + label centered under the band's bar. Indices 7/17/27
    // are the 100 Hz / 1 kHz / 10 kHz bands of the fixed IEC 1/3-octave table.
    struct Anchor {
        int band;
        const char *label;
    };
    static const Anchor anchors[] = {{7, "100"}, {17, "1k"}, {27, "10k"}};
    display->setTextAlignment(TEXT_ALIGN_CENTER);
    const int labelY = specBottom + 1;
    for (const Anchor &a : anchors) {
        int cx = x + a.band * barW + barW / 2;
        display->drawLine(cx, specBottom - 1, cx, specBottom + 1); // tick
        int half = display->getStringWidth(a.label) / 2;
        if (cx - half < x)
            cx = x + half;
        else if (cx + half > w)
            cx = w - half;
        display->drawString(cx, labelY, a.label);
    }
    display->setTextAlignment(TEXT_ALIGN_LEFT);
}
#endif // HAS_SCREEN

#endif // ARCH_ESP32 && SLM_ENABLED
