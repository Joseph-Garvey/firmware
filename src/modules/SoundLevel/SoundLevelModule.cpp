#include "configuration.h"
#if defined(ARCH_ESP32) && defined(SLM_ENABLED)

#include "SoundLevelModule.h"
#include "MeshService.h"
#include "airtime.h"
#include "main.h"
#include "mesh/generated/meshtastic/portnums.pb.h"
#ifdef SLM_FOH_STREAM
#include "PowerFSM.h"   // powerFSM / stateSERIAL — companion-connection gate
#endif

// ESP-IDF I2S PDM-RX driver (the Arduino ESP_I2S wrapper isn't in Meshtastic's
// trimmed framework include path; the IDF driver always is).
#include "driver/i2s_pdm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "octave_bank.h"   // the DSP engine, copied verbatim from the SLM repo

#ifdef SLM_FS_LOG
#include "FSCommon.h"   // LittleFS handle (FSCom) + renameFile()
#include "gps/RTC.h"    // getTime() for a wall-clock stamp when RTC is valid
#endif

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

// I2S DMA ring depth. More headroom = the audio task can be starved longer (e.g. by a
// WiFi burst on a shared core) before the DMA queue overflows and a buffer is dropped.
// 8 x 512 samples ~= 85 ms of audio, vs the IDF default ~30 ms (6 x 240). Per-descriptor
// byte count (frame_num * 2 for 16-bit mono) must stay under the 4092-byte DMA limit.
#ifndef SLM_DMA_DESC_NUM
#define SLM_DMA_DESC_NUM 8
#endif
#ifndef SLM_DMA_FRAME_NUM
#define SLM_DMA_FRAME_NUM 512
#endif

// On-device anomaly log (opt-in: -DSLM_FS_LOG). Appends edge events — DMA buffer drops
// and crest/level excursions — to a LittleFS file under /static, so it is downloadable
// over the WiFi web server (GET /slm.log) on a deployed node, no tethered host needed.
// Only transitions + a slow heartbeat are written, so it costs kilobytes, not megabytes.
#ifndef SLM_FS_LOG_PATH
#define SLM_FS_LOG_PATH "/static/slm.log"   // served by the web server as GET /slm.log
#endif
#ifndef SLM_FS_LOG_MAX_BYTES
#define SLM_FS_LOG_MAX_BYTES 65536          // rotate to <path>.1 past this (2x = total cap)
#endif
#ifndef SLM_FS_LOG_CREST_DB
#define SLM_FS_LOG_CREST_DB 25.0f           // crest above this => impulsive-corruption flag
#endif
#ifndef SLM_FS_LOG_LAEQ_DB
#define SLM_FS_LOG_LAEQ_DB 200.0f           // LAeq above this also flags; default ~off
#endif
#ifndef SLM_FS_LOG_SUSTAIN_S
#define SLM_FS_LOG_SUSTAIN_S 30             // min seconds between repeats during a sustained event
#endif

// Internal: the per-interval raw/peak capture stats feed BOTH the serial DIAG line and
// the on-device anomaly log, so compute them whenever either is enabled.
#if defined(SLM_AUDIO_DIAG) || defined(SLM_FS_LOG)
#define SLM_CAPTURE_STATS 1
#endif

// Front-of-house live streaming (opt-in). When defined, the module also emits the
// most-recent base interval's spectrum to any locally connected client (BLE / USB
// serial / TCP) via MeshService::sendToPhone() — no LoRa transmission, no airtime
// cost. The refresh rate is just SLM_BASE_INTERVAL_MS: 125 ms ~= 8 Hz ("fast") or
// 1000 ms = 1 Hz ("slow"), matching the classic SLM time weightings. The periodic
// LoRa broadcast (sendSpectrum) is unaffected — it stays the same long-window frame
// every node sends. Define in the variant's build_flags to enable.
//   -DSLM_FOH_STREAM

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
#ifdef SLM_CAPTURE_STATS
    double rawSumSq;              // raw (pre-filter) Sigma x^2 over the interval, x in [-1,1)
    uint32_t rawN;               // raw samples summed (= blocks * OCT_BLOCK)
    int rawPeak;                 // max |sample| this interval (0..32768) -> crest factor
#endif
};

// Consumer-owned running accumulator (the averaging window). Energy is additive, so
// summing successive base snapshots == accumulating over the whole window. O(1) RAM.
double s_accE[OCT_NUM_BANDS];
uint32_t s_accBlocks;

// I2S DMA receive-queue overflow counter (ISR-incremented). ALWAYS compiled: the audio
// task compares it across each base interval and discards any interval during which a
// buffer was dropped, so the resulting discontinuity can't ring the filter bank and
// spike the reading. (Also surfaced by the SLM_AUDIO_DIAG log line.)
volatile uint32_t g_ovf = 0;
bool IRAM_ATTR onI2sOvf(i2s_chan_handle_t, i2s_event_data_t *, void *)
{
    g_ovf++;
    return false;   // no higher-priority task to wake
}

#ifdef SLM_AUDIO_DIAG
// Producer heartbeat, so the consumer's DIAG line can print the real capture rate (hz).
uint32_t g_blocks = 0;         // cumulative blocks processed
uint32_t g_startMs = 0;        // millis() at the first block (heartbeat anchor)
#endif
#ifdef SLM_CAPTURE_STATS
// Raw (pre-filter) broadband level + peak for the in-progress base interval. Feeds the
// crest factor used by both the DIAG line and the on-device anomaly log. Producer-only.
double s_rawSumSq = 0;
uint32_t s_rawN = 0;
int s_rawPeak = 0;
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
    uint32_t intervalStartOvf = g_ovf;   // overflow count at the start of the current interval
    for (;;) {
        if (!readBlock()) {
            vTaskDelay(1);
            continue;
        }
#ifdef SLM_AUDIO_DIAG
        if (g_blocks == 0)
            g_startMs = millis();   // anchor the heartbeat at the first block
        g_blocks++;
#endif
#ifdef SLM_CAPTURE_STATS
        for (int i = 0; i < OCT_BLOCK; ++i) {
            double x = (double)s_block[i] * (1.0 / 32768.0);
            s_rawSumSq += x * x;
            int a = abs((int)s_block[i]);
            if (a > s_rawPeak)
                s_rawPeak = a;
        }
        s_rawN += OCT_BLOCK;
#endif
        s_bank.process(s_block);
        if (s_bank.blocks() >= kBaseIntervalBlocks) {
            if (g_ovf != intervalStartOvf) {
                // A DMA buffer was dropped during this interval: the discontinuity has
                // rung the filter bank. Discard the interval and clear the filter state
                // (begin() resets delay lines + accumulator) so the glitch can't bleed
                // into the next interval — the meter skips a beat instead of spiking.
                s_bank.begin();
#ifdef SLM_CAPTURE_STATS
                s_rawSumSq = 0;
                s_rawN = 0;
                s_rawPeak = 0;
#endif
            } else {
                BaseSnap m;
                s_bank.takeInterval(m.snap, &m.blocks);   // copy out by value, reset bank
#ifdef SLM_CAPTURE_STATS
                m.rawSumSq = s_rawSumSq;
                m.rawN = s_rawN;
                m.rawPeak = s_rawPeak;
                s_rawSumSq = 0;
                s_rawN = 0;
                s_rawPeak = 0;
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
            intervalStartOvf = g_ovf;
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

// Build the v2 wire frame (4 + OCT_NUM_BANDS bytes) from an energy accumulator and
// its block count. Shared by the LoRa broadcast (long window) and the FoH live stream
// (single base interval) so both speak the identical format — see the wire-format
// section in README.md. Returns the window length quantized to whole seconds (the
// value written into bytes [2..3]); for a sub-second live frame this rounds toward 0.
uint16_t buildV2Frame(uint8_t *buf, const double *energy, uint32_t blocks)
{
    const double windowS = (double)blocks * OCT_BLOCK / OCT_FS0;
    const uint16_t windowU16 = windowS >= 65535.0 ? 65535 : (uint16_t)lround(windowS);
    buf[0] = 0x02;
    buf[1] = (uint8_t)OCT_NUM_BANDS;
    buf[2] = (uint8_t)(windowU16 & 0xFF);
    buf[3] = (uint8_t)(windowU16 >> 8);
    for (int b = 0; b < OCT_NUM_BANDS; ++b)
        buf[4 + b] = dbToU8(OctaveBank::bandLeqDb(energy, blocks, b));
    return windowU16;
}

#ifdef SLM_FOH_STREAM
// Is a local client (the FoH companion) attached to this node's client API? Gates the
// high-rate stream so we don't pile frames into a toPhoneQueue nobody is draining (which
// would otherwise log a drop warning every base interval). Covers BLE and USB serial;
// a TCP/WiFi client isn't reflected here, so FoH over TCP would need that case added.
bool companionConnected()
{
#if defined(ARCH_ESP32) && !defined(CONFIG_IDF_TARGET_ESP32S2)
    if (nimbleBluetooth && nimbleBluetooth->isConnected())
        return true;
#endif
#if !MESHTASTIC_EXCLUDE_POWER_FSM
    if (powerFSM.getState() == &stateSERIAL)
        return true;
#endif
    return false;
}
#endif

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
    chanCfg.dma_desc_num = SLM_DMA_DESC_NUM;     // deeper ring => more starvation headroom
    chanCfg.dma_frame_num = SLM_DMA_FRAME_NUM;
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
    // Register the DMA-overflow callback so the audio task can drop glitched intervals
    // (and the DIAG build can log overflows). Must be registered while the channel is in
    // READY (not RUNNING), i.e. before enable.
    {
        i2s_event_callbacks_t cbs = {};
        cbs.on_recv_q_ovf = onI2sOvf;
        if (i2s_channel_register_event_callback(s_rxChan, &cbs, nullptr) != ESP_OK)
            LOG_WARN("SoundLevel: overflow callback registration failed");
    }
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

#ifdef SLM_FS_LOG
// Append one CSV line to the LittleFS anomaly log, rotating to <path>.1 past the size cap
// so the file can never grow without bound. Runs on the main loop (same thread as the rest
// of runOnce), so the flash write can't race the audio task. Columns:
//   epoch,uptime_s,tag,LAeq,crest,raw,ovf
// `epoch` is 0 until the RTC has a valid time; `uptime_s` always lines up with the log.
void SoundLevelModule::fsAppend(const char *tag, float la, float crest, float rawDbfs, uint32_t ovf)
{
    // LittleFS won't create a file inside a missing directory (and /static only exists if
    // web-UI content was uploaded), so ensure the parent directory first. One level deep.
    {
        char dir[64];
        strncpy(dir, SLM_FS_LOG_PATH, sizeof(dir) - 1);
        dir[sizeof(dir) - 1] = '\0';
        char *slash = strrchr(dir, '/');
        if (slash && slash != dir) {
            *slash = '\0';
            if (!FSCom.exists(dir))
                FSCom.mkdir(dir);
        }
    }
    if (FSCom.exists(SLM_FS_LOG_PATH)) {
        File rf = FSCom.open(SLM_FS_LOG_PATH, FILE_O_READ);
        const size_t sz = rf ? rf.size() : 0;
        if (rf)
            rf.close();
        if (sz >= SLM_FS_LOG_MAX_BYTES) {
            FSCom.remove(SLM_FS_LOG_PATH ".1");
            renameFile(SLM_FS_LOG_PATH, SLM_FS_LOG_PATH ".1");
        }
    }
    File f = FSCom.open(SLM_FS_LOG_PATH, "a");   // ESP32 LittleFS: append (creates if absent)
    if (!f) {
        LOG_WARN("SoundLevel: FS log open failed (%s)", SLM_FS_LOG_PATH);
        return;
    }
    f.printf("%lu,%lu,%s,%.1f,%.1f,%.1f,%lu\n", (unsigned long)getTime(), (unsigned long)(millis() / 1000),
             tag, (double)la, (double)crest, (double)rawDbfs, (unsigned long)ovf);
    const size_t newSize = f.position();   // append leaves us at EOF => total file size
    f.close();
    LOG_INFO("SoundLevel: FS log += %s (%s now %u bytes)", tag, SLM_FS_LOG_PATH, (unsigned)newSize);
}
#endif

bool SoundLevelModule::drainBaseIntervals()
{
    BaseSnap m;
    bool gotOne = false;
    while (s_snapQ && xQueueReceive(s_snapQ, &m, 0) == pdTRUE) {
        for (int b = 0; b < OCT_NUM_BANDS; ++b)
            s_accE[b] += m.snap[b];
        s_accBlocks += m.blocks;
        gotOne = true;
#ifdef SLM_CAPTURE_STATS
        // Per-interval raw (pre-filter) broadband level and crest factor. `raw` is dBFS
        // (comparable to the test firmware's "# MIC PROBE: ... dBFS"); `crest` = peak-raw
        // separates impulsive corruption (>~25 dB) from steady audio (~12-18 dB).
        double rawMs = m.rawN ? m.rawSumSq / m.rawN : 0.0;
        float rawDbfs = 10.0f * log10f((float)rawMs + 1e-20f);   // = 20*log10(rms), rms in FS units
        float peakDbfs = 20.0f * log10f((float)m.rawPeak / 32768.0f + 1e-9f);
        float crest = peakDbfs - rawDbfs;
        float la = OctaveBank::dBAeq(m.snap, m.blocks);
#endif
#ifdef SLM_AUDIO_DIAG
        // `up` is firmware uptime (s) to line up with the surrounding WiFi/LoRa log; `hz` is
        // the INSTANTANEOUS capture rate over just this interval (not cumulative from boot, so
        // a lossy boot doesn't drag it down forever). hz < 48000 here = audio actually lost in
        // this interval. (Intervals that hit a DMA drop are discarded upstream, so the loss
        // shows as a one-off dip, not in `ovf` alone.)
        static uint32_t s_lastHzBlocks = 0, s_lastHzMs = 0;
        const uint32_t nowMs = millis();
        const uint32_t dBlk = g_blocks - s_lastHzBlocks;
        float hz = (s_lastHzMs && nowMs > s_lastHzMs) ? (float)((double)dBlk * OCT_BLOCK * 1000.0 / (nowMs - s_lastHzMs)) : 0.0f;
        s_lastHzBlocks = g_blocks;
        s_lastHzMs = nowMs;
        LOG_INFO("SLM DIAG: up:%.1fs raw %.1f peak %.1f crest %.1f dBFS | LAeq %.1f | ovf:%u hz:%.0f",
                 millis() / 1000.0, (double)rawDbfs, (double)peakDbfs, (double)crest, (double)la,
                 (unsigned)g_ovf, (double)hz);
#endif
#ifdef SLM_FS_LOG
        // On-device anomaly log: write only the edges of a crest/level excursion (plus a
        // slow heartbeat while it persists), so a sustained event costs a few lines, not one
        // per second. Buffer drops are handled separately below (those intervals never land
        // here, since the audio task discards them).
        const bool anom = crest > SLM_FS_LOG_CREST_DB || la > SLM_FS_LOG_LAEQ_DB;
        const uint32_t now = millis();
        if (anom && !fsAnomActive) {
            fsAnomActive = true;
            fsLastWriteMs = now;
            fsAppend("EXCURSION-START", la, crest, rawDbfs, g_ovf);
        } else if (anom && (now - fsLastWriteMs) >= (uint32_t)SLM_FS_LOG_SUSTAIN_S * 1000) {
            fsLastWriteMs = now;
            fsAppend("EXCURSION", la, crest, rawDbfs, g_ovf);
        } else if (!anom && fsAnomActive) {
            fsAnomActive = false;
            fsAppend("EXCURSION-END", la, crest, rawDbfs, g_ovf);
        }
#endif
    }
#ifdef SLM_FS_LOG
    // Buffer-drop edge: g_ovf advances when the audio task discarded a glitched interval.
    // Record it (throttled), since those intervals are invisible to the loop above.
    if (g_ovf != lastFsOvf) {
        const uint32_t now = millis();
        if (now - fsDropWriteMs >= (uint32_t)SLM_FS_LOG_SUSTAIN_S * 1000) {
            fsDropWriteMs = now;
            fsAppend("DROP", 0.0f, 0.0f, 0.0f, g_ovf);
        }
        lastFsOvf = g_ovf;
    }
#endif
#if HAS_SCREEN || defined(SLM_FOH_STREAM)
    // Keep the most recent base interval for the live on-device meter and/or the FoH
    // stream (the long TX window keeps growing in s_accE, but these want a fresh, short
    // reading).
    if (gotOne) {
        memcpy(dispE, m.snap, sizeof(dispE));
        dispBlocks = m.blocks;
    }
#endif
    return gotOne;
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
    // Wire format v2 (35 bytes = 4 + OCT_NUM_BANDS): the band centers are the fixed
    // IEC base-10 set, so only the levels travel. The MQTT bridge knows the band
    // table and re-derives A/C-weighted broadband levels from the bands, so no
    // synthesized LAeq/LCeq is shipped (avoids the double pow() weighting here).
    //   [0]      version (0x02)
    //   [1]      nBands (OCT_NUM_BANDS = 31)
    //   [2..3]   window_seconds, uint16 little-endian
    //   [4..34]  per-band Leq, uint8 each, 0.5 dB/LSB (OCT_NUM_BANDS entries)
    // The window is sample-accurate from the accumulated block count.
    uint8_t buf[4 + OCT_NUM_BANDS];
    const uint16_t windowU16 = buildV2Frame(buf, s_accE, s_accBlocks);

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

#ifdef SLM_FOH_STREAM
void SoundLevelModule::sendLiveSpectrum()
{
    // Emit the most-recent base interval (dispE/dispBlocks) to the locally connected
    // client only — same v2 format and PRIVATE_APP port as the mesh frame, but routed
    // via sendToPhone() so it never hits the radio or the duty-cycle budget. Skipped
    // when no client is attached so we don't churn the toPhoneQueue.
    if (dispBlocks == 0 || !companionConnected())
        return;

    uint8_t buf[4 + OCT_NUM_BANDS];
    buildV2Frame(buf, dispE, dispBlocks);

    meshtastic_MeshPacket *p = allocDataPacket();   // stamps our port + from, to=BROADCAST
    p->to = NODENUM_BROADCAST;
    p->want_ack = false;
    p->priority = meshtastic_MeshPacket_Priority_BACKGROUND;
    p->decoded.payload.size = sizeof(buf);
    memcpy(p->decoded.payload.bytes, buf, sizeof(buf));

    service->sendToPhone(p);
}
#endif

int32_t SoundLevelModule::runOnce()
{
    if (!started) {
        if (!startCapture())
            return disable();   // bail out cleanly if the mic/queue can't come up
        started = true;
        windowStartMs = millis();
        return SLM_BASE_INTERVAL_MS;
    }

    // Always fold pending base intervals into the window. A fresh interval also feeds
    // the FoH live stream (local client only — never touches the radio/duty cycle).
#ifdef SLM_FOH_STREAM
    if (drainBaseIntervals())
        sendLiveSpectrum();
#else
    drainBaseIntervals();
#endif

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
