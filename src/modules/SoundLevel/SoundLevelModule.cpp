#include "SoundLevelModule.h"
#include "MeshService.h"
#include "airtime.h"
#include "configuration.h"
#include "main.h"
#include "mesh/generated/meshtastic/portnums.pb.h"

// ESP-IDF I2S PDM-RX driver (the Arduino ESP_I2S wrapper isn't in Meshtastic's
// trimmed framework include path; the IDF driver always is).
#include "driver/i2s_pdm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "octave_bank.h"   // the DSP engine, copied verbatim from the SLM repo

// ---------------------------------------------------------------------------
// Build-time configuration (override in the variant's platformio.ini build_flags)
// ---------------------------------------------------------------------------

// External PDM mic pins. NOTE: on the seeed_xiao_s3 variant the SX1262 radio
// already uses GPIO41 (CS) / 42 (RESET) / 7,8,9 (SPI) / 38,39,40, and I2C is on
// 5/6, GPS on 43/44 — so the mic must sit on otherwise-free GPIOs. Defaults below
// are D1/D2 (GPIO2/GPIO3) on the standard XIAO ESP32S3.
#ifndef SLM_PDM_CLK_PIN
#define SLM_PDM_CLK_PIN D5
#endif
#ifndef SLM_PDM_DATA_PIN
#define SLM_PDM_DATA_PIN D4
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
// applied on top of Meshtastic's region gate. Set to 1.0 to stay legal on EU868's
// 1% sub-bands (Meshtastic only models the single region-wide 10% figure and hops
// frequencies — it does not distinguish per-channel sub-bands). See header.
#ifndef SLM_MAX_DUTY_PCT
#define SLM_MAX_DUTY_PCT 1.0f
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
};

// Consumer-owned running accumulator (the averaging window). Energy is additive, so
// summing successive base snapshots == accumulating over the whole window. O(1) RAM.
double s_accE[OCT_NUM_BANDS];
uint32_t s_accBlocks;

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
        s_bank.process(s_block);
        if (s_bank.blocks() >= kBaseIntervalBlocks) {
            BaseSnap m;
            s_bank.takeInterval(m.snap, &m.blocks);   // copy out by value, reset bank
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
    if (i2s_channel_init_pdm_rx_mode(s_rxChan, &pdmCfg) != ESP_OK || i2s_channel_enable(s_rxChan) != ESP_OK) {
        LOG_ERROR("SoundLevel: I2S PDM init failed (clk=%d data=%d)", SLM_PDM_CLK_PIN, SLM_PDM_DATA_PIN);
        return false;
    }
    s_bank.begin();
    memset(s_accE, 0, sizeof(s_accE));
    s_accBlocks = 0;

    s_snapQ = xQueueCreate(SLM_SNAP_QUEUE_DEPTH, sizeof(BaseSnap));
    if (!s_snapQ) {
        LOG_ERROR("SoundLevel: queue alloc failed");
        return false;
    }
    if (xTaskCreatePinnedToCore(audioTask, "slmAudio", 4096, nullptr, SLM_AUDIO_PRIO, nullptr, SLM_AUDIO_CORE) != pdPASS) {
        LOG_ERROR("SoundLevel: audio task create failed");
        return false;
    }
    LOG_INFO("SoundLevel: capturing @%.0f Hz, base %lu ms, Tmin %ds, self-duty %.2f%%", (double)OCT_FS0,
             (unsigned long)SLM_BASE_INTERVAL_MS, (int)SLM_TMIN_S, (double)SLM_MAX_DUTY_PCT);
    return true;
}

void SoundLevelModule::drainBaseIntervals()
{
    BaseSnap m;
    while (s_snapQ && xQueueReceive(s_snapQ, &m, 0) == pdTRUE) {
        for (int b = 0; b < OCT_NUM_BANDS; ++b)
            s_accE[b] += m.snap[b];
        s_accBlocks += m.blocks;
    }
}

bool SoundLevelModule::dutyAllows()
{
    if (!airTime)
        return false;
    // Region gate (EU868 = 10% * polite) + polite channel-utilization gate, then our
    // own stricter cap measured against the firmware's TX airtime tally for the hour.
    return airTime->isTxAllowedAirUtil() && airTime->isTxAllowedChannelUtil(true) &&
           airTime->utilizationTXPercent() < SLM_MAX_DUTY_PCT;
}

void SoundLevelModule::sendSpectrum()
{
    // Exact averaging window from the accumulated block count (sample-accurate).
    const double windowS = (double)s_accBlocks * OCT_BLOCK / OCT_FS0;
    uint16_t windowU16 = windowS >= 65535.0 ? 65535 : (uint16_t)lround(windowS);

    // Wire format (37 bytes): the band centers are the fixed IEC base-10 set, so only
    // the levels travel. The MQTT bridge knows the band table.
    //   [0]    version (0x01)
    //   [1]    nBands (OCT_NUM_BANDS = 31)
    //   [2..3] window_seconds, uint16 little-endian
    //   [4]    LAeq, uint8, 0.5 dB/LSB
    //   [5]    LCeq, uint8, 0.5 dB/LSB
    //   [6..]  per-band Leq, uint8 each, 0.5 dB/LSB
    uint8_t buf[6 + OCT_NUM_BANDS];
    buf[0] = 0x01;
    buf[1] = (uint8_t)OCT_NUM_BANDS;
    buf[2] = (uint8_t)(windowU16 & 0xFF);
    buf[3] = (uint8_t)(windowU16 >> 8);
    buf[4] = dbToU8(OctaveBank::dBAeq(s_accE, s_accBlocks));
    buf[5] = dbToU8(OctaveBank::dBCeq(s_accE, s_accBlocks));
    for (int b = 0; b < OCT_NUM_BANDS; ++b)
        buf[6 + b] = dbToU8(OctaveBank::bandLeqDb(s_accE, s_accBlocks, b));

    meshtastic_MeshPacket *p = allocDataPacket();   // stamps our port, to=BROADCAST
    p->to = NODENUM_BROADCAST;
    p->want_ack = false;
    p->priority = meshtastic_MeshPacket_Priority_BACKGROUND;
    p->decoded.payload.size = sizeof(buf);
    memcpy(p->decoded.payload.bytes, buf, sizeof(buf));

    LOG_INFO("SoundLevel: TX %us window, LAeq=%.1f LCeq=%.1f", windowU16,
             (double)OctaveBank::dBAeq(s_accE, s_accBlocks), (double)OctaveBank::dBCeq(s_accE, s_accBlocks));
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
