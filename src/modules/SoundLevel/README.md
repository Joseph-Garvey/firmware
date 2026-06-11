# SoundLevelModule

A real-time 1/3-octave sound level meter built into the firmware. It captures
audio from an external PDM microphone, runs a 31-band 1/3-octave filter bank,
and periodically broadcasts the band levels onto the mesh as a small binary
packet (intended for pickup by an MQTT bridge).

## What it does

- Captures mono PDM audio at 48 kHz via the ESP-IDF I2S PDM-RX driver.
- A dedicated FreeRTOS task runs the audio capture + DSP (`OctaveBank`,
  see [octave_bank.h](octave_bank.h)), producing per-band energy sums every
  "base interval" (default 1 s).
- The module's `OSThread::runOnce()` drains those per-interval snapshots into
  a running accumulator (band energies are additive, so this is O(1) memory
  regardless of window length).
- When the averaging window has reached at least `SLM_TMIN_S` seconds, there
  is accumulated energy, and Meshtastic's duty-cycle/airtime checks (plus our
  own stricter cap) allow it, the module transmits a 31-band Leq spectrum as a
  broadcast packet on the `PRIVATE_APP` port and resets the accumulator.

The transmit interval is therefore not fixed — it stretches automatically
whenever the duty-cycle gate is closed, and each packet reports the exact
elapsed window (in seconds) so the receiving side can compute correct Leq.

## Enabling it

The module only compiles in when both `ARCH_ESP32` and `SLM_ENABLED` are
defined. It's wired up via the additive `seeed-xiao-s3-slm` PlatformIO
environment (see
[variants/esp32s3/seeed_xiao_s3/platformio.ini](../../../variants/esp32s3/seeed_xiao_s3/platformio.ini)):

```sh
pio run -e seeed-xiao-s3-slm
```

The stock `seeed-xiao-s3` env is untouched — use that for a normal (non-SLM)
build.

## Hardware

- **Mic**: external PDM microphone (e.g. Infineon IM72D128VV01), wired to
  free GPIOs. On the Seeed XIAO ESP32-S3 + Wio-SX1262 combo, the radio already
  owns GPIO 7/8/9/38/39/40/41/42, I2C/Wire uses the variant defaults SDA=47/SCL=48,
  and GPS is on 43/44 — so the mic clock/data pins must avoid those (the defaults
  D4=GPIO5 / D5=GPIO6 are free).
- **Clock pin**: `SLM_PDM_CLK_PIN`
- **Data pin**: `SLM_PDM_DATA_PIN`

## Build-time configuration

All of these are overridable via `build_flags` in the variant's
`platformio.ini`:

| Flag | Default | Meaning |
| --- | --- | --- |
| `SLM_PDM_CLK_PIN` | 6 | PDM clock GPIO |
| `SLM_PDM_DATA_PIN` | 5 | PDM data GPIO |
| `SLM_BASE_INTERVAL_MS` | 1000 | How often the audio task folds energy into the FIFO. The averaging window is always a whole multiple of this. |
| `SLM_TMIN_S` | 15 | Minimum averaging window / transmit spacing, in seconds. Bounds the packet rate. |
| `SLM_MAX_DUTY_PCT` | 1.0 | Self-imposed TX duty cap (% of the last hour), applied on top of Meshtastic's region/channel-utilization gates. Use 1.0 to stay within EU868's 1% sub-band limits (Meshtastic's airtime tracker only models the region-wide 10% figure). |
| `SLM_AUDIO_CORE` | 0 | Core the audio capture/DSP task is pinned to. Core 0 keeps it off the Arduino/mesh loop (core 1) — but if this node also runs WiFi/MQTT (which lives on core 0), expect contention. |
| `SLM_AUDIO_PRIO` | 5 | FreeRTOS priority of the audio task. |
| `SLM_SNAP_QUEUE_DEPTH` | 8 | Depth of the FIFO between the audio task and `runOnce()`. Drained every base interval, so the default is generous margin. |
| `MIC_CAL_OFFSET_DB` | 130.0 | dBFS(rms) -> dB SPL calibration offset for the mic. 130 dB is nominal for the IM72D128VV01 (-36 dBFS @ 94 dB SPL); the onboard XIAO Sense MSM261 mic would be ~120. **Must be field-calibrated** for accurate absolute SPL. |

## Wire format (v2, 35 bytes)

Sent as a broadcast packet on `meshtastic_PortNum_PRIVATE_APP`:

| Bytes | Field | Notes |
| --- | --- | --- |
| 0 | `version` | `0x02` |
| 1 | `nBands` | 31 (the IEC base-10 1/3-octave band set) |
| 2-3 | `window_seconds` | uint16 little-endian; exact elapsed averaging window |
| 4-34 | per-band `Leq` | uint8 each, 0.5 dB/LSB (so 0..127.5 dB range) |

The band center frequencies are a fixed table (`OCT_NUM_BANDS = 31`, IEC
base-10 1/3-octave series), so only the levels need to travel — the receiving
MQTT bridge knows the band table and re-derives A/C-weighted broadband levels
(LAeq/LCeq) from the per-band values, avoiding redundant weighting math on the
device.

## DSP engine

See [octave_bank.h](octave_bank.h) for the filter bank itself: a multirate
decimation tree feeding 6th-order Butterworth band-pass filters (3 cascaded
biquads per band), with float math in the 48 kHz hot loop and double-precision
energy accumulators for the interval totals.

## Failure behavior

If I2S/PDM init or the audio task/queue fail to come up on the first
`runOnce()`, the module disables itself cleanly (`disable()`) rather than
retrying — check the log for `SoundLevel: ...` errors if no spectrum packets
appear.
