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

## On-device screen

On builds with a display (`HAS_SCREEN`), the module adds a UI frame to the
normal screen carousel showing a live meter, refreshed every base interval
(~1 s) from the most recent interval (independent of the LoRa transmit window):

- the broadband **dBA** level (top-left) and the loudest 1/3-octave band's
  center frequency (top-right),
- a horizontal **dBA bar meter**, and
- the full **31-band 1/3-octave spectrum** as vertical bars, with **100 Hz /
  1 kHz / 10 kHz** frequency anchor ticks labeled along the bottom.

Both the meter and the bars map a fixed 20–100 dB SPL range to the screen. The
frame appears once the mic is capturing (a "Warming up..." placeholder shows
until the first interval lands) and is omitted entirely if the mic fails to
start. The displayed levels depend on `MIC_CAL_OFFSET_DB` being calibrated for
real SPL (see [octave_bank.h](octave_bank.h)).

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
  free GPIOs. On the Seeed XIAO ESP32-S3 + Wio-SX1262 combo, the radio owns
  GPIO 7/8/9/38/39/40/41/42, Meshtastic's I2C bus (the IMU + OLED, `variant.h`
  `I2C_SDA=5`/`I2C_SCL=6` — *not* the Arduino `SDA`/`SCL`=47/48) is on GPIO5/6,
  GPS is on 43/44, and GPIO1 is GPS standby — so the only free header pins are
  D1/D2/D3 (GPIO2/3/4). The mic defaults to D1=GPIO2 (clk) / D2=GPIO3 (data).
  **Do not use D4/D5 (GPIO5/6)** — that is the I2C bus and the mic will corrupt
  the IMU/OLED (`i2cWrite … ESP_ERR_INVALID_STATE`).
- **Clock pin**: `SLM_PDM_CLK_PIN`
- **Data pin**: `SLM_PDM_DATA_PIN`

## Build-time configuration

All of these are overridable via `build_flags` in the variant's
`platformio.ini`:

| Flag | Default | Meaning |
| --- | --- | --- |
| `SLM_PDM_CLK_PIN` | 2 | PDM clock GPIO (D1) |
| `SLM_PDM_DATA_PIN` | 3 | PDM data GPIO (D2) |
| `SLM_BASE_INTERVAL_MS` | 1000 | How often the audio task folds energy into the FIFO. The averaging window is always a whole multiple of this. |
| `SLM_TMIN_S` | 15 | Minimum averaging window / transmit spacing, in seconds. Bounds the packet rate. |
| `SLM_MAX_DUTY_PCT` | 2.0 | Self-imposed TX duty cap (% of the last hour), applied on top of Meshtastic's own region/channel-utilization gates. Meshtastic already enforces a "polite" cap of `effectiveDutyCycle * polite_duty_cycle_percent / 100` (e.g. 5% on EU_868) against a tally shared by *all* of the node's traffic — this reserves a fair share of that shared budget for SLM so it doesn't crowd out normal mesh messages. |
| `SLM_AUDIO_CORE` | 0 | Core the audio capture/DSP task is pinned to. Core 0 keeps it off the Arduino/mesh loop (core 1) — but if this node also runs WiFi/MQTT (which lives on core 0), expect contention. |
| `SLM_AUDIO_PRIO` | 5 | FreeRTOS priority of the audio task. |
| `SLM_SNAP_QUEUE_DEPTH` | 8 | Depth of the FIFO between the audio task and `runOnce()`. Drained every base interval, so the default is generous margin. |
| `MIC_CAL_OFFSET_DB` | 130.0 | dBFS(rms) -> dB SPL calibration offset for the mic. 130 dB is nominal for the IM72D128VV01 (-36 dBFS @ 94 dB SPL); the onboard XIAO Sense MSM261 mic would be ~120. **Must be field-calibrated** for accurate absolute SPL. |

## Transmission

Spectrum packets are sent like any other mesh module traffic — there is no
dedicated transport or MQTT publish call:

- Broadcast (`to = NODENUM_BROADCAST`) on the **primary/default channel
  (index 0)**, same as the stock Telemetry modules.
- `want_ack = false`, priority `BACKGROUND`.
- Sent via `service->sendToMesh()`, the same path used by Telemetry's
  `allocDataProtobuf()` + `sendToMesh()`.
- MQTT visibility depends entirely on the node's existing MQTT bridge config
  for that channel ("uplink enabled") — there is no SLM-specific MQTT code.

The key difference from Telemetry is the **port and payload**: Telemetry uses
`TELEMETRY_APP` with a `meshtastic_Telemetry` protobuf, while SLM uses
`PRIVATE_APP` with the custom raw binary payload below. This means SLM packets
are not decoded by the standard MQTT JSON/protobuf pipeline or official
Meshtastic clients — anything consuming this data needs custom logic that
recognizes `PRIVATE_APP` packets and parses the v2 wire format.

### Getting spectrum packets onto MQTT

SLM has no dedicated MQTT code — it just relies on the firmware's normal
mesh→MQTT bridge, so the bridging node needs:

1. **MQTT enabled**: `module_config.mqtt.enabled = true`, with a valid
   `address` (and credentials if required). The node needs network access
   itself, or `proxy_to_client_enabled` so a paired client phone proxies it.
2. **Uplink enabled on channel 0** (the primary channel SLM broadcasts on):
   `channels.settings[0].uplink_enabled = true`. Without this, nothing on
   that channel reaches MQTT.
3. **Encryption choice**:
   - Default PSK ("AQ==") on channel 0 → the node decodes and uplinks
     **plaintext**, which is the easiest case for a bridge to consume.
   - Custom PSK + `mqtt.encryption_enabled = false` → only nodes that know
     the PSK decode-and-uplink plaintext.
   - `mqtt.encryption_enabled = true` → the encrypted blob is always
     uplinked, but the consumer needs the channel PSK to decrypt it.
   - `PRIVATE_APP` (256) is not blocked by any portnum filter, so none of
     this is SLM-specific.
4. **Consumer-side**: the MQTT subscriber must recognize packets with
   `decoded.portnum == PRIVATE_APP` from the SLM node and parse the v2
   wire format below — standard Meshtastic MQTT tooling won't decode it.

### Topic

Packets are published as a `ServiceEnvelope` protobuf (wrapping the
`MeshPacket`) to:

```
<root>/2/e/<channel_name>/<!nodeid>
```

- `<root>` defaults to `msh` (configurable via `module_config.mqtt.root`).
- `<channel_name>` is channel 0's name (e.g. `LongFast` by default).
- `<!nodeid>` is the SLM node's ID, e.g. `!a1b2c3d4`.

So with default config: `msh/2/e/LongFast/!a1b2c3d4`. A bridge can subscribe
to `msh/2/e/LongFast/+` to catch all nodes on that channel, unwrap the
`ServiceEnvelope`/`MeshPacket`/`Data`, filter on `decoded.portnum ==
PRIVATE_APP`, and parse `decoded.payload` per the wire format below.

Recommended setup: keep channel 0 on the default PSK with
`uplink_enabled = true`, set `mqtt.encryption_enabled = false`, and point
`mqtt.address` at your broker — the bridge then receives plaintext `Data`
payloads to parse.

A ready-made bridge that does exactly this — decode the `ServiceEnvelope`, parse
the v2 frame, and republish readable JSON — plus an end-to-end setup guide and
dataflow diagram live in [tools/](../../../tools/README.md). It supports both a
WiFi gateway (subscribe to a broker) and the more reliable USB **client-proxy**
transport (no WiFi). See that README for why client proxy is preferred (Meshtastic
MQTT has no store-and-forward, so dropped-WiFi frames are lost).

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
