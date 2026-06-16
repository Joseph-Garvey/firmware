# SoundLevelModule

A real-time 1/3-octave sound level meter built into the firmware. It captures
audio from an external PDM microphone, runs a 31-band 1/3-octave filter bank,
and periodically broadcasts the band levels onto the mesh as a small binary
packet (intended for pickup by an MQTT bridge).

> Porting this end-to-end pipeline to [MeshCore](https://github.com/meshcore-dev/MeshCore)?
> See [MESHCORE_PORT.md](MESHCORE_PORT.md) for the design/implementation plan.

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
(~1 s) from the most recent interval (independent of the LoRa transmit window).
(Compile it out on a screen-equipped board with `-DSLM_NO_DISPLAY` — see
[Build-time configuration](#build-time-configuration).) The meter shows:

- the broadband **dBA** level (top-left), a **countdown to the next broadcast**
  (top-center) and the loudest 1/3-octave band's center frequency (top-right), and
- the full **31-band 1/3-octave spectrum** as vertical bars, with **100 Hz /
  1 kHz / 10 kHz** frequency anchor ticks labeled along the bottom.

The countdown shows the longer of two waits with predictable timing: the
`SLM_TMIN_S` averaging window filling (shown in seconds, e.g. `12s`) and the
duty-cycle TX-percent budget freeing up (shown in whole minutes, e.g. `3m` —
the same estimate the firmware uses for "send again in N mins"). Channel
utilization, the remaining transmit gate, depends on other radios' traffic and
can't be predicted, so once both known gates clear the field shows `TX` while
that last gate lets the packet out.

The bars map a fixed 20–100 dB SPL range to the screen. The
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
| `SLM_DMA_DESC_NUM` / `SLM_DMA_FRAME_NUM` | 8 / 512 | I2S DMA ring depth (descriptors × samples ≈ 85 ms of buffering). Deeper = the audio task tolerates longer starvation (e.g. a WiFi burst on a shared core) before a buffer is dropped. `FRAME_NUM × 2` must stay under the 4092-byte per-descriptor DMA limit. |
| `MIC_CAL_OFFSET_DB` | 130.0 | dBFS(rms) -> dB SPL calibration offset for the mic. 130 dB is nominal for the IM72D128VV01 (-36 dBFS @ 94 dB SPL); the onboard XIAO Sense MSM261 mic would be ~120. **Must be field-calibrated** for accurate absolute SPL. |
| `SLM_AUDIO_DIAG` | (unset) | Capture-integrity diagnostics. Logs one line per base interval: `SLM DIAG: up:<s> raw <dBFS> peak <dBFS> crest <dB> | LAeq <dB> | ovf:<n> hz:<rate>`. `crest` (peak−raw) separates impulsive corruption (>~25 dB) from steady audio (~12–18 dB); rising `ovf` / sagging `hz` flags DMA starvation. Off by default; adds a per-block raw/peak pass and a log line. |
| `SLM_FS_LOG` | (unset) | On-device **anomaly log to LittleFS** for diagnosing a *deployed* node with no tethered host. Writes only edge events to `SLM_FS_LOG_PATH` — see "Retrieving logs" below. Independent of `SLM_AUDIO_DIAG` (enabling either computes the raw/peak stats). |
| `SLM_FS_LOG_PATH` | `/static/slm.log` | File path for the anomaly log. Under `/static` so the WiFi web server serves it as `GET /slm.log`. |
| `SLM_FS_LOG_MAX_BYTES` | 65536 | Rotate to `<path>.1` past this size (total on-flash ≤ 2×). |
| `SLM_FS_LOG_CREST_DB` / `SLM_FS_LOG_LAEQ_DB` | 25 / 200 | Excursion thresholds: an interval with crest above the first (impulsive corruption) or `LAeq` above the second (default ~off) is flagged. |
| `SLM_FS_LOG_SUSTAIN_S` | 30 | Min seconds between repeated lines during one sustained excursion / drop storm, so a long event costs a few lines, not one per second. |
| `SLM_NO_DISPLAY` | (unset) | **Disable switch.** Compile out the on-device OLED meter even on a `HAS_SCREEN` board — no `drawFrame`, no UI frame in the carousel, no per-interval display copy. Lets you A/B whether the on-device draw is implicated in a slowdown. |
| `SLM_NO_GLITCH_GUARD` | (unset) | **Disable switch.** Revert the capture path to the pre-`1cb7443d` baseline: no I2S-overflow callback, no drop-the-glitched-interval logic, and the IDF-default DMA ring (ignores `SLM_DMA_*`). Every interval is emitted as captured. |

**Glitch robustness:** by default the audio task watches the DMA overflow counter and, for
any base interval during which a buffer was dropped, discards that interval and resets the
filter bank — so a capture discontinuity makes the meter skip a beat rather than spike (a
dropped buffer otherwise rings the high-Q low-frequency 1/3-octave bands for seconds). This
runs independent of `SLM_AUDIO_DIAG`, and can be turned off with `-DSLM_NO_GLITCH_GUARD`
(see the table) to compare against the baseline capture path.

## Retrieving logs from a deployed node

With `-DSLM_FS_LOG`, anomalies are appended to `/static/slm.log` on the internal LittleFS
(1.5 MiB partition on the 8 MB board). Each line is CSV:
`epoch,uptime_s,tag,LAeq,crest,raw,ovf`, where `tag` is `EXCURSION-START/-END`, a periodic
`EXCURSION` heartbeat, or `DROP` (a discarded glitched interval). Only edges + a slow
heartbeat are written, so it stays in the kilobytes and survives reboots.

Pull it without a serial cable:

- **WiFi web server** — because the file lives under `/static`, the built-in server hands it
  back at `http://<node-ip>/slm.log` (and lists it via `GET /json/fs/browse/static`,
  deletes via `DELETE /json/fs/delete/static`). This is the intended path for a node that is
  already a WiFi/MQTT gateway.
- **BLE / USB** when someone is on-site — the normal Meshtastic debug-log stream
  (`RedirectablePrint` → `LogRecord`) carries the live `SLM DIAG` lines if `SLM_AUDIO_DIAG`
  is also built in.

**Why not send logs over the LoRa mesh?** Don't. LoRa is duty-cycle-capped (e.g. 1 % in
EU868) and the airtime is *shared* with every other node — the whole reason `SLM_MAX_DUTY_PCT`
exists. A single ~90-byte log line is a whole packet; streaming diagnostics would blow the
budget and crowd out real traffic, including the SLM spectrum frames themselves. Logs belong
on the side channels above (WiFi/BLE/USB). For live spectrum to a co-located companion there
is already `SLM_FOH_STREAM`, which uses the local client API (BLE/serial/TCP), **not** LoRa.

## Transmission

Spectrum packets are sent like any other mesh module traffic — there is no
dedicated transport or MQTT publish call:

- **Flood broadcast** (`to = NODENUM_BROADCAST`), *not* a direct/addressed
  message, on the **primary/default channel (index 0)**, same as the stock
  Telemetry modules. The node doesn't address the MQTT gateway (or know which
  node it is) — it floods the spectrum onto channel 0 via Meshtastic's managed
  flood routing (every node rebroadcasts, up to the configured `lora.hop_limit`,
  default 3), and whichever in-range node has `uplink_enabled` pushes it to MQTT.
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

The end-to-end flow, from the audio task on the SLM node to a frame landing on
the MQTT broker:

```
 SLM node
 ┌──────────────────────────────────────────────────────────────┐
 │ PDM mic → I2S capture → 31-band 1/3-octave bank → Leq/band     │
 │ audio task folds energy each base interval (default 1 s)       │
 │ runOnce() accumulates until window ≥ SLM_TMIN_S AND duty gate  │
 │ open, then packs a 35-byte v2 frame                            │
 │   [ ver | nBands | window_s | 31 × Leq @ 0.5 dB/LSB ]          │
 │ service->sendToMesh(): broadcast on PRIVATE_APP (port 256),    │
 │ channel 0, want_ack=false, BACKGROUND priority                 │
 └───────────────────────────────┬──────────────────────────────┘
                                 │ LoRa (channel 0 / default PSK)
                                 ▼
 Bridging node (own MQTT config — not SLM-specific)
 ┌──────────────────────────────────────────────────────────────┐
 │ receives packet, decodes with channel 0 PSK (uplink_enabled)   │
 │ wraps it as ServiceEnvelope{ packet, channel_id, gateway_id }  │
 │ mqtt.encryption_enabled=false ⇒ inner Data stays CLEARTEXT     │
 └───────────────────────────────┬──────────────────────────────┘
                                 │ publishes ServiceEnvelope
                                 ▼
                MQTT broker:  <root>/2/e/<channel_name>/<!nodeid>
                (default      msh/2/e/LongFast/!a1b2c3d4)
                                 │
                                 ▼
        consumer that knows PRIVATE_APP + the v2 wire format
        (e.g. SLM-tools/mqtt_slm_bridge.py) → readable JSON / spectrum
```

The transport from the bridging node to the broker is either direct WiFi or a
USB **client proxy** (a paired phone/PC relays MQTT for the node); both carry the
same `ServiceEnvelope`. See [SLM-tools/](../../../SLM-tools/README.md) for the fully
annotated version of this diagram including both transports and the decode side.

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
dataflow diagram live in [SLM-tools/](../../../SLM-tools/README.md). It supports both a
WiFi gateway (subscribe to a broker) and the more reliable USB **client-proxy**
transport (no WiFi). See that README for why client proxy is preferred (Meshtastic
MQTT has no store-and-forward, so dropped-WiFi frames are lost).

## Directed delivery to the gateway (broadcast vs. DM — analysis, not implemented)

A natural airtime question: instead of flood-broadcasting every spectrum frame,
could a sensor **DM the gateway node directly** so the frame only travels the
path to the broker rather than flooding the whole mesh? The short answer is
*yes, in the right topology it saves airtime — but it's a trade, not a free win,
and it is not implemented.* This section records the full analysis so the
trade-offs are explicit before anyone builds it.

### What does *not* help: gateway placement

Sprinkling extra gateways near the sensors does **not** reduce airtime. If the
broker is N LoRa hops away, the frame costs those N forwarding transmissions
regardless of how many gateways are nearby — reaching a gateway is itself a LoRa
hop (see [Transmission](#transmission)). Extra near-sensor gateways just create
*more* nodes that each uplink the same packet, which is why the bridge needs the
`(from, MeshPacket.id)` dedup (see [SLM-tools/](../../../SLM-tools/README.md)). Gateway
placement changes *who* mirrors to MQTT, not the RF cost.

### How the two addressing modes cost airtime

- **Broadcast (current).** Managed flood routing: every node that hears the
  packet rebroadcasts once, up to `lora.hop_limit`, minus suppression when a node
  already heard a neighbor relay it. The whole reachable mesh "lights up." Cost
  scales with the number of nodes in range, not with the distance to the broker.
- **DM (unicast to the gateway nodenum).** This firmware uses the
  `NextHopRouter` (extends `FloodingRouter`, see
  [NextHopRouter.h](../../mesh/NextHopRouter.h)). Once a **next hop** toward the
  destination is known, only nodes *on the path* relay — every off-path node sees
  `next_hop != me` and stays silent. So a DM prunes the sideways flood:

  ```
  broadcast:  sensor floods → path nodes A,B + off-path C,D,E,F… all rebroadcast
  DM (warm):  sensor → A → B → broker        (only the path relays)
  ```

The saving is **width, not depth**: the DM crosses the same N path hops, but
skips the off-path fan-out. The more off-path nodes a broadcast would have
needlessly flooded, the bigger the DM saving. In a *sparse / line* mesh
(sensor→A→B→broker, nothing off to the side) a DM saves **nothing** and only adds
overhead.

### The routing mechanics that decide whether it actually works

The next hop is **learned, not configured** ([NextHopRouter.cpp](../../mesh/NextHopRouter.cpp),
`getNextHop()` + the relayer-tracking around `checkRelayers`):

- A node learns the next hop toward a destination by observing an **ACK** travel
  back via a node that also relayed the original packet. The learned hop is
  stored per-destination in the NodeDB.
- Once stored, `getNextHop()` returns it for **any** unicast to that destination —
  including `want_ack = false` packets. So a warm route gives path-only relaying
  *without* per-frame ACK cost.
- **But** a route is only ever learned from a **reliable (`want_ack`) delivery.**
  A sensor that only ever sends `want_ack = false` DMs will never prime a route,
  `getNextHop()` returns "no preference," and every DM **falls back to flooding** —
  i.e. zero benefit, identical to broadcast but addressed.
- On a failed final retransmission the next hop is **reset** to no-preference
  (fallback to flood). So a hub rebooting or moving makes the route re-flood until
  it is re-learned.

Implication for SLM: to actually realize the saving you must **periodically send
`want_ack = true`** to keep the route warm (accepting the ACK return traffic and
up-to-3× retransmit cost on those frames), then most frames can ride the learned
hop cheaply with `want_ack = false`. Pure fire-and-forget DMs do not help.

### Two more costs specific to this design

- **Cold start + churn re-floods.** The first frame to a destination floods, and
  any delivery failure resets the route. A mobile or flaky mesh spends a lot of
  time "cold," eroding the average saving.
- **PKI decode hazard (the killer).** A *default* DM in current firmware is
  **PKI / end-to-end encrypted** to the recipient's public key. PKI packets are
  **always uplinked encrypted even when `mqtt.encryption_enabled = false`** — the
  gateway can't decrypt them ([MQTT.cpp](../../mqtt/MQTT.cpp), the `"PKI"`
  channel_id path). The bridge would receive an opaque blob on the `…/PKI/…`
  topic, not the 35-byte spectrum. To keep the payload bridge-readable the DM
  must be sent on a **shared-PSK channel** (a channel-encrypted DM, not a PKI DM)
  so the gateway decodes it with the channel key.
- **Loss of multi-gateway redundancy.** The current broadcast + dedup design is
  robust to any single gateway dying: *any* in-range uplink-enabled node mirrors
  the frame, and the bridge de-dups. A DM targets **one** gateway nodenum — if
  that node is down, the data does not reach MQTT even though other gateways are
  in range. Directed delivery trades the mesh's natural redundancy for airtime.

### Is airtime actually the constraint? (the go/no-go signal)

Before building any of this, confirm airtime is your *binding* constraint — and,
crucially, **which** gate is binding, because a DM only relieves one of them.
`dutyAllows()` ([SoundLevelModule.cpp](SoundLevelModule.cpp)) ANDs three gates,
keyed on two different airtime tallies ([airtime.cpp](../../airtime.cpp)):

| Gate | Tally | What it measures | Does a DM relieve it? |
| --- | --- | --- | --- |
| `isTxAllowedAirUtil()` | `utilizationTXPercent()` | **this node's own** TX airtime vs the region polite duty cap | **No** — the node still sends one frame; `want_ack` retransmits make it *worse* |
| `SLM_MAX_DUTY_PCT` cap | `utilizationTXPercent()` | same own-TX tally vs our stricter self-cap | **No** — same reason |
| `isTxAllowedChannelUtil()` | `channelUtilizationPercent()` | **all** traffic on the channel (mesh-wide congestion) | **Yes** — pruning flood fan-out is exactly less shared occupancy |

The key insight: the airtime a DM saves is **other nodes not rebroadcasting** —
shared channel occupancy — *not* this node transmitting less. So:

- **On-device:** the screen countdown showing **minutes** (`3m`) is the *own-duty*
  gate (`getSilentMinutes` on `utilizationTXPercent()`). If that's what stretches
  your interval, a DM **does not help** — the lever is a longer `SLM_TMIN_S`, a
  lower frame rate, or a higher `SLM_MAX_DUTY_PCT`. A countdown in **seconds**
  (`12s`) just means you're waiting for the averaging window, not airtime-limited
  at all.
- **In the log:** `TX air util. >…%. Skip send` is the own-duty gate (DM won't
  help). `Ch. util >…%. Skip send` — or the screen sitting on `TX` for a long
  time without a `SoundLevel: TX … window` line following — is the
  **channel-utilization** gate. *That* is the signal that a DM is worth it:
  mesh-wide congestion is the limiter, and in a busy star-of-stars that
  congestion is substantially your own (and your neighbours') flood fan-out,
  which directed delivery directly attacks.

### Verdict for a star-of-stars topology

A hub-and-spoke ("star of stars") mesh with one well-defined broker is close to
the **best case** for directed delivery: a single destination is exactly what
next-hop routing is for, and the many off-path branches are exactly the fan-out a
DM prunes. If the hubs are stationary (routes stay warm) and **mesh airtime is
actually the binding constraint**, a channel-PSK DM to the gateway is a
defensible win.

That said, it is **not an obvious "just do it":**

1. It only pays off once routes are warm, which requires deliberate periodic
   `want_ack = true` priming — it is not a one-line "change `to`."
2. It forces channel-PSK DMs (not default PKI) to stay bridge-decodable.
3. It gives up the broadcast design's free multi-gateway redundancy.

Recommendation: **measure first, and check the right gate** (see
[the go/no-go signal](#is-airtime-actually-the-constraint-the-go-no-go-signal)
above). If the **channel-utilization** gate is binding — `Ch. util >…%. Skip
send` in the log, or the screen stalling on `TX` — then mesh-wide congestion is
the limiter, directed delivery directly attacks it, and this is the right
topology to spend the effort. If instead your **own-duty** gate is binding (the
countdown sitting in minutes), a DM won't help and the lever is rate/`SLM_TMIN_S`
instead. And if airtime is *not* currently the binding constraint at all, the
broadcast + dedup design is simpler and strictly more robust — the same reasoning the
[LoRa-suppression back-channel](#lora-suppression-back-channel-v2-planned--not-implemented)
is deferred under. This is a pure airtime optimization; v1 is correct without it.

## Front-of-house live streaming

Front-of-house (FoH) use needs a *live* readout — an engineer watching SPL and
the spectrum update several times a second — which LoRa cannot provide (the
duty-cycle gate stretches the mesh broadcast to one frame every few minutes).
The two consumers are therefore split across two transports with no special
casing of the mesh path:

- **Command center** keeps the existing mesh broadcast — the same long-window
  5-min frame every node sends, unchanged and unconditional (see
  [Transmission](#transmission)).
- **FoH display** receives a high-rate local stream over whatever wired/wireless
  link the companion is already using to talk to the node.

### Local stream (v1)

The live stream reuses Meshtastic's own client transport rather than adding a
new one. Each base interval the module emits the *most recent interval's*
spectrum via `service->sendToPhone()` ([MeshService.cpp](../../mesh/MeshService.cpp)),
which enqueues a `MeshPacket` onto the stream already delivered to any locally
connected client — **BLE, USB serial, or TCP** — with **no LoRa transmission and
no duty-cycle cost**. Properties:

- Same `PRIVATE_APP` port and **same [v2 wire format](#wire-format-v2-35-bytes)**
  as the mesh frame, so the FoH app decodes it with the exact same parser used
  by the MQTT bridge in [SLM-tools/](../../../SLM-tools/README.md). The `window_seconds`
  field is sub-second for an instantaneous frame (rounds toward 0) and the FoH
  app ignores it.
- Emitted only while a client is actually connected (the BLE/serial/TCP
  connection state the firmware already tracks), so nothing is queued into a
  stream no one is draining.
- **Refresh rate** follows `SLM_BASE_INTERVAL_MS`, matching the traditional
  sound-level-meter time weightings: **125 ms ≈ 8 Hz ("fast")** or
  **1000 ms = 1 Hz ("slow")**. This only changes how finely energy is folded;
  the LoRa long-window accumulator is unaffected.
- Gated behind the `SLM_FOH_STREAM` build flag (default off) so non-FoH sensors
  are byte-for-byte unchanged.

The FoH machine is just a standard Meshtastic client. When it *also* has
internet it relays these frames onward to the command center over MQTT — the
same bytes, the same parser — giving the command center a live full-rate feed on
top of the coarse LoRa baseline. When it has no internet, the live readout still
works locally; the command center simply falls back to the 5-min mesh frames.

**Why 8 Hz is safe (backpressure).** Each frame is a `sendToPhone()` enqueue onto
the `toPhoneQueue` (cap `MAX_RX_TOPHONE`, 32 on PSRAM boards). The XIAO-S3 has
PSRAM, so the firmware uses the *dynamic* (heap-backed) packet pool — there is no
fixed-array pool for the FoH stream to exhaust, and 8 frames/s of a 35-byte
payload is negligible against the 8 MB PSRAM. If a slow client (e.g. a laggy BLE
link) can't drain at the emit rate, the queue fills and `sendToPhone()` drops
these `BACKGROUND`/`PRIVATE_APP` frames cleanly (`releaseToPool`), so the worst
case is a *reduced effective frame rate* — never unbounded memory or starved mesh
traffic. And because `sendToPhone()` never enqueues to the radio, the live stream
adds **zero** LoRa load and is never re-broadcast.

### LoRa suppression back-channel (v2, planned — not implemented)

When the FoH relay has internet, the node's 5-min mesh frame is redundant and
its airtime could be reclaimed. The node **cannot** observe this on its own: a
BLE/serial link being up says nothing about whether the companion has *upstream*
connectivity. The only correct mechanism is a **companion→node back-channel** —
the FoH app asserts "uplink is up, you may go quiet" to the node, with a timeout
so the node resumes the mesh broadcast the instant the app stops asserting it
(or the back-channel itself drops).

This is deliberately deferred to a future `SLM_FOH_SUPPRESS_LORA` flag:

- It is a pure airtime optimization; the v1 design is already correct without it
  (different windows mean the live and mesh frames aren't even duplicates, so
  dedup at the command center never fires).
- It adds real failure modes (a flaky back-channel could silence the only path
  the command center has), so it should only land if mesh airtime becomes the
  binding constraint.

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
