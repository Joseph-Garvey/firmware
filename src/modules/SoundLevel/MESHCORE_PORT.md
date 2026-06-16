# Porting the SoundLevel (SLM) module to MeshCore

Design doc / implementation plan for moving the SLM end-to-end pipeline
(capture → spectrum frame → mesh broadcast → gateway → MQTT/dashboard) from this
Meshtastic firmware onto [MeshCore](https://github.com/meshcore-dev/MeshCore).

**Status:** design, not implemented. The conclusions below are grounded in a read
of the MeshCore source at `meshcore-dev/MeshCore` (v1.12.0+ companion protocol),
not just its public docs. File:line references are into that repo.

## TL;DR

- **It ports, and most of the value ports cleanly.** The DSP engine, the audio
  capture, the 35-byte wire format, and the entire consumer side (decoder, dedup,
  labels, dashboard) are transport-agnostic and move nearly verbatim.
- **MeshCore has a native, documented, library-supported primitive that maps 1:1
  onto the SLM broadcast model:** the **group datagram** (`PAYLOAD_TYPE_GRP_DATA`,
  `0x06`) — a channel-addressed (shared-key), flooded binary packet carrying a
  `data_type` (uint16) + blob. `data_type` *is* a portnum, the channel hash *is*
  the shared-PSK channel, flooding *is* the broadcast, and any listening node hears
  it (→ keep our multi-gateway dedup).
- **The upload/gateway side needs no fork.** Stock MeshCore **companion radio**
  firmware already forwards received group datagrams to a host over USB/BLE as
  `RESP_CODE_CHANNEL_DATA_RECV` (`0x1B`). A host script (built on the official
  `meshcore_py`) reads those and publishes to MQTT — structurally identical to this
  project's already-preferred client-proxy transport (`mqtt_slm_bridge.py --serial`).
- **Only the sensor needs firmware** (unavoidable — the mic DSP runs on the MCU),
  and it's a *thin* addition: capture + frame + one `sendGroupData(...)` call on top
  of an untouched radio/mesh/crypto/routing stack. This matches the "we trust their
  radio stack, we slap SLM on top" goal.

## Why a fork is optional (and which parts need one)

Split the decision by node role:

| Node | Needs custom firmware? | Why |
| --- | --- | --- |
| **SLM sensor** | **Yes — thin.** A new MeshCore variant/example. | The PDM mic capture + 1/3-octave DSP must run on the MCU. But the only mesh interaction is one `sendGroupData()` call per window; routing/crypto/radio are untouched. In MeshCore this is the natural shape — an added example app, not a patch to the core. |
| **Gateway / uplink** | **No.** Stock `companion_radio` firmware. | It already surfaces inbound group datagrams to the host (`RESP_CODE_CHANNEL_DATA_RECV`). The MQTT publish happens off-device on the host, exactly like our client-proxy path. |
| **Host bridge** | n/a (not firmware) | A Python script using `meshcore_py`; reuses `slm_frame.parse_v2` unchanged. |

So "fork vs. copy" only arises for the **sensor**, and there a MeshCore in-tree
example *is* the low-blast-radius option you want: the proven radio stack is linked
as-is; we add a leaf.

## The locked on-air primitive: `PAYLOAD_TYPE_GRP_DATA` (0x06)

From `src/Packet.h:25`:

```
PAYLOAD_TYPE_GRP_DATA 0x06  // (unverified) group datagram (prefixed with channel
                            //  hash, MAC) (enc data: data_type(uint16), data_len, blob)
```

This is the precise analog of the SLM broadcast (`SoundLevelModule.cpp:sendSpectrum`):

| SLM on Meshtastic | MeshCore group datagram |
| --- | --- |
| Broadcast on channel 0 (shared PSK) | `GRP_DATA` prefixed with channel hash, channel shared key |
| `PRIVATE_APP` portnum (256) demuxes the payload | `data_type` (uint16) demuxes the payload |
| Flood routing, any `uplink_enabled` node mirrors to MQTT | `ROUTE_TYPE_FLOOD`, any listening companion forwards to its host |
| Multiple gateways → `(from,id)` dedup in the bridge | Multiple companions → same dedup (keep it) |
| `mqtt.encryption_enabled=false` → cleartext to anyone with the PSK | "unverified" channel MAC → decodable by anyone with the channel key |
| 35-byte v2 frame as `decoded.payload` | 35-byte v2 frame as the datagram `blob` |

Capacity: `MAX_GROUP_DATA_LENGTH = MAX_PACKET_PAYLOAD(184) − CIPHER_BLOCK_SIZE − 3 ≈ 165 B`
(`src/MeshCore.h:21`); the host-send path caps at `MAX_CHANNEL_DATA_LENGTH = 163 B`
(companion_protocol.md). Our frame is **35 bytes** — fits with ~5× headroom, no
fragmentation.

> Alternative considered: `PAYLOAD_TYPE_RAW_CUSTOM` (`0x0F`, "custom encryption,
> custom payload"). Rejected for v1 — it gives us a blank slate but forces us to
> own framing/crypto/key handling, whereas `GRP_DATA` reuses MeshCore's channel
> crypto and is already wired through the companion protocol and `meshcore_py`.

### Send side (sensor firmware) — one call

`src/helpers/BaseChatMesh.cpp:496`:

```cpp
bool BaseChatMesh::sendGroupData(mesh::GroupChannel& channel, uint8_t* path,
                                 uint8_t path_len, uint16_t data_type,
                                 const uint8_t* data, int data_len);
```

It prepends `[data_type LE][data_len]` and calls `Mesh::createGroupDatagram(
PAYLOAD_TYPE_GRP_DATA, channel, ...)` (`src/Mesh.cpp:525`). The sensor calls it
per window with `path_len = 0xFF` (flood) and our SLM `data_type`. If the sensor
mesh class doesn't derive from `BaseChatMesh`, the base `Mesh::createGroupDatagram`
+ flood-send is available directly.

### Receive side (gateway → host) — already implemented in stock firmware

Radio `GRP_DATA` → `Mesh::onGroupDataRecv` (`src/Mesh.cpp:233`) →
`BaseChatMesh::onGroupDataRecv` parses `data_type`/`data_len`
(`src/helpers/BaseChatMesh.cpp:388-404`) → `onChannelDataRecv(...)` → the
`companion_radio` firmware emits **`RESP_CODE_CHANNEL_DATA_RECV` (0x1B)** to the
host. Documented wire format (companion_protocol.md "Receive Channel Data Datagram"):

```
Byte 0:    0x1B
Byte 1:    SNR (int8 ×4 → /4.0 dB)
Bytes 2-3: reserved
Byte 4:    channel_idx
Byte 5:    path_len (metadata only on receive)
Bytes 6-7: data_type (uint16 LE)
Byte 8:    data_len
Bytes 9..: payload  ← our 35-byte v2 frame
```

The SNR even gives us a free per-frame link-quality field the Meshtastic path
didn't surface to the bridge.

### `data_type` allocation (our portnum)

`data_type` is an application id, transported opaquely (companion_protocol.md
"Registered data_type values"):

- `0xFF00–0xFFFE` testing/dev, **no registration** → use one here during bring-up
  (e.g. `0xFF01`).
- `0x0100–0xFEFF` registered app namespaces — for a real deployment, submit a PR
  to `docs/number_allocations.md` reserving one SLM value.

## Component-by-component plan

### Tier 1 — ports verbatim / near-verbatim
- `octave_bank.h` — pure C++ DSP, no dependencies. Copy as-is.
- I2S PDM capture + FreeRTOS audio task (`SoundLevelModule.cpp` producer side) —
  ESP-IDF `driver/i2s_pdm.h`, **not** Meshtastic APIs. Compiles on any ESP32-S3
  MeshCore target unchanged. (nRF52/RP2040 MeshCore targets would need a different
  capture backend — out of scope; our hardware is XIAO-S3.)
- `buildV2Frame()` and the 35-byte v2 format — unchanged.
- `SLM-tools/slm_frame.py` (`parse_v2`, `CENTERS`), dedup, labels,
  `slm_dashboard.html`, `mosquitto-slm.conf` — unchanged. The decoded
  `TA/SLM/decoded/<node>` JSON contract is preserved, so the dashboard is untouched.

### Tier 2 — re-glue to MeshCore APIs (small)
- **Mesh TX:** replace `service->sendToMesh(allocDataPacket())` with
  `sendGroupData(channel, NULL, 0xFF, SLM_DATA_TYPE, frame, 35)`.
- **Host bridge:** fork `mqtt_slm_bridge.py` — swap the Meshtastic
  protobuf/`ServiceEnvelope` reader for `meshcore_py`, handle the
  `CHANNEL_DATA_RECV` event, pass `payload` straight to `parse_v2`. Keep dedup
  keyed on `(channel_idx, data_type, payload-hash)` or a frame counter (no
  `MeshPacket.id` here — see Open questions). Everything downstream is identical.
- **Module/thread host:** there's no `SinglePortModule`/`OSThread` registry in
  MeshCore. The sensor is a new variant whose `loop()` drives the
  drain-accumulate-maybe-send cadence (model the board/loop scaffold on
  `examples/simple_sensor`, but broadcast unsolicited rather than answer REQs).

### Tier 3 — reimplement (one real piece)
- **Duty-cycle gating.** Meshtastic's `AirTime` (dual tally: own-TX vs
  channel-util, `isTxAllowedAirUtil()`/`isTxAllowedChannelUtil()`/
  `utilizationTXPercent()`) does not exist. MeshCore has a single node-wide
  `set dutycycle` (default 50%, enforced as a silent period after each TX —
  `docs/cli_commands.md:529`) plus CSMA `txdelay`. So:
  - Keep `SLM_TMIN_S` + the window-stretch accumulator in our loop (time-based;
    the design already tolerates a stretchy interval and reports the exact window).
  - Set MeshCore's node `dutycycle` to the regulatory cap as the backstop.
  - Drop the `SLM_MAX_DUTY_PCT` fair-share-of-shared-tally logic — there's no
    shared per-node TX tally to carve up the same way; the node-wide dutycycle
    limiter covers the "don't hog airtime" intent more crudely.
  - The "DM-to-gateway vs broadcast" airtime analysis in `README.md` is moot:
    not implemented, and MeshCore's routing differs — re-derive only if needed.

### What drops away (good riddance)
Meshtastic's `ServiceEnvelope` protobuf, `uplink_enabled`, `mqtt.encryption_enabled`,
the `msh/2/e/...` topic machinery, and the "JSON uplink can't carry PRIVATE_APP"
problem all disappear — replaced by the single `CHANNEL_DATA_RECV` host event.

### FoH live stream
Same mechanism, no special path: the FoH consumer is just another host attached to
a companion node, reading `CHANNEL_DATA_RECV`. The sensor can emit more frequently
for a locally-attached FoH companion. (Meshtastic needed a separate `sendToPhone()`
path; MeshCore folds it into the one primitive.) The `SLM_FOH_SUPPRESS_LORA`
back-channel idea remains deferred for the same reasons as today.

## Phased implementation

1. **Phase 0 — DONE (this doc):** confirm the primitive + receive path against
   MeshCore source. Locked on `GRP_DATA` / `CHANNEL_DATA_RECV`.
2. **Phase 1 — sensor firmware:** new MeshCore ESP32-S3 variant; port
   `octave_bank.h` + capture task; emit a `GRP_DATA` per window; reimplement the
   duty gate (time-based + node dutycycle). Deliverable: a node broadcasting
   35-byte frames on a shared-key channel.
3. **Phase 2 — host bridge (no firmware):** stock `companion_radio` node on USB +
   a `meshcore_py` script that decodes `CHANNEL_DATA_RECV`, runs `parse_v2`, and
   republishes the existing decoded JSON. Dashboard works unchanged. Deliverable:
   end-to-end spectrum → MQTT.
4. **Phase 3 — polish:** OLED meter (redraw the existing bar/dBA/anchor logic
   against MeshCore's display layer), FoH high-rate consumer, labels/dedup parity.
5. **Phase 4 — optional:** native on-node WiFi→MQTT (a community MQTT-repeater
   fork, e.g. `jmead/Meshcore-Repeater-MQTT-Gateway`, which publishes raw hex —
   but note its docs warn it may currently forward only ADVERTs; validate before
   relying on it). Not on the critical path while the host bridge exists.

## Open questions / risks (verify before/while coding)

- **Sensor mesh base class.** Confirm whether the SLM sensor variant should derive
  from `BaseChatMesh` (to get `sendGroupData` directly) or a leaner `Mesh`
  subclass calling `createGroupDatagram` + flood-send. Either works; pick the
  smaller surface.
- **Per-frame dedup key.** Meshtastic gave us `MeshPacket.id`; `CHANNEL_DATA_RECV`
  exposes `channel_idx`/`data_type`/`payload` but no packet id. Add a 1–2 byte
  monotonic frame counter inside the v2 blob (bump to a v3 frame, or steal a
  reserved byte) so the multi-gateway dedup stays exact. Low cost, do it in Phase 1.
- **Companion offline queue.** If no host is attached, datagrams queue and surface
  via `PUSH_CODE_MSG_WAITING` + `CMD_SYNC_NEXT_MESSAGE` rather than a live push —
  confirm the bridge drains them (or accept loss when no consumer, same as today).
- **Duty-cycle realism.** Validate that the node-wide `dutycycle` limiter + our
  `SLM_TMIN_S` actually yields the cadence we want under EU868; there's no
  channel-utilization gate to lean on like Meshtastic.
- **ESP32-S3 + RadioLib pin/SPI coexistence.** Re-confirm the free-GPIO mic pin
  budget (D1/D2) on the MeshCore board variant — MeshCore (RadioLib) may map the
  SX1262 pins differently than this Meshtastic variant.

## References (MeshCore source & docs)

- Packet types: `src/Packet.h:19-32`, `docs/packet_format.md`
- Group datagram send: `src/helpers/BaseChatMesh.cpp:496` (`sendGroupData`),
  `src/Mesh.cpp:525` (`createGroupDatagram`)
- Group datagram receive: `src/Mesh.cpp:215-233`,
  `src/helpers/BaseChatMesh.cpp:388-404` (`onChannelDataRecv`)
- Payload cap: `src/MeshCore.h:21` (`MAX_GROUP_DATA_LENGTH`)
- Companion protocol (send `0x3E` / receive `0x1B`, `data_type` allocation):
  `docs/companion_protocol.md`
- Duty cycle: `docs/cli_commands.md` (`set dutycycle`)
- Sensor scaffold: `examples/simple_sensor/`, gateway: `examples/companion_radio/`
- Host libraries: `meshcore-dev/meshcore_py`, `meshcore-dev/meshcore.js`
</content>
</invoke>
