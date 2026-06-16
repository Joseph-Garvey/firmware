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
- **Operationally it's a lateral move, not a leap:** the same flood airtime
  envelope, a flatter/cleaner host decode, preserved multi-gateway redundancy, and
  one real reliability win (companion-side store-and-forward). See
  [Operational characteristics](#operational-characteristics-dataflow-airtime-redundancy-reliability).

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

## Operational characteristics (dataflow, airtime, redundancy, reliability)

How this architecture behaves versus the current Meshtastic SLM path. Both
broadcast a small frame over **flood routing on a shared-key channel**, so the
fundamentals are shared; the differences live in the gateway/host hop and the
duty-cycle machinery. Headline: **a lateral move with one real reliability win and
a flatter dataflow, offset by minor airtime/redundancy wrinkles that our
dedicated-sensor + host-in-the-loop topology mostly cancels.**

### Dataflow — modest improvement
- Meshtastic hands the host a nested `ServiceEnvelope → MeshPacket → Data`
  protobuf, requires a `portnum == PRIVATE_APP` filter, and forces consuming the
  binary `/e/` topic because JSON uplink drops unknown portnums
  (`MeshPacketSerializer` `default:`).
- MeshCore hands the host one `CHANNEL_DATA_RECV` (0x1B) frame already decrypted,
  demuxed by `data_type`, and reduced to `[SNR][channel_idx][data_type][len][payload]`.
  `parse_v2` runs directly on `payload`; SNR is a free per-frame link metric.
- Cost: upstream MeshCore has **no node→broker path** — a host is mandatory. We
  already chose the USB client-proxy, so this is free for us, but it removes the
  standalone headless-WiFi-gateway option.

### Airtime — roughly neutral (slightly worse on a congested channel)
- Per-frame cost and flood fan-out are ~equal (LoRa SF/BW dominates; the 35-byte
  payload is identical).
- Meshtastic's `AirTime` adds a **channel-utilization** gate
  (`isTxAllowedChannelUtil`, see `SoundLevelModule.cpp` `dutyAllows()`) that backs
  off on mesh-wide congestion. MeshCore's `set dutycycle` throttles only our **own**
  TX (+ CSMA `txdelay`); there is no sustained band-busy gate. So Meshtastic is the
  more polite citizen in a busy mesh.
- Offset: a dedicated SLM sensor has no co-resident traffic, so Meshtastic's
  `SLM_MAX_DUTY_PCT` fair-share carve-out is largely moot here — dropping it costs
  little.
- Minor MeshCore regression: flood **accumulates a path** (up to `MAX_PATH_SIZE`
  = 64 B, `packet_format.md`) as it hops, growing later-hop frames slightly; the
  trade is cheap directed routing later.
- No airtime *win*: the DM-to-gateway fan-out optimization (analyzed in
  [README.md](README.md)) is equally unimplemented in both.

### Redundancy — parity, but two things become our job
- Multi-gateway still works: any in-range companion forwards to its host → N copies
  → dedup.
- **No packet id** in `CHANNEL_DATA_RECV` (Meshtastic dedups on `MeshPacket.id`),
  so multi-gateway dedup requires our own frame counter in the blob — see the v3
  bump in [Open questions](#open-questions--risks-verify-beforewhile-coding).
- **Role split:** MeshCore Companion nodes do not repeat by design, so a
  companion-gateway uplinks only what it directly hears. For coverage *and* uplink,
  pair a Repeater (mesh extension) with companion(s) (uplink). Uplink redundancy is
  unchanged; the topology is just explicit.

### Reliability — the clearest win
- The failure mode this project hit (`SLM-tools/README.md` "Findings"): Meshtastic
  MQTT is **QoS 0, no store-and-forward** — frames arriving while the uplink is down
  are dropped. That is why we abandoned WiFi for the client-proxy.
- MeshCore's companion firmware **queues inbound datagrams** across a host
  disconnect and replays them via `PUSH_CODE_MSG_WAITING` + `CMD_SYNC_NEXT_MESSAGE`
  — store-and-forward at the gateway, the thing Meshtastic lacks. A brief host/link
  flap no longer loses frames (finite queue; a long outage still overflows).
- Tempering: the companion protocol is newer ("still in development"); Meshtastic's
  MQTT is battle-tested, so current bug-surface favors Meshtastic. And the hard host
  dependency means a dead host stops uplink (already true for the client-proxy).

### Scorecard

| Axis | vs current Meshtastic SLM |
| --- | --- |
| Dataflow | Modest improvement — flatter, pre-demuxed, +SNR; loses standalone WiFi gateway |
| Airtime | Neutral; slight regression in congested meshes (no channel-util gate) |
| Redundancy | Parity — needs our dedup counter + deliberate role split |
| Reliability | Real improvement — companion store-and-forward; tempered by maturity |

**Verdict:** not a dramatic upgrade — airtime and redundancy are bounded by the
flood model, which is the same in both. We adopt MeshCore for its radio
stack/networking; the SLM data path comes out **slightly cleaner and meaningfully
more reliable for our host-in-the-loop topology**. One-liner: *same airtime
envelope, simpler decode, better behavior when the uplink flaps.*

Two findings here are actionable design inputs, tracked in
[Open questions / risks](#open-questions--risks-verify-beforewhile-coding): the v3
**dedup counter** (load-bearing for multi-gateway redundancy) and an optional
**channel-busy gate** (to recover Meshtastic's politeness on a congested mesh).

## Routing model: flood discovery vs directed delivery

> Context: the fair critique that "we're just flooding, so we're not benefiting
> from MeshCore." Half-true — and worth pinning down, because MeshCore's routing is
> **reactive source-routed unicast** (DSR/AODV-flavoured), **not** RPL.

What the source says (`src/Mesh.cpp`):

- **Forwarding is opt-in per role.** `allowPacketForward()` returns `false` by
  default (`Mesh.cpp:14` — *"Transport NOT enabled"*); only the **Repeater** role
  flips it on. So a flood is relayed **only by designated repeaters**, not by every
  node (contrast Meshtastic, where every client rebroadcasts by default).
- **Direct packets are source-routed.** They carry an explicit `path` of node
  hashes; a transit node forwards only if it is the next hop
  (`self_id.isHashMatch(pkt->path, …)`, `Mesh.cpp:88`), else discards. Off-path
  nodes stay silent — that is the airtime pruning.
- **Paths are learned reactively from a flood.** A flood accumulates the hashes of
  the nodes it crosses (`Mesh.cpp:332`); the destination reverses that into a path
  and caches it (`out_path`). So **flood is MeshCore's discovery substrate** — even
  directed routing needs an initial flood to learn the path. Flooding is not
  "ignoring MeshCore"; skipping the directed *follow-up* is the only thing left on
  the table.
- **No DODAG / rank / objective function / anycast / link-cost metric** — nothing
  RPL-like to lean on.

### Flood vs. directed, visually

Shared topology (`S` sensor, `Rn` repeater, `G★` gateway = repeater + internet,
`Ln` leaf/client that does **not** relay under MeshCore):

```
            L1                       L2
              \                     /
   S ───────── R1 ──────────────── R2 ───────── G★
               │
               R3 ───── L3
```

Useful route to egress: `S → R1 → R2 → G★`. Everything else (`L1 L2 R3 L3`) is
off-path. "TX" = nodes that key up to move one frame across the mesh:

| Mode | Who transmits | TX | Note |
| --- | --- | --- | --- |
| **(a) Dumb broadcast** | everyone re-sends every copy, forever | ∞ | no dedup / hop-limit — a storm. **Nobody proposes this.** |
| **(b) Managed flood — Meshtastic** | `S R1 R2 R3 L1 L2 L3` | ~7 | every node relays once (dedup + hop limit); leaves relay too |
| **(c) Managed flood — MeshCore (our v1)** | `S R1 R2 R3` | ~4 | only **repeaters** relay; leaves stay silent — already leaner, and **not** a dumb broadcast |
| **(d) Directed — MeshCore (optimisation)** | `S R1 R2` | 3 | path-routed to `G★`; off-path `R3` silent; minimal airtime, one gateway, needs a warm path |

The progression ∞ → 7 → 4 → 3 is the point: our **default already prunes the
leaves** (repeater-only relay), and **directed** additionally prunes the off-path
repeater branch. The `(c) → (d)` gap — the off-path fan-out — is the *only* airtime
a routing change can buy.

### Relay control: roles & defaults vs. Meshtastic

A natural question: isn't MeshCore's "Repeater vs. client" just Meshtastic role
assignment? Not quite — and the Meshtastic analog is **not** "client vs. sensor".
In Meshtastic the rebroadcast decision is (`FloodingRouter.cpp:155`):

```cpp
isRebroadcaster() = (role != CLIENT_MUTE) && (rebroadcast_mode != NONE);
```

so **CLIENT, SENSOR, TRACKER, ROUTER, REPEATER all relay** — the `SENSOR` role
(`NodeDB.cpp:1193`) only changes the node's *own* telemetry/power, not relaying.
The relay knob is `CLIENT_MUTE` / `rebroadcast_mode`, giving the mapping:

| MeshCore | Meshtastic equivalent |
| --- | --- |
| Companion / client — does **not** relay (`allowPacketForward()=false`, `Mesh.cpp:14`) | `CLIENT_MUTE` role, or `rebroadcast_mode=NONE` |
| Repeater — relays | CLIENT / ROUTER / REPEATER (all relay by default) |

**The core difference is an inverted default**, not a capability gap:

- **MeshCore: relay OFF by default** — opt *in* by making a node a Repeater.
- **Meshtastic: relay ON by default** for ~every role — opt *out* via `CLIENT_MUTE`
  / `rebroadcast_mode=NONE`.

So mode (c) vs (b) in the table above is mostly a *default*: you could mute the
Meshtastic leaves to get (c), or make every MeshCore node a Repeater to get (b).

**Two differences that are not just defaults:**

1. **Selective relay — Meshtastic has more no-fork knobs.** `rebroadcast_mode`
   offers `LOCAL_ONLY`, `KNOWN_ONLY` (only nodes in NodeDB, `Router.cpp:444`),
   `CORE_PORTNUMS_ONLY`, `NONE`. MeshCore's `allowPacketForward()` is a plain
   boolean — selective relay (e.g. "only SLM," or intercept-and-suppress) needs a
   repeater fork, i.e. **Option B** above.
2. **Payload-agnostic repeater — favors us.** Meshtastic's **ROUTER role defaults
   to `CORE_PORTNUMS_ONLY`** (`NodeDB.cpp:1186`), which **silently drops
   non-standard portnums** (`Router.cpp:780-795`; allowlist = NodeInfo / Text /
   Position / Telemetry / Routing). **`PRIVATE_APP` is not on it**, so a Meshtastic
   router refuses to relay SLM frames unless set back to `ALL`. A **stock MeshCore
   Repeater relays every payload type, including `GRP_DATA`** — no portnum
   allowlist. For a custom-payload sensor this is a real MeshCore advantage.

**Upshot:** on MeshCore, leave meters as default companions (silent) and make a few
infra nodes Repeaters — they relay `GRP_DATA` with no per-node fiddling. The same
topology on Meshtastic infra would need the leaves muted *and* the relays kept off
the ROUTER `CORE_PORTNUMS_ONLY` default — two easy-to-miss steps MeshCore avoids.

### The three ways to "use the optimised routing"

**Option A — Directed delivery to a chosen gateway** (the RPL-ish "nearest gateway").
- *Native:* send a **Direct `GRP_DATA` along a learned path** —
  `sendGroupData(channel, path, path_len, …)` already takes a path; only on-path
  repeaters relay, and the gateway still decodes via the **channel key** (not PKI),
  so it stays bridge-readable. Path failure auto-falls-back to flood.
- *Not native:* true anycast across N gateways / "nearest of many" — addresses are
  per-node pubkey hashes, no rank. Approximate it **on the sensor**: learn paths to
  1–N gateways from their adverts, pick fewest-hops / best-SNR, re-select on
  failure. All sensor-side, no core changes.
- *Verdict:* the right optimisation **if airtime is the binding constraint**;
  moderate effort, contained to our node.

**Option B — Gateway terminates the flood** (the interception idea).
- *Correction (thanks):* the intercepting node **is** the gateway (a repeater with
  internet), so this is "a gateway uploads and stops propagating," via
  `allowPacketForward` + `Packet::markDoNotRetransmit()` (`Packet.h:89`).
- *Two knobs:* **upload-and-relay** (opportunistic egress — zero redundancy risk,
  no airtime saving) vs **upload-and-suppress** (flood terminates at the egress —
  saves the fan-out *downstream of the gateway*, mild redundancy risk if consumers
  sit beyond it). Saving scales with how **interior** the gateway is: an edge
  gateway (as `G★` above) has little downstream to trim; a central / cut-vertex
  gateway saves more.
- *Cost:* the gateway must run **repeater-role firmware** with an SLM `data_type`
  check (stock companions never relay) — a contained infrastructure-node fork, the
  MeshCore analog of a Meshtastic router-with-uplink.
- *Verdict:* viable, and the better fit if you want to keep flood's discovery +
  multi-gateway redundancy while clawing back downstream airtime. Complementary to
  A (A prunes at the source; B trims at the sink).

**Option C — Make the internet link part of MeshCore's routing cost.**
- MeshCore has **no link-cost metric** and no non-LoRa virtual-edge concept; paths
  are LoRa hop-hashes learned by observation, not cost-optimised. Implementing
  "routing prefers the low-cost egress" means adding a metric-based router + a
  virtual-link abstraction to the **core** — turning MeshCore into
  RPL-with-heterogeneous-links.
- *Verdict:* **not recommended** — a core rewrite against the "slap on top"
  principle, for a benefit Option A already gets app-side.

### Go / no-go (measure first)

Same logic as `README.md` "Directed delivery to the gateway," which transfers
directly:

- Directed delivery saves **width** (off-path fan-out), not **depth** (still N hops
  to egress). The `(c) → (d)` gap above *is* the prize, and it is ~zero on a
  sparse / line mesh.
- It only pays when **mesh-wide channel utilisation** is the binding constraint
  **and** there are real off-path branches — a **star-of-stars is the best case**.
- It needs **warm routes** (reactive paths decay; failures re-flood) and trades away
  the broadcast's **free multi-gateway redundancy** (Option B keeps more of it).
- For a 35-byte frame every ≥15 s, flood's absolute airtime may already be
  negligible. **Ship v1 on managed flood** (mode (c): correct, redundant, simple),
  then add Option A or B as a **measured, build-flagged** optimisation only if the
  channel-utilisation signal says airtime is actually binding.

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
  - For the broadcast-vs-directed airtime trade, see
    [Routing model](#routing-model-flood-discovery-vs-directed-delivery) — directed
    delivery is a deferred, measured optimisation (Phase 5), not part of v1.

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
6. **Phase 5 — optional, measured:** routing optimisation — directed `GRP_DATA` to
   a selected gateway (Option A) and/or gateway-side flood termination (Option B),
   behind a build flag. Only if measurement shows mesh-wide channel utilisation is
   the binding constraint (see
   [Routing model](#routing-model-flood-discovery-vs-directed-delivery)).

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
- **Politeness gate (recover Meshtastic's channel-util backoff).** MeshCore's
  node-wide `dutycycle` throttles only our own TX, with CSMA `txdelay` for
  instantaneous collisions, but has no sustained "band is busy, hold off" gate like
  Meshtastic's `isTxAllowedChannelUtil()`. On a congested shared channel this is a
  regression (see
  [Operational characteristics](#operational-characteristics-dataflow-airtime-redundancy-reliability)).
  If we deploy onto a busy mesh, add an optional channel-activity check (e.g. gate
  on recent RX airtime / RSSI-above-noise) before `sendGroupData`. Skip it on a
  quiet, dedicated channel.
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
