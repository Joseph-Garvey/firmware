# SLM MQTT tooling — end-to-end

Consumer-side tooling for the SoundLevelModule: take the raw `PRIVATE_APP` spectrum
packets off the mesh and turn them into readable dB values on MQTT.

- [mqtt_slm_bridge.py](mqtt_slm_bridge.py) — decode + republish bridge for the **mesh /
  command-center** path (the `tools/mqtt_slm_bridge.py` referenced by the broker config).
- [foh_client_bridge.py](foh_client_bridge.py) — decode + republish bridge for the
  **front-of-house** path: reads the high-rate stream straight off a node's client API
  (USB / BLE / TCP). See [Front-of-house live stream](#front-of-house-live-stream) below.
- [slm_frame.py](slm_frame.py) — shared v2-frame decoder (`CENTERS`, `parse_v2`) imported
  by both bridges. Single source of truth for the band table + wire format.
- [FOH_INTEGRATION.md](FOH_INTEGRATION.md) — handoff spec for wiring the FoH feed into a
  separate FoH display app.
- [mosquitto-slm.conf](mosquitto-slm.conf) — local broker config.
- [slm_dashboard.html](slm_dashboard.html) — live browser spectrum view (see
  [Live dashboard](#live-dashboard) below). `mqtt.min.js` is vendored alongside it.
  Works for **both** bridges — same `TA/SLM/decoded/<node>` topic.
- Device provisioning (the gateway + meter side) lives in
  [../src/modules/SoundLevel/provisioning/](../src/modules/SoundLevel/provisioning/).
- Wire format + on-device behavior: [../src/modules/SoundLevel/README.md](../src/modules/SoundLevel/README.md).

## Dataflow, start to finish

```
 ┌─────────────────────────── SLM meter node (!4f4aece2) ───────────────────────────┐
 │ PDM mic → I2S capture → 31-band 1/3-octave filter bank → Leq per band            │
 │ every ≥15s: pack 35-byte v2 frame  [ver|nBands|window_s|31×Leq@0.5dB]            │
 │ broadcast on PRIVATE_APP (port 256), channel 0, BACKGROUND priority              │
 └───────────────────────────────────────┬──────────────────────────────────────────┘
                                          │ LoRa (EU_868 LongFast)
                                          ▼
 ┌──────────────────────────── gateway node "SLM G" (!a1d84e10) ────────────────────┐
 │ receives packet, decodes with channel PSK (uplink_enabled on ch0)                │
 │ wraps it as a ServiceEnvelope{ packet, channel_id, gateway_id }                  │
 │ mqtt.encryption_enabled=false ⇒ inner Data is CLEARTEXT (portnum+payload)        │
 └───────────────┬───────────────────────────────────────────┬──────────────────────┘
                 │ Transport A: client proxy (USB)            │ Transport B: WiFi
                 │ proxy_to_client_enabled=true, WiFi off     │ wifiEnabled=true
                 ▼                                            ▼
   meshtastic python SerialInterface              broker.hivemq.com / local broker
   fires pubsub "mqttclientproxymessage"          topic <root>/2/e/<chan>/<!gw>
                 │                                            │
                 └──────────────┬─────────────────────────────┘
                                ▼
                    tools/mqtt_slm_bridge.py
                    ServiceEnvelope → MeshPacket → Data
                    filter portnum==PRIVATE_APP, parse 35-byte v2 frame
                    dB = byte / 2,  band centers from fixed IEC table
                                │
                 ┌──────────────┴───────────────┐
                 ▼                              ▼
        ASCII spectrum (console)      JSON → local broker
                                      TA/SLM/decoded/<!node>
                                      {node,label,channel,window_s,bands_hz,levels_db,peak_db,peak_hz}
```

## Multiple sensors / multiple gateways

The bridge fans in any number of sensors automatically — every frame is keyed by
its source node ID (the `from` in the MeshPacket), so N sensors land on N distinct
`TA/SLM/decoded/<!node>` topics with no per-node setup. Two extras make a
multi-node deployment readable:

- **Dedup (`--dedup-ttl`, default 30s).** If more than one gateway hears the same
  broadcast, each uplinks it and the bridge would otherwise publish the frame
  twice. The originating sensor stamps every packet with a `MeshPacket.id`, so the
  bridge drops any repeat `(from, id)` seen within the TTL window (also catches
  retained/redelivered frames). Set `--dedup-ttl 0` to disable. The TTL just needs
  to exceed the spread between gateways relaying the same packet (seconds); it is
  well under the ≥15s sensor transmit spacing, so legitimate next frames (which
  carry a *new* id) are never suppressed. The cache is **persisted** to
  `tools/.slm-dedup-state.json` (override with `--dedup-state FILE`, empty string to
  disable), so a quick bridge restart mid relay-window still drops the second copy;
  only entries newer than the TTL are restored. On startup the bridge logs
  `[dedup] restored N recent packet id(s)` when it picks state back up.
- **Labels (`tools/slm-labels.json`).** Map raw node IDs to names so the console and
  the republished JSON carry a `label` (e.g. `Workshop`) instead of just `!4f4aece2`.
  The bridge auto-loads `tools/slm-labels.json` if it exists — copy the committed
  [slm-labels.example.json](slm-labels.example.json) and edit. Use the same name you
  gave each meter as `owner_short` in
  [provisioning/slm-node.yaml](../src/modules/SoundLevel/provisioning/slm-node.yaml).
  Override the path with `--labels FILE`, or add one-offs with `--label NODE=Name`
  (these win over the file). The republish topic stays keyed by node ID (stable);
  the label rides in the payload.

```sh
cp tools/slm-labels.example.json tools/slm-labels.json    # then edit node IDs -> names
# bridge picks it up automatically -- no flag needed:
.venv/bin/python tools/mqtt_slm_bridge.py --serial /dev/cu.usbmodemXXXX \
    --out-broker 127.0.0.1 --out-user slm --out-pass slmdebug123
```

**Two transports into the bridge, same decode out of it:**

- **A — client proxy (recommended).** The gateway hands every MQTT publish to a
  connected USB client as a `MqttClientProxyMessage`; the bridge *is* that client.
  No WiFi, no broker needed for decode. The gateway only uplinks while the bridge
  is connected.
- **B — WiFi → broker.** The gateway joins WiFi and publishes ServiceEnvelopes to a
  real broker; the bridge subscribes and decodes. Closer to a real deployment, but
  see the reliability note below.

## How to set this up (humans)

### 0. Prerequisites
- Firmware built + flashed: `pio run -e seeed-xiao-s3-slm` on the meter; a normal
  Meshtastic build on the gateway.
- Python deps in the project venv: `meshtastic`, `paho-mqtt`.
  (`.venv` has no pip by default — bootstrap once with
  `.venv/bin/python -m ensurepip` then `.venv/bin/python -m pip install paho-mqtt`.)
- `mosquitto` (broker) + `mosquitto_sub`/`_pub` (clients): `brew install mosquitto`.

### 1. Provision the nodes
Edit the placeholders, then apply (see
[provisioning/README.md](../src/modules/SoundLevel/provisioning/README.md)):
```sh
meshtastic --port /dev/cu.usbmodemXXXX --configure src/modules/SoundLevel/provisioning/slm-node.yaml   # meter
meshtastic --port /dev/cu.usbmodemYYYY --configure src/modules/SoundLevel/provisioning/gateway.yaml    # gateway
```
Both must share channel 0's PSK; the gateway needs `uplink_enabled` on channel 0
and `mqtt.encryption_enabled=false`. Give each meter a unique `owner_short` — that's
the name you'll map to its node ID in the next step.

### 1b. Label the meters (optional but recommended)
So decoded frames read `Workshop` instead of `!4f4aece2`:
```sh
cp tools/slm-labels.example.json tools/slm-labels.json   # then edit: "!<nodeid>": "Name"
```
Find each meter's node ID from `meshtastic --info` or its MQTT topic
(`.../2/e/<chan>/<!nodeid>`). The bridge auto-loads this file (step 4) — no flag
needed; it's gitignored as per-deployment.

### 2. Pick a transport on the gateway
```sh
# Recommended: client proxy over USB (no WiFi)
meshtastic --set mqtt.proxy_to_client_enabled true --set network.wifi_enabled false

# Or: WiFi gateway to a broker
meshtastic --set mqtt.proxy_to_client_enabled false --set network.wifi_enabled true \
           --set mqtt.address <broker-ip> --set mqtt.username slm --set mqtt.password <pw>
```
Either way: `mqtt.enabled true`, `mqtt.encryption_enabled false`, `mqtt.root TA/SLM`.

### 3. Start the local broker (for republished JSON, and for transport B)
```sh
mosquitto_passwd -c /opt/homebrew/etc/mosquitto/passwd slm     # one-time, sets the password
mosquitto -c tools/mosquitto-slm.conf -v                        # leave running
```
The shipped config also opens a **WebSockets listener on 9001** (for the dashboard —
browsers can't speak raw MQTT/TCP) and enables **persistence** so retained frames
survive a broker restart. The bridge publishes with `retain=true` (toggle with
`--no-retain`), so the last spectrum per node is held by the broker and any fresh
subscriber gets it immediately instead of waiting ~15s for the next uplink.

### 4. Run the bridge
```sh
# Transport A — client proxy over USB (keep it running; it IS the proxy host)
.venv/bin/python tools/mqtt_slm_bridge.py --serial /dev/cu.usbmodemXXXX \
    --out-broker 127.0.0.1 --out-user slm --out-pass slmdebug123

# Transport B — subscribe to the gateway's broker
.venv/bin/python tools/mqtt_slm_bridge.py --broker <broker-ip> \
    --broker-user slm --broker-pass <pw> --in-root TA/SLM \
    --out-broker 127.0.0.1 --out-user slm --out-pass slmdebug123
```

### 5. Verify
```sh
# Readable JSON, one frame:
mosquitto_sub -h 127.0.0.1 -u slm -P slmdebug123 -t 'TA/SLM/decoded/#' -C 1
```
You should see a frame every ~15s with 31 `levels_db` values, and a `label` field
carrying the name from `slm-labels.json` (or `null` if the node isn't mapped). The
console also prints an ASCII spectrum per frame, headed by `!<node> (Label)`. On
startup the bridge logs `[labels] loaded tools/slm-labels.json` if the file was
found.

## Front-of-house live stream

Everything above is the **mesh → command-center** path: one integrated frame every
≥15s, rate-limited by LoRa duty cycle. The **front-of-house** path is different — a
*live* readout for an engineer standing at the desk — and it doesn't use LoRa or MQTT
to get there.

A node flashed with the `seeed-xiao-s3-slm-foh` env (build flag `SLM_FOH_STREAM`)
additionally streams its most-recent spectrum interval to **any locally connected
client** — USB serial, BLE, or TCP — via the firmware's `MeshService::sendToPhone()`.
These frames never touch the radio, so they arrive at the configured base-interval
rate (**8 Hz** by default, 1 Hz if `SLM_BASE_INTERVAL_MS=1000`), not once per window.
The same node still broadcasts the slow integrated frame to the command center over
LoRa, unchanged — see *Front-of-house live streaming* in
[../src/modules/SoundLevel/README.md](../src/modules/SoundLevel/README.md).

[foh_client_bridge.py](foh_client_bridge.py) is the consumer for this path. It connects
over the client API, decodes the **identical v2 frame** (via the shared
[slm_frame.py](slm_frame.py)), prints an ASCII spectrum, and — with `--out-broker` —
republishes the **same `TA/SLM/decoded/<node>` JSON** the MQTT bridge does, so the
[live dashboard](#live-dashboard) visualizes the FoH feed with zero changes.

```sh
# Watch live frames over USB (auto-detect port) — no broker needed:
.venv/bin/python tools/foh_client_bridge.py --serial

# Feed the dashboard from the FoH feed (fully local, no internet):
.venv/bin/python tools/foh_client_bridge.py --serial \
    --out-broker 127.0.0.1 --out-user slm --out-pass slmdebug123

# BLE (pair first) or TCP (WiFi-joined node):
.venv/bin/python tools/foh_client_bridge.py --ble AA:BB:CC:DD:EE:FF
.venv/bin/python tools/foh_client_bridge.py --tcp 10.0.0.42
```

Notes specific to this path:
- **No dedup, no labels file needed** — a direct client link delivers each frame once,
  from the one node it's connected to. (The MQTT bridge's multi-gateway dedup is moot here.)
- The node only streams **while a client is attached** (the firmware gates on the
  BLE/serial connection), so an idle queue never builds up.
- `window_s` is sub-second for these instantaneous frames and rounds to `0` — expected;
  don't treat it as an averaging window.
- To wire this feed into a **separate FoH display app** (rather than the bundled
  dashboard), follow [FOH_INTEGRATION.md](FOH_INTEGRATION.md) — it specifies the receive
  pattern, the decode, and the JSON contract for a clean handoff.

## Live dashboard

[slm_dashboard.html](slm_dashboard.html) is a self-contained browser view of the
decoded spectrum: a live 31-band bar chart (teal→red by level, peak band outlined)
with 1 kHz / peak / window / frame-age readouts and a connection indicator. It
subscribes over **MQTT-over-WebSockets** to the same `TA/SLM/decoded/#` topics the
bridge republishes — no extra server-side code, just the broker's 9001 listener.

```
 broker (listener 9001, protocol websockets)
        │  ws://127.0.0.1:9001   subscribe TA/SLM/decoded/#
        ▼
 slm_dashboard.html  ──(mqtt.min.js, vendored)──►  canvas bar chart, ~15s/frame
```

### Run it
The page can't be opened over `file://` (the MQTT client won't load), so serve the
`tools/` dir over HTTP:
```sh
cd tools && python3 -m http.server 8000
open http://127.0.0.1:8000/slm_dashboard.html
```
Prerequisites: the broker's WebSockets listener (`listener 9001` / `protocol
websockets`, already in [mosquitto-slm.conf](mosquitto-slm.conf)) and the bridge
running so frames flow. Thanks to broker persistence + retained frames, the chart
paints from the last spectrum on load rather than waiting for the next uplink.

### Credentials
The broker requires auth, but **no password is embedded in the HTML**. At load the
page reads `dashboard-config.json` (a `{ "username", "password" }` file, gitignored)
and falls back to a one-time `prompt()` cached in `sessionStorage` if it's absent.
Copy [dashboard-config.example.json](dashboard-config.example.json) →
`dashboard-config.json` with your local broker creds for a no-prompt demo.

### Notes
- **mqtt.js is vendored** at `tools/mqtt.min.js` (pinned `mqtt@5.10.1`) — no CDN
  dependency, works offline, no third-party script trust.
- **Retained ≠ live.** Because frames are retained, the broker serves the last
  spectrum even after the sensor goes quiet. The "Last frame" readout turns amber
  after 40s (cadence is ~15s) so a stale retained frame doesn't read as current.

## Findings / why it's built this way

These are the non-obvious things learned bringing this up — read before changing it.

- **Client proxy beats WiFi for reliability.** On the XIAO-S3 gateway, WiFi dropped
  repeatedly (`WiFi lost connection`, MQTT `exceeded timeout` every ~15s). Meshtastic
  MQTT uplink is **QoS 0, clean session, with no store-and-forward** — any frame that
  arrives while MQTT is disconnected is **dropped, not replayed**. So a flaky WiFi
  link silently loses spectra. Client proxy runs MQTT over USB and sidesteps this.
- **JSON output can't carry SLM.** `mqtt.json_enabled` only serializes known portnums
  ([MeshPacketSerializer.cpp](../src/serialization/MeshPacketSerializer.cpp) `default:`
  drops the rest), and SLM uses `PRIVATE_APP`. So JSON uplink yields metadata with no
  payload — you must consume the binary `/e/` ServiceEnvelope and parse it yourself.
  That's what this bridge does.
- **Encryption off = readable without keys.** With `mqtt.encryption_enabled=false` the
  gateway uplinks the *decoded* `Data` (cleartext portnum + 35-byte payload), so the
  bridge needs no PSK. With it on, the envelope carries an encrypted blob and you'd
  need the channel key to decrypt first. We run encryption off here (local/bench).
- **dB = byte / 2.** The wire format stores each band's Leq as a `uint8` at 0.5 dB/LSB
  (`dbToU8` = `round(dB*2)`), range 0–127.5 dB. Band centers are the fixed IEC base-10
  1/3-octave set; index 17 is 1 kHz. Absolute SPL accuracy depends on `MIC_CAL_OFFSET_DB`.
- **Topic.** `<root>/2/e/<channel_name>/<!gatewayid>` — here `TA/SLM/2/e/SLM/!a1d84e10`.
  Republished JSON goes to `TA/SLM/decoded/<!node>` keyed by the *meter* node.

### Gotchas (dev environment)
- `meshtastic --get` right after `--set` can return stale values mid-reboot — re-read
  after ~15s, or check the device screen.
- ESP32-S3 native-USB CDC carries only the framed Meshtastic API; opening it raw can
  wedge the device (needs a power-cycle). Firmware logs arrive as framed `LogRecord`s
  via pubsub `meshtastic.log.line`, not as plaintext on the port.
- macOS has no `timeout`; use `mosquitto_sub -W <secs>` for bounded subscribes.
