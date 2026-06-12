# SLM MQTT tooling — end-to-end

Consumer-side tooling for the SoundLevelModule: take the raw `PRIVATE_APP` spectrum
packets off the mesh and turn them into readable dB values on MQTT.

- [mqtt_slm_bridge.py](mqtt_slm_bridge.py) — decode + republish bridge (this is the
  `tools/mqtt_slm_bridge.py` referenced by the broker config).
- [mosquitto-slm.conf](mosquitto-slm.conf) — local broker config.
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
                                      {node,channel,window_s,bands_hz,levels_db,peak_db,peak_hz}
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
and `mqtt.encryption_enabled=false`.

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
You should see a frame every ~15s with 31 `levels_db` values. The console also
prints an ASCII spectrum per frame.

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
