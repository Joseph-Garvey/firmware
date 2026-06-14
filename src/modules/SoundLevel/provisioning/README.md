# SLM provisioning configs

Meshtastic CLI config files for flashing settings onto the two node roles in an
SLM deployment. These set the LoRa region, the shared channel-0 PSK, and (for
the gateway) the MQTT bridge — they do **not** flash firmware.

| File | Role | Builds with |
| --- | --- | --- |
| [slm-node.yaml](slm-node.yaml) | Sound level meter — LoRa-only, radios off | `pio run -e seeed-xiao-s3-slm` |
| [foh-node.yaml](foh-node.yaml) | Front-of-house meter — same, Bluetooth on for the phone app | `pio run -e seeed-xiao-s3-slm` |
| [gateway.yaml](gateway.yaml) | mesh → MQTT bridge | a normal (non-SLM) Meshtastic build with WiFi |

## Apply

```sh
# settings only — firmware is flashed separately with PlatformIO / the flasher
meshtastic --configure slm-node.yaml      # on each plain meter (radios off)
meshtastic --configure foh-node.yaml      # on the front-of-house meter (BT on)
meshtastic --configure gateway.yaml       # on the gateway
```

Add `--port /dev/cu.usbmodemXXXX` (or `--host <ip>`) to target a specific device.

## Before you apply

- **gateway.yaml**: replace `password: REPLACE_WITH_MQTT_PASSWORD` with the real
  password for broker user `slm`. The WiFi PSK is already filled in.
- **slm-node.yaml / foh-node.yaml**: give each meter a unique `owner` / `owner_short`.
  Reuse that short name in `tools/slm-labels.json` (keyed by node ID) so the MQTT
  bridge tags decoded frames by name instead of raw `!<nodeid>`.
- **Which meter config?** Both broadcast spectrum on channel 0 and differ only in
  radios. `slm-node.yaml` runs LoRa-only (WiFi + BT off) to save power — apply it over
  USB, since you lose the BLE phone-app link once it's set. `foh-node.yaml` keeps
  Bluetooth on so an operator can connect with the app at front-of-house. Neither needs
  WiFi — only the gateway does.
- These files contain secrets (WiFi PSK, MQTT password) — keep real values out of
  git.

## How they fit together

Both files put channel 0 ("LongFast") on the **same custom 32-byte PSK**, EU_868 /
LONG_FAST. The meters broadcast their `PRIVATE_APP` spectrum packets on that
channel; the gateway has `uplink_enabled` on channel 0 and `encryptionEnabled:
false`, so it decodes with the shared PSK and republishes **plaintext** to MQTT:

```
msh/2/e/LongFast/!<gateway-nodeid>
```

Your consumer subscribes to `msh/2/e/LongFast/+`, unwraps the
ServiceEnvelope/MeshPacket/Data, filters `portnum == PRIVATE_APP`, and parses the
35-byte v2 wire format (see [../README.md](../README.md#wire-format-v2-35-bytes)).

> Rotating the PSK: regenerate it and rebuild both `channel_url`s together — the
> meters and the gateway must always share one PSK.
