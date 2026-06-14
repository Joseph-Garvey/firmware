# Front-of-house app integration — handoff

**Goal:** pipe *real* SLM frames from a node into the existing FoH app (which
currently runs on synthetic data). This document is the spec for that wiring.

You do **not** need to touch the firmware or invent a wire format — both exist.
Your job is the seam between "frames arriving off the node" and "the data
structure the FoH app already consumes."

## What the node emits

A node flashed with the `seeed-xiao-s3-slm-foh` env (build flag `SLM_FOH_STREAM`,
see [../src/modules/SoundLevel/README.md](../src/modules/SoundLevel/README.md) →
*Front-of-house live streaming*) emits its most-recent spectrum interval to **any
locally connected client** — USB serial, BLE, or TCP — via the firmware's
`MeshService::sendToPhone()`. Key properties:

- **No LoRa, no airtime.** These frames go *only* to the attached client, so they
  arrive at the configured base-interval rate: **8 Hz** (`SLM_BASE_INTERVAL_MS=125`,
  the FoH default) or 1 Hz if set to 1000. This is unrelated to the once-per-few-
  minutes LoRa broadcast the same node still sends to the command center.
- They are ordinary Meshtastic packets on **`PRIVATE_APP`** (portnum 256), `from` =
  the node's ID, `to` = broadcast. The firmware emits them only while a client is
  connected, so nothing accumulates when the app is closed.
- The payload is the **35-byte v2 frame** — byte-identical to the LoRa/MQTT path.

## How to receive them

Use the `meshtastic` Python library's client interface + its pubsub events. This
is the **same library and pattern** the MQTT client-proxy bridge already uses;
the only change is subscribing to `meshtastic.receive` (packets off the radio)
instead of `meshtastic.mqttclientproxymessage`.

```python
from pubsub import pub
import meshtastic.serial_interface          # or .ble_interface / .tcp_interface

def on_receive(packet=None, interface=None):
    dec = packet.get("decoded")
    if not dec or dec.get("portnum") not in ("PRIVATE_APP", 256):
        return
    payload = dec.get("payload")             # raw bytes; lib leaves PRIVATE_APP undecoded
    frame = parse_v2(bytes(payload))         # see below / foh_client_bridge.py
    node  = f"!{packet.get('from', 0):08x}"
    # ... hand `frame` to the FoH app ...

pub.subscribe(on_receive, "meshtastic.receive")
iface = meshtastic.serial_interface.SerialInterface()   # auto-detects the USB node
```

A complete, runnable reference doing exactly this lives in
[foh_client_bridge.py](foh_client_bridge.py) — serial/BLE/TCP modes, ASCII
spectrum, optional MQTT republish. Read `parse_v2` and `on_receive` there.

## The v2 wire format (decode)

| Bytes | Field | Notes |
| --- | --- | --- |
| 0 | `version` | `0x02` |
| 1 | `nBands` | 31 (fixed IEC base-10 1/3-octave set) |
| 2–3 | `window_seconds` | uint16 LE. **Sub-second for a FoH frame → rounds to 0. Ignore it.** |
| 4–34 | per-band `Leq` | uint8 each, **0.5 dB/LSB** (so `byte / 2.0` → dB) |

The decode is already implemented once, in [slm_frame.py](slm_frame.py) — the
single source of truth shared by both bridges. **Import it; don't re-implement:**

```python
from slm_frame import CENTERS, parse_v2   # tools/ must be on sys.path

frame = parse_v2(payload_bytes)   # -> dict, or None if it isn't a v2 frame
# {window_s, bands_hz, levels_db, peak_db, peak_hz}; CENTERS[17] == 1 kHz
```

`CENTERS` is the fixed band table and must match the firmware's `OctaveBank`; it
lives only in `slm_frame.py`, so there's nothing to keep in sync.

## The decoded JSON contract (what to feed the app)

This is the canonical shape used across the project — the MQTT bridge republishes
it and [slm_dashboard.html](slm_dashboard.html) renders it. **If your synthetic
data already looks like this, you're done — just swap the source.** If not, map
these fields onto whatever the FoH app expects:

```json
{
  "node":     "!a1b2c3d4",
  "label":    null,
  "channel":  0,
  "window_s": 0,
  "bands_hz": [20, 25, 31.5, ...],
  "levels_db":[41.5, 40.0, ...],
  "peak_db":  75.0,
  "peak_hz":  1000
}
```

**The app uses an in-process synthetic generator**, so the integration is a
**source swap**, not a new transport: find the function/loop that currently
produces synthetic frames and replace it with a live Meshtastic feed that pushes
real frames onto the *same* queue/callback the generator fed.

Concretely:

1. **Locate the seam** — the one place synthetic frames enter the app (the
   generator call, or the queue/callback it writes to). Note the exact shape it
   produces; map `parse_v2` output (the dict above) onto it, renaming fields if
   the app's internal shape differs.
2. **Start a Meshtastic interface once, at app startup** (replacing the generator's
   start) and subscribe to `meshtastic.receive`:
   ```python
   from pubsub import pub
   import meshtastic.serial_interface
   from slm_frame import parse_v2          # tools/ on sys.path

   def on_receive(packet=None, interface=None):
       dec = packet.get("decoded")
       if not dec or dec.get("portnum") not in ("PRIVATE_APP", 256):
           return
       frame = parse_v2(bytes(dec["payload"]))
       if frame is None:
           return
       frame["node"] = f"!{packet.get('from', 0):08x}"
       app_push(frame)                     # <-- the same sink the generator used

   pub.subscribe(on_receive, "meshtastic.receive")
   iface = meshtastic.serial_interface.SerialInterface()   # keep a reference; .close() on exit
   ```
   `on_receive` runs on the meshtastic library's RX thread, so if the app's sink
   isn't thread-safe, hand off via a `queue.Queue` (the synthetic path may already
   do this).
3. **Keep a dev fallback** — leave the synthetic generator behind a flag so the UI
   can still run with no node attached.

[foh_client_bridge.py](foh_client_bridge.py) is the same logic as a standalone
script — copy its `on_receive`/`open_interface` rather than rewriting them.

> Other patterns (not needed here, for reference): an MQTT-fed app can just consume
> `foh_client_bridge.py --out-broker …` on `TA/SLM/decoded/<node>`; a pipe-fed app
> can read its JSON-lines stdout.

## Testing

1. Flash a node: `pio run -e seeed-xiao-s3-slm-foh -t upload` (XIAO ESP32-S3 + PDM mic).
2. Connect it over USB. Confirm raw frames first with the reference bridge:
   ```sh
   .venv/bin/python tools/foh_client_bridge.py --serial
   ```
   You should see an ASCII spectrum updating ~8×/second (vs the MQTT path's
   once-per-window). Tap/whistle near the mic — the peak band should track.
3. Visualize via the existing dashboard (optional, fully local):
   ```sh
   .venv/bin/python tools/foh_client_bridge.py --serial --out-broker 127.0.0.1 \
       --out-user slm --out-pass slmdebug123
   # then serve tools/ and open slm_dashboard.html (see tools/README.md)
   ```
4. Wire into the FoH app per the seam above; verify it now updates from the live
   feed at the same rate.

## Gotchas

- **Connection gate:** the firmware only streams while a client is attached. If
  you see nothing, confirm the interface actually connected (the bridge prints
  `[ready]`) and that the build has `SLM_FOH_STREAM`.
- **Rate vs the LoRa path:** FoH frames are instantaneous and frequent; the LoRa
  frames are long-window integrated Leq. Don't average FoH `window_s` (it's ~0).
- **No dedup needed.** Unlike the MQTT multi-gateway path, a direct client link
  delivers each frame once. (Dedup logic in `mqtt_slm_bridge.py` is irrelevant here.)
- **Multi-node:** if the FoH laptop talks to several nodes, key everything by
  `node` (`!xxxxxxxx`). One interface = one node; use multiple interfaces or a TCP
  node-per-host for several. (The command center already fans in over MQTT.)
- **BLE** needs the node paired first and can be flakier than USB; prefer the
  cable for a fixed FoH position. TCP requires the node joined to a local network.
- **Band table** (`CENTERS`) and the v2 parser live once in `slm_frame.py`, shared
  by both bridges. If the firmware's `OctaveBank` band set ever changes, update
  `slm_frame.py` and nowhere else.
```
