#!/usr/bin/env python3
"""SLM MQTT bridge / decoder.

Turns the SoundLevelModule's raw PRIVATE_APP spectrum packets into human-readable
dB values: it unwraps the Meshtastic ServiceEnvelope -> MeshPacket -> Data, filters
on portnum == PRIVATE_APP, parses the 35-byte v2 wire format (see
../src/modules/SoundLevel/README.md), prints an ASCII spectrum, and republishes
clean JSON to  <out-root>/decoded/<!node>  on a local broker.

It can pull frames from either side of the gateway:

  --serial PORT    Client-proxy mode. The gateway (mqtt.proxy_to_client_enabled=true,
                   WiFi off) relays each MQTT publish to us over USB as a
                   MqttClientProxyMessage. Most reliable path — no WiFi, no broker
                   round-trip needed for decode. This process IS the proxy host, so
                   it must stay running for the gateway to uplink at all.

  --broker HOST    Broker mode. A WiFi gateway uplinks ServiceEnvelopes to a real
                   broker; we subscribe to <in-root>/2/e/+/+ and decode.

REQUIRES mqtt.encryption_enabled=false on the gateway, so the Data payload arrives
in cleartext. (With encryption on you'd need the channel PSK to decrypt first.)

Examples:
  # USB client-proxy + republish to the local broker
  python mqtt_slm_bridge.py --serial /dev/cu.usbmodemXXXX \
      --out-broker 127.0.0.1 --out-user slm --out-pass slmdebug123

  # Subscribe to a WiFi gateway's broker and republish decoded JSON
  python mqtt_slm_bridge.py --broker 10.250.0.155 --broker-user slm --broker-pass slmdebug123 \
      --in-root TA/SLM --out-broker 127.0.0.1 --out-user slm --out-pass slmdebug123
"""
import argparse, json, os, sys, time

from meshtastic.protobuf import mqtt_pb2, portnums_pb2

from slm_frame import CENTERS, fmt_hz, parse_v2   # shared v2 decode (also used by foh bridge)

PRIVATE_APP = portnums_pb2.PortNum.PRIVATE_APP  # 256


def decode_envelope(data, args, out):
    """Parse one MQTT payload (ServiceEnvelope bytes); print + republish if it's an SLM frame."""
    try:
        env = mqtt_pb2.ServiceEnvelope.FromString(data)
    except Exception:
        return
    pkt = env.packet
    if pkt.WhichOneof("payload_variant") != "decoded":
        # encrypted blob -- needs the channel PSK; we require encryptionEnabled=false
        if not getattr(decode_envelope, "_warned", False):
            print("  !! packet is encrypted -- set mqtt.encryption_enabled=false on the gateway",
                  file=sys.stderr)
            decode_envelope._warned = True
        return
    if pkt.decoded.portnum != PRIVATE_APP:
        return
    p = pkt.decoded.payload
    frm = getattr(pkt, "from")
    node = f"!{frm:08x}"

    # Dedup: the inner MeshPacket.id is assigned by the *originating* sensor, so
    # if several gateways hear the same broadcast and each uplinks it (or a frame
    # is redelivered/retained), the (from, id) pair repeats. Squelch those within
    # a short TTL window. id==0 means "unset" -- can't dedup, let it through.
    pid = getattr(pkt, "id", 0)
    if args.dedup_ttl and pid:
        now = time.time()
        seen = decode_envelope.__dict__.setdefault("_seen", {})
        cutoff = now - args.dedup_ttl
        for k in [k for k, t in seen.items() if t < cutoff]:
            del seen[k]
        key = (frm, pid)
        if key in seen:
            seen[key] = now  # refresh so a steady stream of dups keeps it pinned
            save_dedup_state(seen, args.dedup_state)
            if not args.quiet:
                print(f"  .. {node}: dup packet id={pid:#010x} via {env.gateway_id} -- skipped")
            return
        seen[key] = now
        save_dedup_state(seen, args.dedup_state)  # survive a restart mid-relay-window

    label = args.label_map.get(node)
    tag = f"{node} ({label})" if label else node
    frame = parse_v2(bytes(p))   # shared decode -> {window_s, bands_hz, levels_db, peak_db, peak_hz}
    if frame is None:
        print(f"  !! {tag}: not a v2 35-byte SLM frame (len={len(p)} ver={p[0] if p else '?'})")
        return

    if not args.quiet:
        levels = frame["levels_db"]
        print(f"\n=== {tag}  ch={env.channel_id}  window={frame['window_s']}s  "
              f"1kHz={levels[17]:.1f}dB  peak={frame['peak_db']:.1f}dB "
              f"@ {fmt_hz(frame['peak_hz'])}Hz ===")
        for b in range(len(levels)):
            print(f"   {fmt_hz(CENTERS[b]):>6}Hz {levels[b]:5.1f} |{'#' * int(levels[b] / 2)}")

    if out is not None:
        # retain=True so a fresh subscriber (e.g. the dashboard) gets the last
        # frame immediately instead of waiting ~15s for the next uplink.
        out.publish(f"{args.out_root}/decoded/{node}",
                    json.dumps({"node": node, "label": label, "channel": env.channel_id,
                                **frame}),
                    retain=args.retain)
    decode_envelope.count = getattr(decode_envelope, "count", 0) + 1


# Picked up automatically when --labels isn't given, so a deployment just drops a
# slm-labels.json next to this script and it's part of the standard bring-up.
_HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_LABELS = os.path.join(_HERE, "slm-labels.json")
# Dedup cache persisted here so a quick bridge restart mid relay-window still drops
# the second copy of a packet two gateways both uplinked. Only entries newer than
# --dedup-ttl matter, so this just bridges restarts shorter than that window.
DEFAULT_DEDUP_STATE = os.path.join(_HERE, ".slm-dedup-state.json")


def load_dedup_state(args):
    """Restore the recent (from, id) set from disk, dropping anything past the TTL."""
    seen = {}
    if not args.dedup_ttl or not args.dedup_state:
        return seen
    try:
        with open(args.dedup_state) as f:
            raw = json.load(f)
    except (FileNotFoundError, ValueError):
        return seen
    cutoff = time.time() - args.dedup_ttl
    for k, ts in raw.items():
        if ts >= cutoff:
            frm, pid = k.split(":")
            seen[(int(frm), int(pid))] = ts
    if seen:
        print(f"[dedup] restored {len(seen)} recent packet id(s) from "
              f"{os.path.relpath(args.dedup_state)}")
    return seen


def save_dedup_state(seen, path):
    """Atomically write the (from, id) -> last-seen map. Best-effort; never fatal."""
    if not path:
        return
    tmp = f"{path}.tmp"
    try:
        with open(tmp, "w") as f:
            json.dump({f"{frm}:{pid}": ts for (frm, pid), ts in seen.items()}, f)
        os.replace(tmp, path)
    except OSError:
        pass  # a debug bridge shouldn't die because the state file isn't writable


def load_labels(args):
    """Build a node-id -> human label map from a JSON file and/or inline --label flags.

    Keys are normalized to the `!aabbccdd` form used for `node`, so the file can use
    either `!a1b2c3d4` or `a1b2c3d4`. Inline --label entries win over the file. With
    no --labels flag, tools/slm-labels.json is auto-loaded if present.
    """
    def norm(k):
        k = k.strip().lower()
        return k if k.startswith("!") else "!" + k

    m = {}
    path = args.labels or (DEFAULT_LABELS if os.path.exists(DEFAULT_LABELS) else None)
    if path:
        with open(path) as f:
            for k, v in json.load(f).items():
                if k.startswith("_"):
                    continue  # _comment etc. -- documentation keys, not nodes
                m[norm(k)] = v
        print(f"[labels] loaded {os.path.relpath(path)}")
    for item in args.label or []:
        if "=" not in item:
            sys.exit(f"--label expects NODE=Name, got {item!r}")
        k, v = item.split("=", 1)
        m[norm(k)] = v
    if m:
        print(f"[labels] {len(m)} node label(s): " + ", ".join(f"{k}={v}" for k, v in m.items()))
    return m


def make_out(args):
    if not args.out_broker:
        return None
    import paho.mqtt.client as mqtt
    c = mqtt.Client()
    if args.out_user:
        c.username_pw_set(args.out_user, args.out_pass)
    c.connect(args.out_broker, args.out_port, 30)
    c.loop_start()
    print(f"[republish] -> {args.out_broker}:{args.out_port}  {args.out_root}/decoded/<node>")
    return c


def run_serial(args, out):
    import meshtastic.serial_interface
    from pubsub import pub

    def on_proxy(proxymessage=None, interface=None):
        decode_envelope(proxymessage.data, args, out)

    pub.subscribe(on_proxy, "meshtastic.mqttclientproxymessage")
    print(f"[serial] proxy host on {args.serial or '(auto)'} -- keep running for uplink to flow")
    iface = meshtastic.serial_interface.SerialInterface(devPath=args.serial)
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        iface.close()


def run_broker(args, out):
    import paho.mqtt.client as mqtt
    topic = f"{args.in_root}/2/e/+/+"

    def on_connect(c, u, flags, rc):
        c.subscribe(topic)
        print(f"[broker] subscribed {args.broker}:{args.broker_port}  {topic}")

    def on_message(c, u, msg):
        decode_envelope(msg.payload, args, out)

    c = mqtt.Client()
    if args.broker_user:
        c.username_pw_set(args.broker_user, args.broker_pass)
    c.on_connect = on_connect
    c.on_message = on_message
    c.connect(args.broker, args.broker_port, 30)
    try:
        c.loop_forever()
    except KeyboardInterrupt:
        pass


def main():
    ap = argparse.ArgumentParser(description="Decode SLM PRIVATE_APP spectrum frames from MQTT.")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--serial", nargs="?", const=None, metavar="PORT",
                     help="USB client-proxy mode (optional explicit port, else auto-detect)")
    src.add_argument("--broker", metavar="HOST", help="Broker-subscribe mode: gateway's broker host")
    ap.add_argument("--broker-port", type=int, default=1883)
    ap.add_argument("--broker-user"); ap.add_argument("--broker-pass")
    ap.add_argument("--in-root", default="TA/SLM", help="Gateway mqtt.root (broker mode), default TA/SLM")
    # republish target
    ap.add_argument("--out-broker", help="Republish decoded JSON to this broker (omit to disable)")
    ap.add_argument("--out-port", type=int, default=1883)
    ap.add_argument("--out-user"); ap.add_argument("--out-pass")
    ap.add_argument("--out-root", default="TA/SLM", help="Republish root, default TA/SLM -> TA/SLM/decoded/<node>")
    ap.add_argument("--quiet", action="store_true", help="No ASCII spectrum, just republish")
    # multi-node fan-in: dedup overlapping gateway uplinks, label nodes by ID
    ap.add_argument("--dedup-ttl", type=float, default=30.0, metavar="SECS",
                    help="Drop repeat (from,packet-id) frames seen within this window "
                         "(e.g. same broadcast relayed by 2 gateways). 0 disables. Default 30.")
    ap.add_argument("--dedup-state", default=DEFAULT_DEDUP_STATE, metavar="FILE",
                    help="Where to persist the dedup cache across restarts "
                         "(default tools/.slm-dedup-state.json). Empty string disables persistence.")
    ap.add_argument("--labels", metavar="FILE",
                    help='JSON map of node ID -> name, e.g. {"!4f4aece2": "Workshop"}. '
                         "Defaults to tools/slm-labels.json if present.")
    ap.add_argument("--label", action="append", metavar="NODE=Name",
                    help="Inline node label (repeatable); overrides --labels file")
    ap.add_argument("--retain", dest="retain", action="store_true", default=True,
                    help="Publish decoded frames with the MQTT retain flag (default on)")
    ap.add_argument("--no-retain", dest="retain", action="store_false",
                    help="Publish without retain (last frame won't be held by the broker)")
    # allow --serial to consume the next arg as a port when given as `--serial /dev/...`
    args, extra = ap.parse_known_args()
    if args.serial is None and extra and not extra[0].startswith("-"):
        args.serial = extra[0]

    args.label_map = load_labels(args)
    decode_envelope._seen = load_dedup_state(args)
    out = make_out(args)
    if args.broker:
        run_broker(args, out)
    else:
        run_serial(args, out)


if __name__ == "__main__":
    main()
