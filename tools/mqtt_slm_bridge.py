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
import argparse, json, struct, sys, time

from meshtastic.protobuf import mqtt_pb2, portnums_pb2

PRIVATE_APP = portnums_pb2.PortNum.PRIVATE_APP  # 256
# IEC base-10 1/3-octave centers, 31 bands. Index 17 == 1 kHz. Must match OctaveBank.
CENTERS = [20, 25, 31.5, 40, 50, 63, 80, 100, 125, 160, 200, 250, 315, 400, 500, 630,
           800, 1000, 1250, 1600, 2000, 2500, 3150, 4000, 5000, 6300, 8000, 10000,
           12500, 16000, 20000]


def fmt_hz(hz):
    return f"{hz/1000:g}k" if hz >= 1000 else f"{hz:g}"


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
    if len(p) != 35 or p[0] != 0x02:
        print(f"  !! {node}: not a v2 35-byte SLM frame (len={len(p)} ver={p[0] if p else '?'})")
        return
    nbands = p[1]
    window = struct.unpack_from("<H", p, 2)[0]
    levels = [p[4 + b] / 2.0 for b in range(nbands)]  # 0.5 dB/LSB
    peak = max(levels)
    peak_hz = CENTERS[levels.index(peak)]

    if not args.quiet:
        print(f"\n=== {node}  ch={env.channel_id}  window={window}s  "
              f"1kHz={levels[17]:.1f}dB  peak={peak:.1f}dB @ {fmt_hz(peak_hz)}Hz ===")
        for b in range(nbands):
            print(f"   {fmt_hz(CENTERS[b]):>6}Hz {levels[b]:5.1f} |{'#' * int(levels[b] / 2)}")

    if out is not None:
        out.publish(f"{args.out_root}/decoded/{node}",
                    json.dumps({"node": node, "channel": env.channel_id,
                                "window_s": window, "bands_hz": CENTERS,
                                "levels_db": levels, "peak_db": peak, "peak_hz": peak_hz}))
    decode_envelope.count = getattr(decode_envelope, "count", 0) + 1


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
    # allow --serial to consume the next arg as a port when given as `--serial /dev/...`
    args, extra = ap.parse_known_args()
    if args.serial is None and extra and not extra[0].startswith("-"):
        args.serial = extra[0]

    out = make_out(args)
    if args.broker:
        run_broker(args, out)
    else:
        run_serial(args, out)


if __name__ == "__main__":
    main()
