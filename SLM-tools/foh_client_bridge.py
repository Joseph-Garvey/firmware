#!/usr/bin/env python3
"""SLM front-of-house client bridge / decoder.

The companion to mqtt_slm_bridge.py for the FRONT-OF-HOUSE path. Where that tool
pulls SLM frames out of MQTT (ServiceEnvelope), this one pulls them straight off a
node's **client API** (USB serial / BLE / TCP) — the high-rate stream the firmware
emits via MeshService::sendToPhone() when built with -DSLM_FOH_STREAM (see
../src/modules/SoundLevel/README.md, "Front-of-house live streaming").

Those frames never touch LoRa, so they arrive at whatever rate the node is
configured for (SLM_BASE_INTERVAL_MS: 125 ms = 8 Hz "fast", 1000 ms = 1 Hz "slow")
rather than once every few minutes. The payload is the identical 35-byte v2 frame,
so the decode below is the same as the MQTT bridge's.

It prints an ASCII spectrum and (optionally) republishes the same decoded JSON the
MQTT bridge does — to TA/SLM/decoded/<node> on a local broker — so the existing
tools/slm_dashboard.html visualizes the live FoH feed unchanged.

Examples:
  # USB serial, auto-detect port, ASCII spectrum only
  python foh_client_bridge.py --serial

  # USB serial + republish to a LOCAL broker so slm_dashboard.html shows it live
  python foh_client_bridge.py --serial /dev/cu.usbmodemXXXX \
      --out-broker 127.0.0.1 --out-user slm --out-pass slmdebug123

  # BLE (pair the node first) or TCP (WiFi-joined node)
  python foh_client_bridge.py --ble  AA:BB:CC:DD:EE:FF
  python foh_client_bridge.py --tcp  10.0.0.42
"""
import argparse, json, time

from meshtastic.protobuf import portnums_pb2
from pubsub import pub

from slm_frame import CENTERS, fmt_hz, parse_v2   # shared v2 decode (also used by mqtt bridge)

PRIVATE_APP = portnums_pb2.PortNum.PRIVATE_APP  # 256


# pypubsub keeps only a WEAK reference to a subscribed listener, so the callback has to be
# a module-level function that stays alive here. An inline lambda/closure passed straight to
# subscribe() has no other reference, gets garbage-collected right after subscribe() returns,
# and then silently never fires (the node streams, but nothing is decoded). args/out are
# stashed as module globals so the bare function can reach them.
_ARGS = None
_OUT = None


def on_receive(packet=None, interface=None):
    """pubsub callback for every packet the node hands up the client API."""
    args, out = _ARGS, _OUT
    dec = packet.get("decoded") if isinstance(packet, dict) else None
    if not dec:
        return  # encrypted / undecodable — FoH frames are always local & cleartext
    # portnum may be the enum name "PRIVATE_APP" or the raw int 256 depending on lib version
    if dec.get("portnum") not in ("PRIVATE_APP", PRIVATE_APP):
        return
    payload = dec.get("payload")
    if not isinstance(payload, (bytes, bytearray)):
        return
    frame = parse_v2(bytes(payload))
    node = f"!{packet.get('from', 0):08x}"
    if frame is None:
        print(f"  !! {node}: not a v2 35-byte SLM frame (len={len(payload)})")
        return
    chan = packet.get("channel", 0)

    if not args.quiet:
        f = frame
        print(f"\n=== {node}  ch={chan}  window={f['window_s']}s  "
              f"1kHz={f['levels_db'][17]:.1f}dB  peak={f['peak_db']:.1f}dB "
              f"@ {fmt_hz(f['peak_hz'])}Hz ===")
        for b in range(len(f["bands_hz"])):
            print(f"   {fmt_hz(CENTERS[b]):>6}Hz {f['levels_db'][b]:5.1f} "
                  f"|{'#' * int(f['levels_db'][b] / 2)}")

    if out is not None:
        # Same topic + JSON shape as mqtt_slm_bridge.py so slm_dashboard.html (and the
        # FoH app, if it reads this contract) consume the FoH feed with no changes.
        out.publish(f"{args.out_root}/decoded/{node}",
                    json.dumps({"node": node, "label": None, "channel": chan,
                                **frame}),
                    retain=args.retain)


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


def open_interface(args):
    if args.ble is not None:
        import meshtastic.ble_interface
        print(f"[ble] connecting {args.ble or '(first found)'}")
        return meshtastic.ble_interface.BLEInterface(args.ble)
    if args.tcp is not None:
        import meshtastic.tcp_interface
        print(f"[tcp] connecting {args.tcp}")
        return meshtastic.tcp_interface.TCPInterface(hostname=args.tcp)
    import meshtastic.serial_interface
    print(f"[serial] connecting {args.serial or '(auto-detect)'}")
    return meshtastic.serial_interface.SerialInterface(devPath=args.serial)


def main():
    ap = argparse.ArgumentParser(description="Decode SLM PRIVATE_APP frames off a node's client API.")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--serial", nargs="?", const=None, metavar="PORT",
                     help="USB serial (optional explicit port, else auto-detect)")
    src.add_argument("--ble", nargs="?", const="", metavar="ADDR",
                     help="BLE (optional MAC/name; else first SLM node found)")
    src.add_argument("--tcp", metavar="HOST", help="TCP to a WiFi-joined node")
    ap.add_argument("--quiet", action="store_true", help="No ASCII spectrum, just republish")
    ap.add_argument("--out-broker", help="Republish decoded JSON to this broker (omit to disable)")
    ap.add_argument("--out-port", type=int, default=1883)
    ap.add_argument("--out-user"); ap.add_argument("--out-pass")
    ap.add_argument("--out-root", default="TA/SLM", help="Republish root, default TA/SLM")
    ap.add_argument("--retain", dest="retain", action="store_true", default=True)
    ap.add_argument("--no-retain", dest="retain", action="store_false")
    args, extra = ap.parse_known_args()
    if args.serial is None and extra and not extra[0].startswith("-"):
        args.serial = extra[0]

    global _ARGS, _OUT
    _ARGS = args
    _OUT = make_out(args)
    pub.subscribe(on_receive, "meshtastic.receive")
    iface = open_interface(args)
    print("[ready] streaming FoH frames -- Ctrl-C to stop")
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        iface.close()


if __name__ == "__main__":
    main()
