"""Shared SLM v2 frame decode — the single source of truth for both bridges.

Imported by mqtt_slm_bridge.py (MQTT / ServiceEnvelope path) and foh_client_bridge.py
(client-API / sendToPhone path). Both transports carry the identical 35-byte v2 frame,
so the band table and parser live here once. If the firmware's OctaveBank band set ever
changes, update CENTERS here and nowhere else.

Wire format v2 (35 bytes), see ../src/modules/SoundLevel/README.md:
  [0]     version = 0x02
  [1]     nBands  = 31 (fixed IEC base-10 1/3-octave set)
  [2..3]  window_seconds, uint16 little-endian (sub-second FoH frames round to 0)
  [4..34] per-band Leq, uint8 each, 0.5 dB/LSB
"""
import struct

# IEC base-10 1/3-octave centers, 31 bands. Index 17 == 1 kHz. MUST match OctaveBank.
CENTERS = [20, 25, 31.5, 40, 50, 63, 80, 100, 125, 160, 200, 250, 315, 400, 500, 630,
           800, 1000, 1250, 1600, 2000, 2500, 3150, 4000, 5000, 6300, 8000, 10000,
           12500, 16000, 20000]


def fmt_hz(hz):
    """Compact frequency label, e.g. 1000 -> '1k', 31.5 -> '31.5'."""
    return f"{hz/1000:g}k" if hz >= 1000 else f"{hz:g}"


def parse_v2(payload):
    """Decode a 35-byte v2 SLM frame into a dict, or return None if it isn't one.

    Returns: {window_s, bands_hz, levels_db, peak_db, peak_hz}. The dict matches the
    decoded-JSON contract the dashboard consumes (minus node/label/channel, which the
    caller adds from the transport envelope).
    """
    if len(payload) != 35 or payload[0] != 0x02:
        return None
    nbands = payload[1]
    levels = [payload[4 + b] / 2.0 for b in range(nbands)]   # 0.5 dB/LSB
    peak = max(levels)
    return {"window_s": struct.unpack_from("<H", payload, 2)[0],
            "bands_hz": CENTERS[:nbands], "levels_db": levels,
            "peak_db": peak, "peak_hz": CENTERS[levels.index(peak)]}
