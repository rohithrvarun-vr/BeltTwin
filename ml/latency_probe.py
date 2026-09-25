"""
BeltTwin - end-to-end latency probe, PLC scan -> subscriber.

nPlcTime is Windows FILETIME in ms (since 1601-01-01 UTC) modulo 2^32, i.e. the
PLC-side wall clock at the scan that produced the sample. Converting readTs/ts
(Unix ms, Node-RED) to the same scale gives the age of the PLC value at each hop.

Only listens to MQTT; safe to run during data collection.
Usage: python latency_probe.py [--n 240] [--host localhost]
"""
import argparse
import json
import statistics
import time

import paho.mqtt.client as mqtt

EPOCH_OFFSET_MS = 11644473600000   # 1601-01-01 -> 1970-01-01
WRAP = 2 ** 32


def plc_age(unix_ms, nplc):
    """ms between the PLC scan (nPlcTime) and a Unix-ms timestamp, wrap-safe."""
    d = ((int(unix_ms) + EPOCH_OFFSET_MS) - int(nplc)) % WRAP
    return d - WRAP if d > WRAP // 2 else d


def pct(v, q):
    s = sorted(v)
    return s[min(len(s) - 1, int(len(s) * q))]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=240)
    ap.add_argument("--host", default="localhost")
    a = ap.parse_args()

    rows, last = [], None

    def on_message(c, u, msg):
        nonlocal last
        now = time.time() * 1000.0
        d = json.loads(msg.payload)
        if d.get("nPlcTime") == last or "readTs" not in d:
            return
        last = d["nPlcTime"]
        rows.append((plc_age(d["readTs"], d["nPlcTime"]),     # PLC scan -> OPC UA read done
                     d["ts"] - d["readTs"],                    # read -> Node-RED publish
                     now - d["ts"],                            # publish -> this subscriber
                     plc_age(now, d["nPlcTime"])))             # PLC scan -> subscriber
        if len(rows) % 40 == 0:
            print(f"  {len(rows)}/{a.n} samples", flush=True)
        if len(rows) >= a.n:
            c.disconnect()

    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    c.on_connect = lambda c, u, f, rc, p: c.subscribe("conveyor2/telemetry")
    c.on_message = on_message
    c.connect(a.host, 1883, 30)
    print(f"collecting {a.n} samples (~{a.n / 4:.0f} s at 4 Hz)...", flush=True)
    c.loop_forever()

    names = ["PLC scan -> OPC UA read", "read -> Node-RED publish",
             "publish -> subscriber", "TOTAL PLC scan -> subscriber"]
    print(f"\n{'hop':32s} {'min':>7s} {'median':>7s} {'p95':>7s} {'max':>7s}   (ms)")
    for i, name in enumerate(names):
        v = [r[i] for r in rows]
        print(f"{name:32s} {min(v):7.0f} {statistics.median(v):7.0f} {pct(v, 0.95):7.0f} {max(v):7.0f}")

    if min(r[0] for r in rows) < 0:
        print("\nWARNING: negative PLC->read age: TwinCAT and Windows clocks are not aligned. "
              "Absolute PLC-side numbers are NOT valid.")
    else:
        print("\nClock check passed (no negative ages). nPlcTime resolution is 10 ms (one PLC cycle).")


if __name__ == "__main__":
    main()
