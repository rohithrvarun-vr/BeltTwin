"""
BeltTwin - live fault detector

Subscribes to conveyor2/telemetry, computes the SAME features as training
(imported from train_rf.py), runs the Random Forest, and when a prediction
holds for --consec samples in a row it emits an event:
  * printed to the console
  * published to MQTT topic conveyor2/detection (Unity can show it)
  * POSTed to Firebase Realtime Database if --firebase is given

Usage:
    python live_detector.py --model <path to rf_model.joblib> [--firebase https://....firebasedatabase.app]
"""
import argparse
import collections
import json
import time

import joblib
import numpy as np

from train_rf import LONG, SHORT, HZ, POS_WRAP, SIGNALS

GAP_MS = 1500     # a gap bigger than this breaks the window: start over
MAX_AGE_MS = 1000 # a sample older than this on arrival is buffered but not predicted (catch-up)


def _slope(x):
    """Least-squares slope per second, same closed form as train_rf.roll_slope."""
    n = len(x)
    i = np.arange(n, dtype=float)
    s_x, s_i, s_ix, s_ii = x.sum(), i.sum(), (i * x).sum(), (i * i).sum()
    return (n * s_ix - s_i * s_x) / (n * s_ii - s_i ** 2) * HZ


def fast_features(rows):
    """Features for the LAST sample only, numpy, no pandas.
    Must match train_rf.add_features exactly (checked by the parity test)."""
    ts = np.array([r["ts"] for r in rows], dtype=float)
    pos = np.array([r["rPosition"] for r in rows], dtype=float)
    spd = np.array([r["rSpeed"] for r in rows], dtype=float)
    belt = np.full(len(rows), np.nan)
    belt[1:] = (np.diff(pos) % POS_WRAP) / (np.diff(ts) / 1000.0)
    with np.errstate(divide="ignore", invalid="ignore"):
        ratio = np.where(spd > 1.0, belt / spd, np.nan)
    series = {s: (ratio if s == "belt_ratio" else np.array([r[s] for r in rows], dtype=float))
              for s in SIGNALS}
    f = {}
    for s, x in series.items():
        xs, xl = x[-SHORT:], x[-LONG:]
        m_s = xs.mean()
        f[f"{s}_mean_s"] = m_s
        f[f"{s}_std_s"] = xs.std(ddof=1)
        f[f"{s}_slope_s"] = _slope(xs)
        f[f"{s}_slope_l"] = _slope(xl)
        f[f"{s}_delta_sl"] = m_s - xl.mean()
    f["speed"] = spd[-1]
    return f


class Detector:
    def __init__(self, bundle, consec, emit):
        self.rf = bundle["model"]
        self.rf.n_jobs = 1                     # single-sample predict: threads only add latency
        self.feats = bundle["features"]
        self.consec = consec
        self.emit = emit
        self.buf = collections.deque(maxlen=LONG + 5)
        self.last_plc = None
        self.streak_label, self.streak_n = None, 0
        self.announced = None
        self.skipped = 0

    def reset(self, reason):
        self.buf.clear()
        self.streak_label, self.streak_n = None, 0
        if self.announced != reason:
            self.announced = reason
            self.emit({"event": reason})

    def handle(self, d, now_ms=None):
        if d["nPlcTime"] == self.last_plc:
            return None                        # duplicate sample
        self.last_plc = d["nPlcTime"]

        if d["eState"] != 2:
            self.reset("not_running")          # model is only valid while Running
            return None
        if self.buf and d["ts"] - self.buf[-1]["ts"] > GAP_MS:
            self.buf.clear()                   # data gap: window would be wrong
        self.buf.append(d)
        if len(self.buf) < LONG + 1:
            return None                        # not enough history yet
        if now_ms is not None and now_ms - d["ts"] > MAX_AGE_MS:
            self.skipped += 1                  # behind real time: keep the window, skip the prediction
            return None

        f = fast_features(list(self.buf)[-(LONG + 1):])
        x = np.array([[f[k] for k in self.feats]])
        if np.isnan(x).any():
            return None
        proba = self.rf.predict_proba(x)[0]
        i = proba.argmax()
        pred, conf = self.rf.classes_[i], float(proba[i])

        if pred == self.streak_label:
            self.streak_n += 1
        else:
            self.streak_label, self.streak_n = pred, 1
        if self.streak_n >= self.consec and pred != self.announced:
            self.announced = pred
            self.emit({"event": "prediction", "pred": pred, "confidence": round(conf, 3),
                       "speed": d["rSpeed"], "plc_ts": int(d["ts"]), "skipped": self.skipped})
        return pred


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True)
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--consec", type=int, default=3)
    ap.add_argument("--firebase", default=None, help="Realtime Database URL, no trailing slash")
    args = ap.parse_args()

    import paho.mqtt.client as mqtt
    session = None
    if args.firebase:
        import requests
        session = requests.Session()

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    TOPIC = "conveyor2/detection"
    # if this process dies without saying goodbye, the broker announces it for us
    client.will_set(TOPIC, json.dumps({"event": "detector_offline"}), retain=True)

    def emit(ev):
        ev["ts"] = int(time.time() * 1000)
        print(time.strftime("%H:%M:%S"), json.dumps(ev), flush=True)
        info = client.publish(TOPIC, json.dumps(ev), retain=True)   # retained: late subscribers get current state
        if session:
            try:
                r = session.post(args.firebase + "/detections.json", json=ev, timeout=3)
                if r.status_code != 200:
                    print("  firebase HTTP", r.status_code, r.text[:200], flush=True)
            except Exception as e:
                print("  firebase error:", e, flush=True)
        return info

    det = Detector(joblib.load(args.model), args.consec, emit)

    def on_connect(c, userdata, flags, reason_code, properties):
        print("mqtt connected:", reason_code, flush=True)
        c.subscribe("conveyor2/telemetry")
        det.announced = None               # re-announce current state after a (re)connect
        emit({"event": "detector_online"})

    stats = {"n": 0, "sum": 0.0, "max": 0.0}

    def on_message(c, userdata, msg):
        t0 = time.perf_counter()
        try:
            d = json.loads(msg.payload)
            det.handle(d, now_ms=time.time() * 1000.0)
        except Exception as e:
            print("handle error:", repr(e), flush=True)
            return
        dt = (time.perf_counter() - t0) * 1000.0
        stats["n"] += 1; stats["sum"] += dt; stats["max"] = max(stats["max"], dt)
        if stats["n"] >= 240:                  # every ~60 s: health line, so falling behind is visible
            lag = time.time() * 1000.0 - d["ts"]
            print(time.strftime("%H:%M:%S"), f"stats: handle avg {stats['sum'] / stats['n']:.1f} ms, "
                  f"max {stats['max']:.1f} ms, lag {lag:.0f} ms, skipped total {det.skipped}", flush=True)
            stats.update(n=0, sum=0.0, max=0.0)

    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(args.host, 1883, keepalive=60)
    print("model classes:", list(det.rf.classes_), "| waiting for telemetry...", flush=True)
    try:
        client.loop_forever(retry_first_connection=True)
    except KeyboardInterrupt:
        pass
    finally:
        info = emit({"event": "detector_offline"})
        client.loop_start()
        try:
            info.wait_for_publish(timeout=2)
        except Exception:
            pass
        client.disconnect()
        client.loop_stop()
        print("detector stopped", flush=True)


if __name__ == "__main__":
    main()
