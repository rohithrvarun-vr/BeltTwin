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
import pandas as pd

from train_rf import add_features, LONG

GAP_MS = 1500   # a gap bigger than this breaks the window: start over


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

    def reset(self, reason):
        self.buf.clear()
        self.streak_label, self.streak_n = None, 0
        if self.announced != reason:
            self.announced = reason
            self.emit({"event": reason})

    def handle(self, d):
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

        f = add_features(pd.DataFrame(list(self.buf)))
        x = f[self.feats].iloc[[-1]]
        if x.isna().any(axis=1).iloc[0]:
            return None
        proba = self.rf.predict_proba(x.values)[0]
        i = proba.argmax()
        pred, conf = self.rf.classes_[i], float(proba[i])

        if pred == self.streak_label:
            self.streak_n += 1
        else:
            self.streak_label, self.streak_n = pred, 1
        if self.streak_n == self.consec and pred != self.announced:
            self.announced = pred
            self.emit({"event": "prediction", "pred": pred, "confidence": round(conf, 3),
                       "speed": d["rSpeed"], "plc_ts": d["ts"]})
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

    def emit(ev):
        ev["ts"] = int(time.time() * 1000)
        print(time.strftime("%H:%M:%S"), json.dumps(ev), flush=True)
        client.publish("conveyor2/detection", json.dumps(ev))
        if session:
            try:
                r = session.post(args.firebase + "/detections.json", json=ev, timeout=3)
                if r.status_code != 200:
                    print("  firebase HTTP", r.status_code, r.text[:200], flush=True)
            except Exception as e:
                print("  firebase error:", e, flush=True)

    det = Detector(joblib.load(args.model), args.consec, emit)

    def on_connect(c, userdata, flags, reason_code, properties):
        print("mqtt connected:", reason_code, flush=True)
        c.subscribe("conveyor2/telemetry")

    def on_message(c, userdata, msg):
        try:
            det.handle(json.loads(msg.payload))
        except Exception as e:
            print("handle error:", repr(e), flush=True)

    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(args.host, 1883, keepalive=30)
    print("model classes:", list(det.rf.classes_), "| waiting for telemetry...", flush=True)
    client.loop_forever(retry_first_connection=True)


if __name__ == "__main__":
    main()
