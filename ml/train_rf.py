"""
BeltTwin - Random Forest fault classifier (Phase 1)

Usage:
    python train_rf.py <data_dir> [--exclude wear] [--consec 3]

<data_dir> must contain manifest.csv and the run_*.csv files it lists.

Design rules (do not break these):
  * Features come ONLY from sensors, rSpeed and belt motion. Never eState,
    eFaultCode, counters, phase or label.
  * Features are causal: each sample only uses its own past (trailing windows).
  * Cross-validation is grouped by run, never by window.
  * The true trip is the first row with eState == 3, not the runner's phase.
"""
import argparse
import os
import sys

import numpy as np
import pandas as pd
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import classification_report, confusion_matrix
from sklearn.model_selection import GroupKFold
import joblib

HZ = 4.0            # sample rate
SHORT = 8           # 2 s window  (fast faults: jam trips in ~2 s)
LONG = 120          # 30 s window (slow faults: thermal trends)
POS_WRAP = 1000.0
SIGNALS = ["rMotorCurrent", "rMotorTemp", "rBearingTemp", "rVibration", "belt_ratio"]


def roll_slope(x: pd.Series, n: int) -> pd.Series:
    """Trailing least-squares slope per second, closed form (fast)."""
    i = pd.Series(np.arange(len(x), dtype=float), index=x.index)
    s_x, s_i = x.rolling(n).sum(), i.rolling(n).sum()
    s_ix, s_ii = (x * i).rolling(n).sum(), (i * i).rolling(n).sum()
    return (n * s_ix - s_i * s_x) / (n * s_ii - s_i ** 2) * HZ


def load_run(path: str, run: pd.Series) -> pd.DataFrame:
    df = pd.read_csv(path)
    df = df.sort_values("ts").drop_duplicates("nPlcTime").reset_index(drop=True)

    # belt motion from position (unwrap the 1000 wrap), relative to commanded speed
    dpos = df["rPosition"].diff() % POS_WRAP
    dt = df["ts"].diff() / 1000.0
    belt = dpos / dt
    df["belt_ratio"] = np.where(df["rSpeed"] > 1.0, belt / df["rSpeed"], np.nan)

    for s in SIGNALS:
        x = df[s].astype(float)
        df[f"{s}_mean_s"] = x.rolling(SHORT).mean()
        df[f"{s}_std_s"] = x.rolling(SHORT).std()
        df[f"{s}_slope_s"] = roll_slope(x, SHORT)
        df[f"{s}_slope_l"] = roll_slope(x, LONG)
        df[f"{s}_delta_sl"] = df[f"{s}_mean_s"] - x.rolling(LONG).mean()
    df["speed"] = df["rSpeed"]

    # true trip = first eState==3 row; relabel from there, not from the runner
    trip = df.index[df["eState"] == 3]
    trip_i = trip[0] if len(trip) else len(df)
    before_trip = df.index < trip_i

    target = pd.Series(pd.NA, index=df.index, dtype="object")
    target[(df["phase"] == "healthy") & before_trip] = "healthy"
    target[(df["phase"] == "developing") & before_trip] = run["kind"]
    df["target"] = target

    keep = df["target"].notna() & (df["eState"] == 2)
    df = df[keep].copy()
    df["run_id"] = run["run_id"]
    df["kind"] = run["kind"]
    df["inject_ts"] = run["inject_ts"]
    return df


def feature_cols(df):
    return [c for c in df.columns
            if c.endswith(("_mean_s", "_std_s", "_slope_s", "_slope_l", "_delta_sl"))
            or c == "speed"]


def first_consecutive(mask: np.ndarray, k: int) -> int:
    """Index of the first position where mask is True k times in a row, or -1."""
    run = 0
    for i, m in enumerate(mask):
        run = run + 1 if m else 0
        if run >= k:
            return i - k + 1
    return -1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("data_dir")
    ap.add_argument("--exclude", nargs="*", default=[], help="fault kinds to drop entirely")
    ap.add_argument("--consec", type=int, default=3, help="consecutive predictions to count a detection")
    args = ap.parse_args()

    man = pd.read_csv(os.path.join(args.data_dir, "manifest.csv"))
    man = man[man["run_id"] != "run_id"]            # repeated header lines
    bad = man[man["status"] != "ok"]
    if len(bad):
        print(f"skipping {len(bad)} runs with status != ok:")
        print(bad[["run_id", "kind", "status"]].to_string(index=False))
    man = man[(man["status"] == "ok") & (~man["kind"].isin(args.exclude))]

    frames = []
    for _, run in man.iterrows():
        p = os.path.join(args.data_dir, run["run_id"] + ".csv")
        if not os.path.exists(p):
            print("MISSING file for", run["run_id"]); continue
        frames.append(load_run(p, run))
    if not frames:
        sys.exit("no usable runs")
    data = pd.concat(frames, ignore_index=True)
    feats = feature_cols(data)
    data = data.dropna(subset=feats)

    print(f"\nruns: {data['run_id'].nunique()}   samples: {len(data)}   features: {len(feats)}")
    print(data.groupby("target").size().rename("samples").to_string())

    groups = data["run_id"].values
    n_groups = len(np.unique(groups))
    if n_groups < 3:
        print("\nfewer than 3 runs: pipeline smoke test only, no evaluation.")
        return

    X, y = data[feats].values, data["target"].values
    rf = RandomForestClassifier(n_estimators=300, min_samples_leaf=5,
                                class_weight="balanced", n_jobs=-1, random_state=0)

    # ---- out-of-fold predictions, grouped by run ----
    oof = np.empty(len(y), dtype=object)
    for tr, te in GroupKFold(n_splits=min(5, n_groups)).split(X, y, groups):
        rf.fit(X[tr], y[tr])
        oof[te] = rf.predict(X[te])
    data["pred"] = oof

    labels = sorted(np.unique(y))
    print("\n=== sample-level (grouped 5-fold CV) ===")
    print(classification_report(y, oof, labels=labels, zero_division=0))
    print(pd.DataFrame(confusion_matrix(y, oof, labels=labels),
                       index=["true " + l for l in labels], columns=labels).to_string())

    # ---- run-level: detection delay and false alarms ----
    print(f"\n=== run-level (detection = {args.consec} consecutive predictions) ===")
    rows = []
    for rid, g in data.groupby("run_id"):
        kind = g["kind"].iloc[0]
        h = g[g["target"] == "healthy"]
        fa = first_consecutive((h["pred"] != "healthy").values, args.consec) >= 0
        rec = {"run_id": rid, "kind": kind, "false_alarm": fa}
        if not kind.startswith("healthy"):
            d = g[g["target"] == kind]
            i = first_consecutive((d["pred"] == kind).values, args.consec)
            rec["detected"] = i >= 0
            rec["delay_s"] = (d["ts"].iloc[i] - float(d["inject_ts"].iloc[0])) / 1000.0 if i >= 0 else np.nan
            wrong = d["pred"][d["pred"] != "healthy"]
            rec["most_common_wrong"] = wrong[wrong != kind].mode().iloc[0] if (wrong != kind).any() else ""
        rows.append(rec)
    r = pd.DataFrame(rows)

    faults = r[~r["kind"].str.startswith("healthy")]
    if len(faults):
        summ = faults.groupby("kind").agg(runs=("run_id", "count"),
                                          detected=("detected", "sum"),
                                          median_delay_s=("delay_s", "median"),
                                          max_delay_s=("delay_s", "max"))
        print(summ.to_string())
    print(f"\nruns with a false alarm in their healthy segment: "
          f"{int(r['false_alarm'].sum())} / {len(r)}")
    r.to_csv(os.path.join(args.data_dir, "rf_run_results.csv"), index=False)

    # ---- final model on all data, for the live detector ----
    rf.fit(X, y)
    imp = pd.Series(rf.feature_importances_, index=feats).sort_values(ascending=False)
    print("\ntop features:\n" + imp.head(10).to_string())
    out = os.path.join(args.data_dir, "rf_model.joblib")
    joblib.dump({"model": rf, "features": feats, "short": SHORT, "long": LONG,
                 "excluded": args.exclude}, out)
    print("\nsaved", out)


if __name__ == "__main__":
    main()