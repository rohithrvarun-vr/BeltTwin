"""BeltTwin Phase 2, step 5: four detectors, tuned on TRAIN runs only.

Usage:  python ml/detectors.py data/v3
Needs:  ml/split_v3.csv, ml/behaviour_v3.json, data/v3/residuals/ (from behaviour_model.py)

Detectors (each reduces to one statistic per sample and one threshold):
  fixed   static, speed-independent limits on MEASURED signals (current, motor T, bearing T,
          vibration high; 2 s belt ratio low), like PLC alarms. Limits at the train-healthy
          99.9th percentile, scaled by one common factor; 1 s debounce (4 samples).
  cusum   two-sided CUSUM on residual z-scores (current, vib_lc, ratio_lc, motor_lc, bearing_lc),
          k = 0.5, z clipped to +-4 against spikes; statistic = largest CUSUM sum.
          CUSUM assumes independent samples: channels whose healthy lag-1 autocorrelation exceeds
          0.2 are pre-whitened with an AR(1) model fitted on train healthy residuals.
  iforest Isolation Forest on residual features (2 s and 30 s means of 9 z channels), trained
          on healthy samples only; 3-sample debounce.
  rf      Random Forest on raw-signal features (Phase 1 design, 31 features), classes healthy /
          jam / slip / overload; statistic = 1 - P(healthy); 3-sample debounce.

Rules (fixed before any result):
  - every threshold is tuned on train runs only, to the SAME false-alarm budget:
    FA_PER_HOUR alarm events per hour of healthy, motor-on train data
  - iforest and rf statistics on train runs are out-of-fold (same 4 folds as the behaviour model)
  - an alarm event = upward crossing of the threshold while the motor is on; it re-arms when the
    statistic falls back below the threshold
  - detection = first upward crossing at or after the injection sample (first nInjectCount
    increment) and before the trip; delay measured on the PLC clock
  - wear is not in train; the test set is not touched here (step 6)
Outputs: ml/detectors_v3.json, ml/iforest_v3.joblib, ml/rf_v3.joblib, train report on stdout.
"""
import sys, os, json
import numpy as np
import pandas as pd
import joblib
from sklearn.ensemble import IsolationForest, RandomForestClassifier

HERE = os.path.dirname(os.path.abspath(__file__))
FA_PER_HOUR = 0.5
DT = 0.25
W_SHORT, W_LONG = 8, 120            # 2 s and 30 s in samples
FOLD_SEED, N_FOLDS = 20260930, 4
CUSUM_K, CUSUM_CLIP, WHITEN_ABOVE = 0.5, 4.0, 0.2
CUSUM_CH = ["z_current", "z_vib_lc", "z_ratio_lc", "z_motor_lc", "z_bearing_lc"]
IF_CH = ["z_current", "z_motor", "z_bearing", "z_vib", "z_ratio", "z_motor_lc", "z_bearing_lc", "z_vib_lc", "z_ratio_lc"]
RAW_CH = ["rMotorCurrent", "rMotorTemp", "rBearingTemp", "rVibration", "belt_ratio", "rSpeed"]
FIXED_HIGH = ["rMotorCurrent", "rMotorTemp", "rBearingTemp", "rVibration"]
DEBOUNCE = {"fixed": 4, "cusum": 1, "iforest": 3, "rf": 3}
CLASSES = {"healthy": 0, "jam": 1, "slip": 2, "overload": 4}


# ---------------------------------------------------------------- data
def load(data_dir, split, P):
    runs = {}
    for _, row in split.iterrows():
        rid = row.run_id
        raw = pd.read_csv(os.path.join(data_dir, rid + ".csv")).sort_values("nPlcTime").reset_index(drop=True)
        res = pd.read_csv(os.path.join(data_dir, "residuals", rid + ".csv"))
        d = pd.concat([raw[["nPlcTime", "eState", "rSpeed", "rMotorCurrent", "rMotorTemp", "rBearingTemp",
                            "rVibration", "nInjectCount"]],
                       res[["healthy", "t_since_start", "ratio"] + IF_CH]], axis=1)
        d["belt_ratio"] = d["ratio"] + P["ratio"]
        d["on"] = d.eState.isin([1, 2]).values
        inj = np.flatnonzero(d.nInjectCount.values > d.nInjectCount.values[0])
        trip = np.flatnonzero(d.eState.values == 3)
        d.attrs.update(run_id=rid, kind=row.kind, split=row.split,
                       inject=int(inj[0]) if len(inj) else -1, trip=int(trip[0]) if len(trip) else -1)
        runs[rid] = d
    return runs


def folds_of(keys):
    keys = sorted(keys)
    perm = np.random.default_rng(FOLD_SEED).permutation(len(keys))
    return [set(np.array(keys)[perm[i::N_FOLDS]]) for i in range(N_FOLDS)]


# ---------------------------------------------------------------- causal rolling helpers
def roll_mean(x, w):
    return pd.Series(x).rolling(w, min_periods=1).mean().values


def roll_std(x, w):
    return pd.Series(x).rolling(w, min_periods=2).std().fillna(0).values


def roll_slope(x, w):
    """Least-squares slope per second over the last w samples (fewer at the start)."""
    x = np.asarray(x, float)
    n = len(x)
    i = np.arange(n, dtype=float)
    def csum(a):
        c = np.cumsum(a)
        lag = np.r_[np.zeros(w), c[:-w]] if n > w else np.zeros(n)
        return c - lag[:n]
    cnt = np.minimum(np.arange(1, n + 1), w).astype(float)
    S1, S2, Sx, Sxi = csum(i), csum(i * i), csum(x), csum(i * x)
    den = cnt * S2 - S1 * S1
    with np.errstate(invalid="ignore", divide="ignore"):
        s = np.where(den > 0, (cnt * Sxi - S1 * Sx) / den, 0.0)
    return s / DT


def debounce(stat, n):
    if n <= 1:
        return stat
    return pd.Series(stat).rolling(n, min_periods=n).min().fillna(0).values


# ---------------------------------------------------------------- features
def rf_features(d, P):
    cols = {}
    for ch in RAW_CH:
        x = d[ch].values.astype(float)
        if ch == "belt_ratio":
            x = np.where(np.isnan(x), P["ratio"], x)
        m2, m30 = roll_mean(x, W_SHORT), roll_mean(x, W_LONG)
        cols[ch + "_m2"] = m2
        cols[ch + "_s2"] = roll_std(x, W_SHORT)
        cols[ch + "_k2"] = roll_slope(x, W_SHORT)
        cols[ch + "_k30"] = roll_slope(x, W_LONG)
        cols[ch + "_d"] = m2 - m30
    cols["rSpeed"] = d.rSpeed.values
    return pd.DataFrame(cols)


def if_features(d):
    cols = {}
    for ch in IF_CH:
        x = np.nan_to_num(d[ch].values.astype(float))
        cols[ch + "_m2"] = roll_mean(x, W_SHORT)
        cols[ch + "_m30"] = roll_mean(x, W_LONG)
    return pd.DataFrame(cols)


def rf_labels(d):
    y = np.full(len(d), -1)
    on = d.on.values
    y[on & d.healthy.values] = 0
    a = d.attrs
    if a["kind"] in CLASSES and a["kind"] != "healthy" and a["inject"] >= 0 and a["trip"] > a["inject"]:
        seg = np.zeros(len(d), bool)
        seg[a["inject"]:a["trip"]] = True
        y[seg & on] = CLASSES[a["kind"]]
    return y


# ---------------------------------------------------------------- statistics
def stat_fixed(d, L):
    s = np.zeros(len(d))
    for ch in FIXED_HIGH:
        s = np.maximum(s, d[ch].values / L[ch])
    r = d.belt_ratio.values
    s = np.maximum(s, np.where(np.isnan(r), 0.0, L["belt_ratio"] / np.where(r > 0, r, 1e-9)))
    return np.where(d.on.values, s, 0.0)


def ar1_phi(runs):
    """Lag-1 autocorrelation of each CUSUM channel on train healthy, motor-on samples."""
    phi = {}
    for ch in CUSUM_CH:
        a, b = [], []
        for d in runs.values():
            z = d[ch].values
            h = d.on.values & d.healthy.values
            ok = h[1:] & h[:-1] & ~np.isnan(z[1:]) & ~np.isnan(z[:-1])
            a.append(z[:-1][ok]); b.append(z[1:][ok])
        phi[ch] = float(np.corrcoef(np.concatenate(a), np.concatenate(b))[0, 1])
    return phi


def whiten(d, phi):
    Z = {}
    for ch in CUSUM_CH:
        z = d[ch].values.astype(float)
        p = phi[ch]
        if p > WHITEN_ABOVE:
            prev = np.r_[np.nan, z[:-1]]
            e = (z - p * prev) / np.sqrt(1 - p * p)
            z = np.where(np.isnan(prev), np.nan, e)
        Z[ch] = z
    return pd.DataFrame(Z)


def stat_cusum(d, phi):
    on = d.on.values
    Z = np.clip(np.nan_to_num(whiten(d, phi).values), -CUSUM_CLIP, CUSUM_CLIP)
    hi = np.zeros(Z.shape[1]); lo = np.zeros(Z.shape[1])
    out = np.zeros(len(d))
    for t in range(len(d)):
        if not on[t]:
            hi[:] = 0; lo[:] = 0
            continue
        hi = np.maximum(0.0, hi + Z[t] - CUSUM_K)
        lo = np.maximum(0.0, lo - Z[t] - CUSUM_K)
        out[t] = max(hi.max(), lo.max())
    return out


def stat_model(d, score):
    return np.where(d.on.values, score, 0.0)


# ---------------------------------------------------------------- events, tuning, evaluation
def crossings(stat, thr, on):
    above = stat >= thr
    up = above & ~np.r_[False, above[:-1]]
    return np.flatnonzero(up & on)


def fa_count(runs, stats, thr):
    n = 0
    for rid, d in runs.items():
        h = d.on.values & d.healthy.values
        idx = crossings(stats[rid], thr, d.on.values)
        n += int(h[idx].sum())
    return n


def healthy_hours(runs):
    return sum((d.on.values & d.healthy.values).sum() for d in runs.values()) * DT / 3600


def tune(runs, stats):
    hours = healthy_hours(runs)
    allowed = int(np.floor(FA_PER_HOUR * hours))
    pool = np.concatenate([stats[r][runs[r].on.values & runs[r].healthy.values] for r in runs])
    lo, hi = np.quantile(pool, 0.9), pool.max() * 1.0001 + 1e-9
    grid = np.unique(np.r_[np.quantile(pool, np.linspace(0.9, 1, 300)), np.linspace(lo, hi, 300)])
    thr = hi
    for g in grid[::-1]:                 # walk down until the budget is exceeded
        if fa_count(runs, stats, g) > allowed:
            break
        thr = g
    return float(thr), allowed, hours


def evaluate(runs, stats, thr):
    rows = []
    for rid, d in runs.items():
        a = d.attrs
        if a["inject"] < 0 or a["trip"] < 0:
            continue
        idx = crossings(stats[rid], thr, d.on.values)
        hit = idx[(idx >= a["inject"]) & (idx < a["trip"])]
        t_inj = d.nPlcTime.values[a["inject"]]
        ttf = (d.nPlcTime.values[a["trip"]] - t_inj) / 1000
        if len(hit):
            delay = (d.nPlcTime.values[hit[0]] - t_inj) / 1000
            rows.append(dict(kind=a["kind"], detected=True, delay=delay, frac=delay / ttf, ttf=ttf))
        else:
            rows.append(dict(kind=a["kind"], detected=False, delay=np.nan, frac=np.nan, ttf=ttf))
    return pd.DataFrame(rows)


# ---------------------------------------------------------------- main
def main(data_dir):
    split = pd.read_csv(os.path.join(HERE, "split_v3.csv"))
    P = json.load(open(os.path.join(HERE, "behaviour_v3.json")))
    train_ids = list(split[split.split == "train"].run_id)
    runs = load(data_dir, split[split.split == "train"], P)
    print(f"train runs: {len(runs)}, healthy motor-on hours: {healthy_hours(runs):.2f}")

    # fixed limits from train healthy data
    hp = {r: d[d.on & d.healthy] for r, d in runs.items()}
    H = pd.concat(hp.values())
    L = {ch: float(np.quantile(H[ch], 0.999)) for ch in FIXED_HIGH}
    L["belt_ratio"] = float(np.nanquantile(H["belt_ratio"], 0.001))

    feats_rf = {r: rf_features(d, P) for r, d in runs.items()}
    feats_if = {r: if_features(d) for r, d in runs.items()}
    labels = {r: rf_labels(d) for r, d in runs.items()}

    def fit_if(keys):
        X = pd.concat([feats_if[k][runs[k].on.values & runs[k].healthy.values] for k in keys]).iloc[::4]
        return IsolationForest(n_estimators=200, max_samples=256, random_state=0, n_jobs=-1).fit(X.values)

    def fit_rf(keys):
        X = pd.concat([feats_rf[k][labels[k] >= 0] for k in keys])
        y = np.concatenate([labels[k][labels[k] >= 0] for k in keys])
        keep = (y != 0) | (np.arange(len(y)) % 4 == 0)           # thin healthy samples 4x
        return RandomForestClassifier(n_estimators=150, min_samples_leaf=5, class_weight="balanced_subsample",
                                      random_state=0, n_jobs=-1).fit(X.values[keep], y[keep])

    # out-of-fold model scores
    s_if, s_rf = {}, {}
    for f in folds_of(train_ids):
        others = [k for k in train_ids if k not in f]
        mi, mr = fit_if(others), fit_rf(others)
        h_col = list(mr.classes_).index(0)
        for k in f:
            s_if[k] = -mi.score_samples(feats_if[k].values)
            s_rf[k] = 1.0 - mr.predict_proba(feats_rf[k].values)[:, h_col]

    phi = ar1_phi(runs)
    print("lag-1 autocorrelation (whitened if > %.1f): " % WHITEN_ABOVE +
          ", ".join(f"{k} {v:.3f}" for k, v in phi.items()))
    stats = {
        "fixed": {r: debounce(stat_fixed(d, L), DEBOUNCE["fixed"]) for r, d in runs.items()},
        "cusum": {r: stat_cusum(d, phi) for r, d in runs.items()},
        "iforest": {r: debounce(stat_model(d, s_if[r]), DEBOUNCE["iforest"]) for r, d in runs.items()},
        "rf": {r: debounce(stat_model(d, s_rf[r]), DEBOUNCE["rf"]) for r, d in runs.items()},
    }

    cfg = {"fa_per_hour": FA_PER_HOUR, "fixed_limits": L, "cusum": {"k": CUSUM_K, "clip": CUSUM_CLIP,
           "channels": CUSUM_CH, "phi": phi, "whiten_above": WHITEN_ABOVE}, "debounce": DEBOUNCE, "thresholds": {}}
    print(f"\n=== TRAIN report (out-of-fold for iforest and rf). Budget {FA_PER_HOUR} FA per healthy hour ===")
    summary = []
    for name, st in stats.items():
        thr, allowed, hours = tune(runs, st)
        cfg["thresholds"][name] = thr
        fa = fa_count(runs, st, thr)
        ev = evaluate(runs, st, thr)
        for kind, g in ev.groupby("kind"):
            summary.append(dict(detector=name, fault=kind, detected=f"{int(g.detected.sum())}/{len(g)}",
                                delay_med_s=round(g.delay.median(), 2), delay_max_s=round(g.delay.max(), 2),
                                frac_med=round(g.frac.median(), 3), frac_max=round(g.frac.max(), 3)))
        print(f"{name:8s} threshold {thr:.4g}   false alarms {fa} in {hours:.2f} h = {fa / hours:.2f}/h (allowed {allowed})")
    print()
    print(pd.DataFrame(summary).to_string(index=False))

    # final models on all train runs, for step 6
    joblib.dump(fit_if(train_ids), os.path.join(HERE, "iforest_v3.joblib"))
    mr = fit_rf(train_ids)
    joblib.dump(mr, os.path.join(HERE, "rf_v3.joblib"))
    with open(os.path.join(HERE, "detectors_v3.json"), "w") as fh:
        json.dump(cfg, fh, indent=2)
    print(f"\nwrote detectors_v3.json, iforest_v3.joblib, rf_v3.joblib in {HERE}")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit("usage: python ml/detectors.py data/v3")
    main(sys.argv[1])
