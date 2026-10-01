"""BeltTwin Phase 2, step 4: behaviour model and residuals.

Usage:  python ml/behaviour_model.py data/v3

Fits a behaviour model of the HEALTHY conveyor on the healthy segments of the TRAIN runs only
(ml/split_v3.csv), then writes residuals for every run:
  - train runs: out-of-fold residuals (4 folds grouped by run), so detector tuning in step 5
    never sees residuals from a model that was fitted on the same run
  - test runs:  residuals from the model fitted on all train runs
Outputs:
  ml/behaviour_v3.json              model parameters and residual sigmas
  <data_dir>/residuals/<run_id>.csv  residuals and z-scores per sample
The report printed at the end uses TRAIN runs only (out-of-fold). Test runs are not summarised.

Model (inputs: rSpeed, rAmbientTemp, motor on = eState 1 or 2; nothing fault-affected):
  temperatures  first-order output-error simulation from the run's first samples:
                T[k] = (1-a) T[k-1] + a (ambient + on*(b*speed + c))
  current       same structure, on*(c0 + c1*speed), fast lag
  vibration     static on*(v0 + v1*speed), sigma grows linearly with level
  belt ratio    2 s window: dPosition / (dt_plc * speed), constant healthy value
Load-compensated residuals (suffix _lc): the unmeasured load is visible in the current residual;
regressing it out of the temperature and vibration residuals removes most healthy wander.
Overload IS extra load, so _lc residuals are blind to overload by construction.
"""
import sys, os, json
import numpy as np
import pandas as pd
from scipy.signal import lfilter
from scipy.optimize import least_squares

DT = 0.25                 # nominal sample period, s
N_FOLDS, FOLD_SEED = 4, 20260930
RATIO_WIN = 8             # samples in the belt-ratio window (2 s)
INIT_N = 8                # samples used to estimate the initial thermal state
CHANNELS = ["current", "motor", "bearing", "vib", "ratio", "motor_lc", "bearing_lc", "vib_lc"]
HERE = os.path.dirname(os.path.abspath(__file__))


# ---------------------------------------------------------------- data
def load_runs(data_dir, split):
    runs = {}
    for rid in split.run_id:
        d = pd.read_csv(os.path.join(data_dir, rid + ".csv"))
        d = d.sort_values("nPlcTime").reset_index(drop=True)
        inj = np.flatnonzero(d.nInjectCount.values > d.nInjectCount.values[0])
        d["healthy"] = True if len(inj) == 0 else (np.arange(len(d)) < inj[0])
        d["inject_idx"] = -1 if len(inj) == 0 else inj[0]
        d["on"] = d.eState.isin([1, 2]).astype(float)
        st = np.flatnonzero((d.on.values > 0) & (np.r_[0.0, d.on.values[:-1]] == 0))
        t = np.full(len(d), np.nan)
        if len(st):
            t[st[0]:] = (d.nPlcTime.values[st[0]:] - d.nPlcTime.values[st[0]]) / 1000.0
        d["t_since_start"] = t
        runs[rid] = d
    return runs


# ---------------------------------------------------------------- model pieces
def lag(a, u, y0):
    """y[k] = (1-a) y[k-1] + a u[k], with y[-1] = y0."""
    y, _ = lfilter([a], [1.0, -(1.0 - a)], u, zi=[(1.0 - a) * y0])
    return y


def sim_temp(p, d, col, load=None):
    """p = (a, b, c) or (a, b, c, k). With k and a load proxy, k*load is added to the steady state."""
    a, b, c = p[:3]
    tss = d.rAmbientTemp.values + d.on.values * (b * d.rSpeed.values + c)
    if load is not None and len(p) > 3:
        tss = tss + p[3] * load
    return lag(a, tss, initial_state(d[col].values))


def initial_state(y):
    """State just before the first sample: a straight line through the first INIT_N samples,
    extrapolated to index -1. A plain mean is centred about 1 s late, and a motor that is
    still cooling from the previous run then starts about 0.5 degC off."""
    n = min(INIT_N, len(y))
    if n < 2:
        return float(y[0])
    k, c = np.polyfit(np.arange(n), y[:n], 1)
    return float(c - k)


def current_residual_on(P, d):
    """Measured minus predicted current while the motor is on, 0 while off: the load proxy."""
    rc = d.rMotorCurrent.values - sim_current(P["current"], d)
    return np.where(d.on.values > 0, rc, 0.0)


def sim_current(p, d):
    a, c0, c1 = p
    return lag(a, d.on.values * (c0 + c1 * d.rSpeed.values), d.rMotorCurrent.values[0])


def pred_vib(p, d):
    return d.on.values * (p[0] + p[1] * d.rSpeed.values)


def belt_ratio(d):
    """Causal 2 s belt ratio. Single-sample ratios are dominated by read-timing jitter
    between rPosition and nPlcTime (separate OPC UA reads), so a window is required."""
    W = RATIO_WIN
    dp = np.mod(d.rPosition.diff(W).values, 1000.0)
    dt = d.nPlcTime.diff(W).values / 1000.0
    sp = d.rSpeed.values
    on_all = d.on.rolling(W + 1).min().values > 0
    const = d.rSpeed.rolling(W + 1).std().values == 0
    ok = on_all & const & (sp > 10) & (dt > 0)
    r = np.full(len(d), np.nan)
    r[ok] = dp[ok] / (dt[ok] * sp[ok])
    return r


def fit_temp(H, col, p0, loads):
    """Joint output-error fit of (a, b, c, k), with the current residual as auxiliary input.
    Fitting without it biases the time constant, because the unmeasured load is a large,
    slow disturbance on the motor temperature. The input-only residual reuses (a, b, c)."""
    res = lambda p: np.concatenate([sim_temp(p, d, col, L) - d[col].values for d, L in zip(H, loads)])
    return least_squares(res, p0, bounds=([1e-5, 0, -20, -50], [0.5, 2, 20, 50])).x


def fit(H):
    """H: list of healthy-segment DataFrames. Returns parameter dict."""
    P = {}
    res = lambda p: np.concatenate([sim_current(p, d) - d.rMotorCurrent.values for d in H])
    P["current"] = least_squares(res, [0.3, 2.0, 0.09], bounds=([1e-3, -5, 0], [1, 10, 1]),
                                 loss="soft_l1", f_scale=0.2).x.tolist()
    loads = [current_residual_on(P, d) for d in H]
    P["motor"] = fit_temp(H, "rMotorTemp", [0.004, 0.5, 0.0, 5.0], loads).tolist()
    P["bearing"] = fit_temp(H, "rBearingTemp", [0.002, 0.17, 0.0, 1.0], loads).tolist()
    res = lambda p: np.concatenate([(pred_vib(p, d) - d.rVibration.values)[d.on.values > 0] for d in H])
    P["vib"] = least_squares(res, [0.2, 0.025], loss="soft_l1", f_scale=0.1).x.tolist()
    # vibration sigma model: |r| * sqrt(pi/2) ~ s0 + s1 * level
    lv = np.concatenate([pred_vib(P["vib"], d)[d.on.values > 0] for d in H])
    ar = np.abs(np.concatenate([(d.rVibration.values - pred_vib(P["vib"], d))[d.on.values > 0] for d in H]))
    A = np.c_[np.ones_like(lv), lv]
    P["vib_sigma"] = np.linalg.lstsq(A, ar * np.sqrt(np.pi / 2), rcond=None)[0].tolist()
    r = np.concatenate([belt_ratio(d) for d in H])
    P["ratio"] = float(np.nanmedian(r))
    # load compensation gains, regressed on train data
    raw = [raw_residuals(P, d) for d in H]
    on = np.concatenate([d.on.values > 0 for d in H])
    late = np.concatenate([np.nan_to_num(d.t_since_start.values, nan=-1) > 60 for d in H])
    m = on & late
    x = np.concatenate([q["load_vib"] for q in raw])[m]
    y = np.concatenate([q["vib"] for q in raw])[m]
    P["k_vib"] = float(np.dot(x, y) / np.dot(x, x))
    return P


def raw_residuals(P, d):
    """Residual = measured - predicted, plus the load proxies used for compensation."""
    out = {}
    on = d.on.values > 0
    ic = sim_current(P["current"], d)
    rc = d.rMotorCurrent.values - ic
    rc_on = np.where(on, rc, 0.0)
    out["current"] = np.where(on, rc, np.nan)
    out["motor"] = d.rMotorTemp.values - sim_temp(P["motor"][:3], d, "rMotorTemp")
    out["bearing"] = d.rBearingTemp.values - sim_temp(P["bearing"][:3], d, "rBearingTemp")
    out["motor_lc"] = d.rMotorTemp.values - sim_temp(P["motor"], d, "rMotorTemp", rc_on)
    out["bearing_lc"] = d.rBearingTemp.values - sim_temp(P["bearing"], d, "rBearingTemp", rc_on)
    pv = pred_vib(P["vib"], d)
    out["vib"] = np.where(on, d.rVibration.values - pv, np.nan)
    out["ratio"] = belt_ratio(d) - P["ratio"]
    out["load_vib"] = lag(1 - np.exp(-DT / 2.0), rc_on, 0.0)    # 2 s smoothing against noise and spikes
    out["vib_level"] = pv
    return out


def residuals(P, d):
    q = raw_residuals(P, d)
    r = pd.DataFrame({k: q[k] for k in ("current", "motor", "bearing", "vib", "ratio")})
    r["motor_lc"] = q["motor_lc"]
    r["bearing_lc"] = q["bearing_lc"]
    r["vib_lc"] = np.where(d.on.values > 0, q["vib"] - P.get("k_vib", 0) * q["load_vib"], np.nan)
    r["vib_level"] = q["vib_level"]
    return r


def add_z(P, r):
    z = pd.DataFrame(index=r.index)
    s0, s1 = P["vib_sigma"]
    lvl = r["vib_level"].values
    for ch in CHANNELS:
        if ch == "vib":
            z["z_vib"] = r["vib"] / (s0 + s1 * lvl)
        elif ch == "vib_lc":
            z["z_vib_lc"] = r["vib_lc"] / ((s0 + s1 * lvl) * P["sigma"]["vib_lc_rel"])
        else:
            z["z_" + ch] = r[ch] / P["sigma"][ch]
    return z


def sigmas(P, R, H):
    """Residual sigmas from (out-of-fold) healthy residuals, motor on, after 60 s."""
    S = {}
    m = np.concatenate([(d.on.values > 0) & (np.nan_to_num(d.t_since_start.values, nan=-1) > 60) for d in H])
    for ch in CHANNELS:
        x = np.concatenate([r[ch].values for r in R])[m]
        S[ch] = float(np.nanstd(x))
    s0, s1 = P["vib_sigma"]
    lvl = np.concatenate([r["vib_level"].values for r in R])[m]
    v = np.concatenate([r["vib_lc"].values for r in R])[m] / (s0 + s1 * lvl)
    S["vib_lc_rel"] = float(np.nanstd(v))
    return S


# ---------------------------------------------------------------- main
def folds_of(keys):
    keys = sorted(keys)
    perm = np.random.default_rng(FOLD_SEED).permutation(len(keys))
    return [set(np.array(keys)[perm[i::N_FOLDS]]) for i in range(N_FOLDS)]


def report(runs, R_oof, S):
    rows = []
    for rid, r in R_oof.items():
        d = runs[rid]
        h = d.healthy.values & (d.on.values > 0)
        x = r[h].copy()
        x["t"] = d.t_since_start.values[h]
        x["speed"] = d.rSpeed.values[h]
        x.loc[~np.isin(x["speed"], [40, 50, 60, 70, 80]), "speed"] = np.nan
        x["run"] = rid
        rows.append(x)
    A = pd.concat(rows)
    print("\n=== behaviour model report (TRAIN runs, out-of-fold, healthy, motor on) ===")
    print(f"healthy samples: {len(A)}  ({len(A) * DT / 3600:.1f} h)")
    print("\nresidual sigma (after 60 s):")
    for ch in CHANNELS:
        print(f"  {ch:11s} {S[ch]:.4f}")
    A["tbin"] = pd.cut(A.t, [0, 30, 60, 120, 240, 480, 1e9])
    runs_u = A.run.unique()
    groups = {k: v for k, v in A.groupby("run")}
    rng = np.random.default_rng(1)
    chans = ["motor", "bearing", "motor_lc", "bearing_lc", "current"]
    def bias_by(col):
        B = []
        for _ in range(300):
            X = pd.concat([groups[k] for k in rng.choice(runs_u, len(runs_u))])
            B.append(X.groupby(col, observed=True)[chans].mean() / [S[c] for c in chans])
        B = pd.concat(B)
        g = B.groupby(level=0, observed=True)
        mean = A.groupby(col, observed=True)[chans].mean() / [S[c] for c in chans]
        lo, hi = g.quantile(0.025), g.quantile(0.975)
        out = mean.round(2).astype(str) + " [" + lo.round(2).astype(str) + "," + hi.round(2).astype(str) + "]"
        return out
    print("\nbias in sigma by time since start [95 % run-bootstrap CI]:")
    print(bias_by("tbin").to_string())
    print("\nbias in sigma by steady speed [95 % CI]:")
    print(bias_by("speed").to_string())
    pr = A.groupby("run")[["motor", "bearing", "motor_lc", "bearing_lc"]].mean() / \
        [S["motor"], S["bearing"], S["motor_lc"], S["bearing_lc"]]
    print("\nper-run mean residual in sigma (5 / 50 / 95 %):")
    print(pr.quantile([0.05, 0.5, 0.95]).round(2).to_string())


def main(data_dir):
    split = pd.read_csv(os.path.join(HERE, "split_v3.csv"))
    runs = load_runs(data_dir, split)
    train = [k for k in split[split.split == "train"].run_id]
    healthy = {k: runs[k][runs[k].healthy] for k in train}

    # out-of-fold residuals for train runs
    R_oof, P_folds = {}, []
    for f in folds_of(train):
        P = fit([healthy[k] for k in train if k not in f])
        P_folds.append(P)
        for k in f:
            R_oof[k] = residuals(P, runs[k])
    # final model on all train runs; sigmas from the out-of-fold healthy residuals
    P = fit([healthy[k] for k in train])
    S = sigmas(P, [R_oof[k].iloc[:len(healthy[k])] for k in train], [healthy[k] for k in train])
    P["sigma"] = S
    P["fold_seed"], P["n_folds"] = FOLD_SEED, N_FOLDS
    with open(os.path.join(HERE, "behaviour_v3.json"), "w") as fh:
        json.dump(P, fh, indent=2)

    out_dir = os.path.join(data_dir, "residuals")
    os.makedirs(out_dir, exist_ok=True)
    for _, row in split.iterrows():
        k = row.run_id
        if row.split == "train":
            r = R_oof[k]
        else:
            r = residuals(P, runs[k])
        z = add_z(P, r)
        d = runs[k]
        out = pd.concat([d[["ts", "nPlcTime", "eState", "rSpeed", "phase", "label", "healthy", "t_since_start"]],
                         r, z], axis=1)
        out.insert(0, "split", row.split)
        out.insert(0, "kind", row.kind)
        out.insert(0, "run_id", k)
        out.to_csv(os.path.join(out_dir, k + ".csv"), index=False, float_format="%.5g")

    tau = lambda a: -DT / np.log(1 - a)
    print("identified (all train runs):")
    print(f"  motor    tau {tau(P['motor'][0]):6.1f} s   b {P['motor'][1]:.4f}   c {P['motor'][2]:+.3f}   load gain {P['motor'][3]:.2f} degC/A")
    print(f"  bearing  tau {tau(P['bearing'][0]):6.1f} s   b {P['bearing'][1]:.4f}   c {P['bearing'][2]:+.3f}   load gain {P['bearing'][3]:.3f} degC/A")
    print(f"  current  tau {tau(P['current'][0]):6.2f} s   c0 {P['current'][1]:.3f}   c1 {P['current'][2]:.4f}")
    print(f"  vib      v0 {P['vib'][0]:.3f}   v1 {P['vib'][1]:.4f}   sigma {P['vib_sigma'][0]:.3f} + {P['vib_sigma'][1]:.3f}*level")
    print(f"  ratio    {P['ratio']:.4f}")
    print(f"  vib load gain {P['k_vib']:.3f} per A")
    report(runs, {k: R_oof[k] for k in train}, S)
    print(f"\nwrote {os.path.join(HERE, 'behaviour_v3.json')} and {len(split)} files in {out_dir}")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit("usage: python ml/behaviour_model.py data/v3")
    main(sys.argv[1])
