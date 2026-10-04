"""BeltTwin Phase 2, step 4: behaviour model and residuals.

Usage:  python ml/behaviour_model.py data/v3
        python ml/behaviour_model.py data/simscape --tag sim   (split_sim.csv -> behaviour_sim.json)
If rVibration is missing (all NaN, as in the Simscape data) the vibration channels are skipped.

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
using it as an extra input removes most healthy wander from temperatures, vibration and belt ratio.
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
QUAD_SPEED = False        # steady-state temperature quadratic in speed (selected per dataset by grouped CV)
SPEED_TOL = 0.0           # steady-speed test: PLC rSpeed is exactly constant; Simscape gets 0.05 (tiny wander)
INIT_N = 120              # samples (30 s) used to estimate the initial thermal state; not monitored
CHANNELS = ["current", "motor", "bearing", "vib", "ratio", "motor_lc", "bearing_lc", "vib_lc", "ratio_lc"]
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


def sim_temp(p, d, col, load=None, x0=None):
    """p = (a, b, c) or (a, b, c, k). With k and a load proxy, k*load is added to the steady state.
    The response is forced(inputs) + (1-a)^(k+1) * x0. If x0 is None it is estimated by least
    squares from the first INIT_N samples (state estimation; those samples are not monitored)."""
    a, b, c = p[:3]
    tss = d.rAmbientTemp.values + d.on.values * (b * d.rSpeed.values + c)
    if len(p) > 4:                                   # optional speed^2 term (selected per dataset)
        tss = tss + d.on.values * p[4] * d.rSpeed.values ** 2
    if load is not None and len(p) > 3:
        tss = tss + p[3] * load
    forced = lag(a, tss, 0.0)
    g = (1.0 - a) ** (np.arange(len(tss)) + 1.0)
    if x0 is None:
        x0 = initial_state(d[col].values, forced, g)
    return forced + g * x0


def initial_state(y, forced, g):
    """Least-squares initial thermal state from the first INIT_N samples, given the model's forced
    response. A plain mean of the first samples was about 1 s late on a still-cooling motor, and a
    2 s line fit left about 1 sigma of error that decayed over minutes and fed false CUSUM alarms."""
    n = min(INIT_N, len(y))
    e = y[:n] - forced[:n]
    return float(np.dot(g[:n], e) / np.dot(g[:n], g[:n]))


def initial_state_lc(P, d, col, key, rc_on):
    a, b, c, k = P[key][:4]
    b2 = P[key][4] if len(P[key]) > 4 else 0.0
    tss = d.rAmbientTemp.values + d.on.values * (b * d.rSpeed.values + c + b2 * d.rSpeed.values ** 2) + k * rc_on
    forced = lag(a, tss, 0.0)
    g = (1.0 - a) ** (np.arange(len(tss)) + 1.0)
    return initial_state(d[col].values, forced, g)


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
    const = d.rSpeed.rolling(W + 1).std().values <= SPEED_TOL   # steady speed
    ok = on_all & const & (sp > 10) & (dt > 0)
    r = np.full(len(d), np.nan)
    r[ok] = dp[ok] / (dt[ok] * sp[ok])
    return r


def fit_temp(H, col, p0, loads):
    """Joint output-error fit of (a, b, c, k), with the current residual as auxiliary input.
    Fitting without it biases the time constant, because the unmeasured load is a large,
    slow disturbance on the motor temperature. The input-only residual reuses (a, b, c)."""
    res = lambda p: np.concatenate([sim_temp(p, d, col, L) - d[col].values for d, L in zip(H, loads)])
    lb, ub = [1e-5, 0, -20, -50], [0.5, 2, 20, 50]
    if QUAD_SPEED:
        p0 = list(p0) + [0.0]; lb = lb + [-1.0]; ub = ub + [1.0]
    return least_squares(res, p0, bounds=(lb, ub)).x


def fit(H):
    """H: list of healthy-segment DataFrames. Returns parameter dict."""
    P = {}
    res = lambda p: np.concatenate([sim_current(p, d) - d.rMotorCurrent.values for d in H])
    P["current"] = least_squares(res, [0.3, 2.0, 0.09], bounds=([1e-3, -5, 0], [1, 10, 1]),
                                 loss="soft_l1", f_scale=0.2).x.tolist()
    loads = [current_residual_on(P, d) for d in H]
    P["motor"] = fit_temp(H, "rMotorTemp", [0.004, 0.5, 0.0, 5.0], loads).tolist()
    P["bearing"] = fit_temp(H, "rBearingTemp", [0.002, 0.17, 0.0, 1.0], loads).tolist()
    P["has_vib"] = bool(any(np.isfinite(d.rVibration.values).any() for d in H))
    if not P["has_vib"]:
        P["vib"], P["vib_sigma"] = [0.0, 0.0], [1.0, 0.0]
    if P["has_vib"]:
        res = lambda p: np.concatenate([(pred_vib(p, d) - d.rVibration.values)[d.on.values > 0] for d in H])
        P["vib"] = least_squares(res, [0.2, 0.025], loss="soft_l1", f_scale=0.1).x.tolist()
    if P["has_vib"]:
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
    if P["has_vib"]:
        y = np.concatenate([q["vib"] for q in raw])[m]
        P["k_vib"] = float(np.dot(x, y) / np.dot(x, x))
    else:
        P["k_vib"] = 0.0
    # belt creep grows with load but cannot go below zero (belt never outruns the drum):
    # ratio = 1 - max(0, creep0 + k * load_proxy)
    br = np.concatenate([belt_ratio(d) for d in H])[m]
    ok = ~np.isnan(br)
    res = lambda p: (1.0 - np.maximum(0.0, p[0] + p[1] * x[ok])) - br[ok]
    P["creep"] = least_squares(res, [0.01, 0.015], loss="soft_l1", f_scale=0.005).x.tolist()
    return P


def pred_ratio_lc(P, load_proxy):
    c0, k = P["creep"]
    return 1.0 - np.maximum(0.0, c0 + k * load_proxy)


def raw_residuals(P, d):
    """Residual = measured - predicted, plus the load proxies used for compensation."""
    out = {}
    on = d.on.values > 0
    ic = sim_current(P["current"], d)
    rc = d.rMotorCurrent.values - ic
    rc_on = np.where(on, rc, 0.0)
    out["current"] = np.where(on, rc, np.nan)
    # one physical initial state per run, estimated with the load-compensated model
    xm = initial_state_lc(P, d, "rMotorTemp", "motor", rc_on)
    xb = initial_state_lc(P, d, "rBearingTemp", "bearing", rc_on)
    guard = np.arange(len(d)) < INIT_N
    for ch, col, key, x0 in (("motor", "rMotorTemp", "motor", xm), ("bearing", "rBearingTemp", "bearing", xb)):
        out[ch] = np.where(guard, np.nan, d[col].values - sim_temp(P[key], d, col, x0=x0))   # load=None: input-only
        out[ch + "_lc"] = np.where(guard, np.nan, d[col].values - sim_temp(P[key], d, col, rc_on, x0=x0))
    pv = pred_vib(P["vib"], d)
    out["vib"] = np.where(on & P.get("has_vib", True), d.rVibration.values - pv, np.nan)
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
    r["ratio_lc"] = (q["ratio"] + P["ratio"]) - pred_ratio_lc(P, q["load_vib"])
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
        S[ch] = float(np.nanstd(x)) if np.isfinite(x).any() else float("nan")
    s0, s1 = P["vib_sigma"]
    lvl = np.concatenate([r["vib_level"].values for r in R])[m]
    v = np.concatenate([r["vib_lc"].values for r in R])[m] / (s0 + s1 * lvl)
    S["vib_lc_rel"] = float(np.nanstd(v)) if np.isfinite(v).any() else float("nan")
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
        x["speed"] = np.round(x["speed"])
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


def main(data_dir, tag="v3"):
    global SPEED_TOL, QUAD_SPEED
    if tag != "v3":
        SPEED_TOL = 0.05
        # Structure chosen by grouped CV on the Simscape TRAIN runs (tests.md A15): a speed^2 term
        # cut the bearing residual 0.313 -> 0.086 degC and removed a +-1.2 sigma speed bias.
        # The frozen v3 model stays linear (the PLC plant is linear in speed).
        QUAD_SPEED = True
    split = pd.read_csv(os.path.join(HERE, f"split_{tag}.csv"))
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
    if P.get("has_vib") is True:
        del P["has_vib"]          # keep the v3 file byte-identical to the frozen one
    with open(os.path.join(HERE, f"behaviour_{tag}.json"), "w") as fh:
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
    if P.get("has_vib", True):
        print(f"  vib      v0 {P['vib'][0]:.3f}   v1 {P['vib'][1]:.4f}   sigma {P['vib_sigma'][0]:.3f} + {P['vib_sigma'][1]:.3f}*level")
    else:
        print("  vib      not in the data (skipped)")
    print(f"  ratio    {P['ratio']:.4f}")
    print(f"  vib load gain {P['k_vib']:.3f} per A   belt creep {P['creep'][0]:.4f} + {P['creep'][1]:.4f} per A (>= 0)")
    report(runs, {k: R_oof[k] for k in train}, S)
    print(f"\nwrote {os.path.join(HERE, f'behaviour_{tag}.json')} and {len(split)} files in {out_dir}")



def parse_tag(argv):
    """Optional '--tag NAME' (default v3) selects the file set: split_NAME.csv, behaviour_NAME.json, ..."""
    argv = list(argv)
    tag = "v3"
    if "--tag" in argv:
        i = argv.index("--tag")
        tag = argv[i + 1]
        del argv[i:i + 2]
    return tag, argv

if __name__ == "__main__":
    TAG, argv = parse_tag(sys.argv)
    if len(argv) != 2:
        raise SystemExit("usage: python ml/behaviour_model.py data/v3")
    main(argv[1], TAG)
