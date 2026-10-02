"""BeltTwin Phase 2, step 6: evaluate the FROZEN detectors on the TEST runs.

Usage:  python ml/evaluate.py data/v3
Needs:  ml/split_v3.csv, ml/behaviour_v3.json, ml/detectors_v3.json, ml/iforest_v3.joblib,
        ml/rf_v3.joblib, data/v3/residuals/ (all from steps 4 and 5)

Nothing is fitted or tuned here. Thresholds, limits, whitening coefficients and models are loaded
as they were frozen in step 5. Wear (25 runs) was never seen in fitting or tuning.

Integrity guard: on the first test evaluation, ml/TEST_EVALUATED.txt records a hash of the frozen
detector settings. If the settings change afterwards, the script refuses to evaluate the test set
again: changing a detector after seeing test results would invalidate the comparison.
Re-running with unchanged settings is allowed and gives identical numbers.

Outputs: report on stdout, ml/results_test_v3.csv (one row per test fault run and detector).
"""
import sys, os, json, hashlib, datetime
import numpy as np
import pandas as pd
import joblib
from scipy.stats import chi2

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import detectors as det     # same feature, statistic and event definitions as step 5

FROZEN = ["detectors_v3.json", "behaviour_v3.json", "iforest_v3.joblib", "rf_v3.joblib", "split_v3.csv"]
MARKER = os.path.join(HERE, "TEST_EVALUATED.txt")


def frozen_hash():
    h = hashlib.sha256()
    for f in FROZEN:
        with open(os.path.join(HERE, f), "rb") as fh:
            h.update(f.encode()); h.update(fh.read())
    return h.hexdigest()[:16]


def poisson_ci(n, hours):
    lo = 0.0 if n == 0 else chi2.ppf(0.025, 2 * n) / 2
    hi = chi2.ppf(0.975, 2 * n + 2) / 2
    return lo / hours, hi / hours


def main(data_dir, which="test"):
    if which == "test":
        fh = frozen_hash()
        if os.path.exists(MARKER):
            prev = open(MARKER).read().split()[0]
            if prev != fh:
                raise SystemExit("REFUSED: frozen settings changed after the test set was evaluated "
                                 f"(was {prev}, now {fh}). The test set may only be used once.")
            print(f"re-run with unchanged frozen settings ({fh}): numbers are identical to the first run")
        else:
            with open(MARKER, "w") as m:
                m.write(f"{fh} first test evaluation {datetime.datetime.now().isoformat(timespec='seconds')}\n")

    split = pd.read_csv(os.path.join(HERE, "split_v3.csv"))
    P = json.load(open(os.path.join(HERE, "behaviour_v3.json")))
    cfg = json.load(open(os.path.join(HERE, "detectors_v3.json")))
    mi = joblib.load(os.path.join(HERE, "iforest_v3.joblib"))
    mr = joblib.load(os.path.join(HERE, "rf_v3.joblib"))
    runs = det.load(data_dir, split[split.split == which], P)
    L, phi, thr, deb = cfg["fixed_limits"], cfg["cusum"]["phi"], cfg["thresholds"], cfg["debounce"]
    h_col = list(mr.classes_).index(0)

    stats = {"fixed": {}, "cusum": {}, "iforest": {}, "rf": {}}
    for r, d in runs.items():
        stats["fixed"][r] = det.debounce(det.stat_fixed(d, L), deb["fixed"])
        stats["cusum"][r] = det.stat_cusum(d, phi)
        s_if = -mi.score_samples(det.if_features(d).values)
        stats["iforest"][r] = det.debounce(det.stat_model(d, s_if), deb["iforest"])
        s_rf = 1.0 - mr.predict_proba(det.rf_features(d, P).values)[:, h_col]
        stats["rf"][r] = det.debounce(det.stat_model(d, s_rf), deb["rf"])

    hours = det.healthy_hours(runs)
    kinds = pd.Series({r: d.attrs["kind"] for r, d in runs.items()})
    print(f"\n=== {which.upper()} evaluation of frozen detectors ===")
    print(f"runs: {len(runs)} ({', '.join(f'{k} {v}' for k, v in kinds.value_counts().items())})")
    print(f"healthy motor-on hours: {hours:.2f}   (train budget was {cfg['fa_per_hour']} FA/h)\n")

    print("false alarms on healthy data:")
    fa_rows = []
    for name, st in stats.items():
        n = det.fa_count(runs, st, thr[name])
        lo, hi = poisson_ci(n, hours)
        fa_rows.append(dict(detector=name, false_alarms=n, per_hour=round(n / hours, 2),
                            ci95=f"[{lo:.2f}, {hi:.2f}]"))
    print(pd.DataFrame(fa_rows).to_string(index=False))

    rows, per_run = [], []
    for name, st in stats.items():
        ev = det.evaluate(runs, st, thr[name])
        ev["run_id"] = [r for r, d in runs.items() if d.attrs["inject"] >= 0 and d.attrs["trip"] >= 0]
        ev["detector"] = name
        per_run.append(ev)
        for kind, g in ev.groupby("kind"):
            rows.append(dict(detector=name, fault=kind, detected=f"{int(g.detected.sum())}/{len(g)}",
                             delay_med_s=round(g.delay.median(), 2), delay_max_s=round(g.delay.max(), 2),
                             frac_med=round(g.frac.median(), 3), frac_max=round(g.frac.max(), 3)))
    print("\ndetection (delay after injection; frac = delay / time to trip):")
    res = pd.DataFrame(rows)
    res["fault"] = pd.Categorical(res.fault, ["jam", "slip", "overload", "wear"])
    print(res.sort_values(["fault", "detector"]).to_string(index=False))

    out = pd.concat(per_run)[["detector", "run_id", "kind", "detected", "delay", "frac", "ttf"]]
    dst = os.path.join(HERE, f"results_{which}_v3.csv")
    out.to_csv(dst, index=False, float_format="%.4g")
    print(f"\nwrote {dst}")


if __name__ == "__main__":
    if len(sys.argv) not in (2, 3):
        raise SystemExit("usage: python ml/evaluate.py data/v3")
    main(sys.argv[1], sys.argv[2] if len(sys.argv) == 3 else "test")
