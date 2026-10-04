"""Pre-registered train/test split for campaign v3 (Phase 2).

Usage:  python ml/make_split.py data/v3            (writes ml/split_v3.csv)
        python ml/make_split.py data/simscape --tag sim   (writes ml/split_sim.csv)
Writes ml/split_<tag>.csv (committed to git). Run ONCE, before any model is fitted.

Rules (fixed 30 Sep 2026, before looking at any Phase 2 result):
  - jam, slip, overload: 15 train / 10 test each, spread evenly over speeds
  - healthy 12/8, healthy_change 12/8, soak 6/4
  - wear: 0 train / 25 test  (the unseen fault: no detector may see it in fitting or tuning)
  - only manifest rows with status ok and stale_ticks 0
"""
import sys, os
import numpy as np
import pandas as pd

SEED = 20260930
N_TRAIN = {"jam": 15, "slip": 15, "overload": 15, "wear": 0,
           "healthy": 12, "healthy_change": 12, "soak": 6}

def main(data_dir, tag="v3"):
    m = pd.read_csv(os.path.join(data_dir, "manifest.csv"))
    m = m[(m.status == "ok") & (m.stale_ticks == 0)].copy()
    rng = np.random.default_rng(SEED)
    split = {}
    for kind, n_train in N_TRAIN.items():
        g = m[m.kind == kind]
        by_speed = {sp: list(rng.permutation(sorted(g[g.speed == sp].run_id))) for sp in sorted(g.speed.unique())}
        speeds = list(rng.permutation(sorted(by_speed)))
        picked = []
        while len(picked) < n_train:            # round-robin over speeds keeps train balanced
            moved = False
            for sp in speeds:
                if len(picked) < n_train and by_speed[sp]:
                    picked.append(by_speed[sp].pop(0)); moved = True
            if not moved:
                raise SystemExit(f"not enough {kind} runs for {n_train} train")
        for rid in g.run_id:
            split[rid] = "train" if rid in picked else "test"
    out = m[["run_id", "kind", "speed"]].copy()
    out["split"] = out.run_id.map(split)
    dst = os.path.join(os.path.dirname(os.path.abspath(__file__)), f"split_{tag}.csv")
    if os.path.exists(dst):
        raise SystemExit(f"{dst} already exists. The split is pre-registered: do not regenerate it.")
    out.sort_values(["kind", "speed", "run_id"]).to_csv(dst, index=False)
    print(out.groupby(["kind", "split"]).size().unstack(fill_value=0))
    print("wrote", dst)


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
        raise SystemExit("usage: python ml/make_split.py data/v3")
    main(argv[1], TAG)
