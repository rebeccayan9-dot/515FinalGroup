#!/usr/bin/env python3
"""SafeStep E2E pipeline evaluation harness.

Demonstrates two things required for the milestone:

  1. The end-to-end inference pipeline (raw CSV -> clean -> sliding window ->
     DSP features -> Random Forest -> label) executes for >=2 consecutive runs,
     with results recorded each run. This mirrors exactly what the firmware
     (src/main.cpp computeFeatures + clf.predict) does on-device.

  2. Cross-condition generalization via leave-one-session-out: the data was
     collected on 3 separate days by 3 people / environments. We hold each
     session out, train on the others, and test on the unseen session. This
     is the honest cross-condition measure (no in-sample inflation).

Outputs land in results/:
  - e2e_run{1,2}_<ts>.txt   per-run E2E logs (proves reproducibility)
  - E2E_RESULTS.md          consolidated human-readable report

Run:  python e2e_eval.py
"""

import json
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import pandas as pd
import joblib
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import accuracy_score, classification_report, confusion_matrix

ROOT       = Path(__file__).parent
DATA_DIR   = ROOT / "data"
MODEL_PATH = ROOT / "model.pkl"
RESULTS    = ROOT / "results"
RESULTS.mkdir(exist_ok=True)

# Must match train_model.py / firmware exactly
WINDOW = 5
STEP   = 2
FEATURES = ["lidar_cm", "front_cm", "left_cm", "right_cm",
            "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps"]
STATS = ["mean", "std", "min", "max"]
FEATURE_NAMES = [f"{c}_{s}" for c in FEATURES for s in STATS]

# Cross-condition session map: collection day -> session/user label
SESSION_OF = {
    "20260422": "A (2026-04-22)",
    "20260428": "B (2026-04-28/29)",
    "20260429": "B (2026-04-28/29)",
    "20260508": "C (2026-05-08)",
}


# ─── pipeline stages (identical to firmware + train_model.py) ─────────────────
def load_and_clean(csv_path):
    df = pd.read_csv(csv_path)
    if "label" not in df.columns or len(df) == 0:
        return None
    df["lidar_cm"] = pd.to_numeric(df["lidar_cm"], errors="coerce").ffill().fillna(400)
    for col in ["front_cm", "left_cm", "right_cm"]:
        df[col] = df[col].replace(-1.0, np.nan).ffill().fillna(400)
    df = df.dropna(subset=FEATURES)
    return df if len(df) >= WINDOW else None


def make_windows(df):
    X, y = [], []
    for start in range(0, len(df) - WINDOW + 1, STEP):
        w = df.iloc[start:start + WINDOW]
        if w["label"].nunique() > 1:
            continue
        feats = []
        for col in FEATURES:
            v = w[col].values.astype(float)
            feats += [v.mean(), v.std(ddof=0), v.min(), v.max()]
        X.append(feats)
        y.append(w["label"].iloc[0])
    return X, y


def plot_confusion(cm, labels, title, subtitle, out_path):
    """Render a confusion matrix as a heatmap PNG (counts + row-normalized color)."""
    cm = np.asarray(cm, dtype=float)
    row_sums = cm.sum(axis=1, keepdims=True)
    norm = np.divide(cm, row_sums, out=np.zeros_like(cm), where=row_sums != 0)

    n = len(labels)
    fig, ax = plt.subplots(figsize=(max(5, 0.9 * n + 2), max(4, 0.9 * n + 1.5)))
    im = ax.imshow(norm, cmap="Blues", vmin=0, vmax=1, aspect="equal")
    cbar = fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
    cbar.set_label("row-normalized (recall)", fontsize=8)
    cbar.ax.tick_params(labelsize=7)

    ax.set_xticks(range(n)); ax.set_yticks(range(n))
    ax.set_xticklabels(labels, rotation=45, ha="right", fontsize=8)
    ax.set_yticklabels(labels, fontsize=8)
    ax.set_xlabel("Predicted", fontsize=9)
    ax.set_ylabel("True", fontsize=9)
    ax.set_title(subtitle, fontsize=8, color="#555", loc="center", pad=6)
    fig.suptitle(title, fontsize=12, fontweight="bold", y=0.97)

    for i in range(n):
        for j in range(n):
            cnt = int(cm[i, j])
            if cnt == 0:
                continue
            ax.text(j, i, str(cnt), ha="center", va="center", fontsize=7.5,
                    color="white" if norm[i, j] > 0.5 else "#1a1a1a")
    ax.set_xticks(np.arange(-.5, n, 1), minor=True)
    ax.set_yticks(np.arange(-.5, n, 1), minor=True)
    ax.grid(which="minor", color="#d0d0d0", linewidth=0.5)
    ax.tick_params(which="minor", length=0)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_summary(insample_acc, cc, out_path):
    """Headline bar chart: in-sample vs each leave-one-session-out fold."""
    labels = ["In-sample\n(deployed)"] + [f"Hold-out\n{r['held_out'].split()[0]}" for r in cc]
    vals   = [insample_acc] + [r["accuracy"] for r in cc]
    mean_cc = float(np.mean([r["accuracy"] for r in cc]))
    colors = ["#2f6db0"] + ["#e0892e"] * len(cc)

    fig, ax = plt.subplots(figsize=(max(5, 1.1 * len(vals) + 1.5), 4.2))
    bars = ax.bar(labels, vals, color=colors, width=0.62, edgecolor="white")
    for b, v in zip(bars, vals):
        ax.text(b.get_x() + b.get_width() / 2, v + 0.015, f"{v:.3f}",
                ha="center", va="bottom", fontsize=9, fontweight="bold")

    ax.axhline(mean_cc, color="#c0392b", ls="--", lw=1.2, zorder=0)
    ax.text(len(vals) - 0.5, mean_cc + 0.012, f"cross-cond mean {mean_cc:.3f}",
            ha="right", va="bottom", fontsize=8, color="#c0392b")

    ax.set_ylim(0, 1.08)
    ax.set_ylabel("Accuracy", fontsize=9)
    ax.set_title("blue = reproduces on training data · "
                 "orange = generalization to an unseen session/user",
                 fontsize=8, color="#555", pad=6)
    fig.suptitle("SafeStep — in-sample vs cross-condition accuracy",
                 fontsize=12, fontweight="bold", y=0.98)
    ax.grid(axis="y", color="#e3e3e3", lw=0.6)
    ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def session_for(csv_path):
    date = csv_path.name.rsplit("_", 2)[-2]
    return SESSION_OF.get(date, f"?({date})")


def build_dataset():
    """Run clean+window over every CSV. Returns X, y, session-per-window."""
    rows_X, rows_y, rows_s = [], [], []
    n_files = 0
    for csv in sorted(DATA_DIR.glob("*.csv")):
        df = load_and_clean(csv)
        if df is None:
            continue
        X, y = make_windows(df)
        if not X:
            continue
        n_files += 1
        sess = session_for(csv)
        rows_X.extend(X)
        rows_y.extend(y)
        rows_s.extend([sess] * len(X))
    X = pd.DataFrame(rows_X, columns=FEATURE_NAMES)
    return X, np.array(rows_y), np.array(rows_s), n_files


# ─── Part 1: consecutive E2E inference runs on the deployed model ─────────────
def e2e_run(run_idx, bundle):
    """Execute the full pipeline once, end to end, and record results."""
    t0 = time.perf_counter()
    X, y, _, n_files = build_dataset()             # raw -> clean -> window -> features
    model = bundle["model"]
    le    = bundle["label_encoder"]
    y_enc = le.transform(y)
    y_pred = model.predict(X[bundle["features"]])  # RF inference (same as firmware clf)
    dt = time.perf_counter() - t0

    acc = accuracy_score(y_enc, y_pred)
    report = classification_report(y_enc, y_pred, target_names=le.classes_,
                                   digits=3, zero_division=0)
    cm = confusion_matrix(y_enc, y_pred)
    cm_df = pd.DataFrame(cm, index=le.classes_, columns=le.classes_)

    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    lines = [
        f"SafeStep E2E pipeline — RUN {run_idx}",
        f"timestamp     : {ts}",
        f"files parsed  : {n_files}",
        f"windows (samples through pipeline): {len(X)}",
        f"pipeline wall time: {dt*1000:.1f} ms",
        f"deployed-model accuracy (in-sample): {acc:.4f}",
        "",
        "per-class report:",
        report,
        "confusion matrix (rows=true, cols=pred):",
        cm_df.to_string(),
    ]
    out = "\n".join(lines)
    fn = RESULTS / f"e2e_run{run_idx}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
    fn.write_text(out)
    print(out)
    print(f"\n  -> recorded to {fn.relative_to(ROOT)}\n" + "=" * 70)

    png = None
    if run_idx == 1:  # identical across runs; render once
        png = RESULTS / "cm_deployed_model.png"
        plot_confusion(cm, list(le.classes_), "Deployed model — confusion matrix",
                       f"all data, in-sample · {len(X)} windows · acc {acc:.3f}", png)

    # fingerprint of predictions, to prove run-to-run reproducibility
    fp = hash(y_pred.tobytes())
    return {"run": run_idx, "n_windows": int(len(X)), "n_files": n_files,
            "accuracy": round(float(acc), 6), "wall_ms": round(dt * 1000, 1),
            "pred_fingerprint": fp, "file": fn.name,
            "png": png.name if png else None}


# ─── Part 2: cross-condition leave-one-session-out ───────────────────────────
def cross_condition(X, y, sess):
    sessions = sorted(set(sess))
    results = []
    blocks = []
    for held in sessions:
        te = sess == held
        tr = ~te
        if tr.sum() == 0 or te.sum() == 0:
            continue
        clf = RandomForestClassifier(n_estimators=50, max_depth=10,
                                     class_weight="balanced",
                                     random_state=42, n_jobs=-1)
        clf.fit(X[tr], y[tr])
        y_pred = clf.predict(X[te])
        acc = accuracy_score(y[te], y_pred)
        labels = sorted(set(y[te]) | set(y_pred))
        cm = confusion_matrix(y[te], y_pred, labels=labels)
        cm_df = pd.DataFrame(cm, index=labels, columns=labels)
        rep = classification_report(y[te], y_pred, digits=3, zero_division=0)
        classes_present = sorted(set(y[te]))

        tag = held.split()[0]   # "A", "B", "C"
        png = RESULTS / f"cm_holdout_{tag}.png"
        plot_confusion(cm, labels, f"Cross-condition — held-out session {held}",
                       f"trained on other sessions · {int(te.sum())} test windows "
                       f"· acc {acc:.3f}", png)

        results.append({"held_out": held, "train_windows": int(tr.sum()),
                        "test_windows": int(te.sum()), "accuracy": round(float(acc), 4),
                        "classes_in_test": len(classes_present), "png": png.name})
        blocks.append(
            f"### Held-out session {held}\n\n"
            f"- Train windows: {int(tr.sum())}  |  Test windows: {int(te.sum())}\n"
            f"- Classes present in test: {len(classes_present)} "
            f"({', '.join(classes_present)})\n"
            f"- **Cross-condition accuracy: {acc:.3f}**\n\n"
            f"![confusion matrix, held-out {tag}]({png.name})\n\n"
            f"```\n{rep}\nConfusion matrix (rows=true, cols=pred):\n{cm_df.to_string()}\n```\n"
        )
    return results, blocks


def main():
    print("=" * 70)
    print("SafeStep — E2E pipeline + cross-condition evaluation")
    print("=" * 70 + "\n")

    if not MODEL_PATH.exists():
        raise SystemExit("model.pkl not found — run train_model.py first.")
    bundle = joblib.load(MODEL_PATH)

    # Part 1: two consecutive end-to-end runs
    runs = [e2e_run(1, bundle), e2e_run(2, bundle)]
    reproducible = runs[0]["pred_fingerprint"] == runs[1]["pred_fingerprint"]
    print(f"\nConsecutive-run reproducibility: "
          f"{'IDENTICAL predictions ✓' if reproducible else 'DIFFER ✗'}")
    print(f"  run1 acc={runs[0]['accuracy']:.4f} ({runs[0]['n_windows']} windows, "
          f"{runs[0]['wall_ms']} ms)")
    print(f"  run2 acc={runs[1]['accuracy']:.4f} ({runs[1]['n_windows']} windows, "
          f"{runs[1]['wall_ms']} ms)")

    # Part 2: cross-condition
    print("\n" + "=" * 70)
    print("Cross-condition: leave-one-session-out generalization")
    print("=" * 70)
    X, y, sess, _ = build_dataset()
    cc, blocks = cross_condition(X, y, sess)
    print(f"\n{'held-out session':<22}{'test win':>10}{'classes':>9}{'accuracy':>11}")
    for r in cc:
        print(f"{r['held_out']:<22}{r['test_windows']:>10}"
              f"{r['classes_in_test']:>9}{r['accuracy']:>11.3f}")
    mean_cc = np.mean([r["accuracy"] for r in cc])
    print(f"{'mean':<22}{'':>10}{'':>9}{mean_cc:>11.3f}")

    # Headline summary bar chart
    summary_png = RESULTS / "accuracy_summary.png"
    plot_summary(runs[0]["accuracy"], cc, summary_png)
    print(f"Summary chart -> {summary_png.relative_to(ROOT)}")

    # Consolidated markdown report
    md = []
    md.append("# SafeStep — E2E Pipeline & Cross-Condition Results\n")
    md.append(f"_Generated {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}_\n")
    md.append("## Summary\n")
    md.append(f"![accuracy summary]({summary_png.name})\n")
    md.append(f"The pipeline reproduces perfectly on training data (**{runs[0]['accuracy']:.3f}** "
              f"in-sample) but generalization to an unseen session/user averages "
              f"**{mean_cc:.3f}** — the gap is the cross-condition story (see Part 2).\n")
    md.append("## Pipeline\n")
    md.append("`raw sensor CSV → clean (lidar/ultrasonic gap-fill) → sliding window "
              "(5 samples, step 2) → DSP features (mean/std/min/max × 10 channels = "
              "40 features) → Random Forest → activity label`\n")
    md.append("This is byte-for-byte the same DSP + classifier the firmware runs "
              "on-device (`src/main.cpp` `computeFeatures()` + `clf.predict()`).\n")

    md.append("\n## Part 1 — Consecutive E2E runs\n")
    md.append("Each run re-executes the *entire* pipeline from raw CSVs through "
              "inference and records its own log file.\n")
    md.append("| Run | Files | Windows | Wall time | Accuracy | Log |")
    md.append("|----|------|--------|----------|---------|-----|")
    for r in runs:
        md.append(f"| {r['run']} | {r['n_files']} | {r['n_windows']} | "
                  f"{r['wall_ms']} ms | {r['accuracy']:.4f} | `results/{r['file']}` |")
    md.append(f"\n**Reproducibility:** run 1 and run 2 produced "
              f"{'**identical** predictions ✓' if reproducible else 'differing predictions ✗'} "
              "(deterministic pipeline).\n")
    if runs[0].get("png"):
        md.append(f"\n![deployed-model confusion matrix]({runs[0]['png']})\n")
    md.append("> Note: Part-1 accuracy is in-sample (the deployed model was trained on "
              "this data). It demonstrates the pipeline executes and reproduces — not "
              "generalization. For generalization see Part 2.\n")

    md.append("\n## Part 2 — Cross-condition (leave-one-session-out)\n")
    md.append("Data spans 3 collection sessions (different days / users / environments). "
              "Each is held out in turn; the model trains only on the other two and is "
              "tested on the unseen session.\n")
    md.append("| Held-out session | Train win | Test win | Classes | Accuracy |")
    md.append("|---|---|---|---|---|")
    for r in cc:
        md.append(f"| {r['held_out']} | {r['train_windows']} | {r['test_windows']} | "
                  f"{r['classes_in_test']} | **{r['accuracy']:.3f}** |")
    md.append(f"| **mean** | | | | **{mean_cc:.3f}** |")
    md.append("")
    md.extend(blocks)

    report_path = RESULTS / "E2E_RESULTS.md"
    report_path.write_text("\n".join(md))
    print(f"\nConsolidated report -> {report_path.relative_to(ROOT)}")

    # machine-readable summary
    (RESULTS / "summary.json").write_text(json.dumps(
        {"consecutive_runs": runs, "reproducible": reproducible,
         "cross_condition": cc, "cross_condition_mean": round(float(mean_cc), 4)},
        indent=2, default=str))


if __name__ == "__main__":
    main()
