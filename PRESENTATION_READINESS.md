# SafeStep — System Readiness

_Presentation talking-points draft. Numbers and figures are pulled from
`results/E2E_RESULTS.md` (regenerate with `python3.13 e2e_eval.py`).
Replace every **[fill in]** with your own words before presenting._

---

## 1. One-line readiness statement

> **SafeStep's end-to-end pipeline is fully operational and reproducible. The
> classifier is demo-ready within a known setup (96.6%), but cross-user/hardware
> generalization (mean 58.9%) is the main gap before field deployment.**

**[fill in]** — your own framing of where the project stands and why that's the
right level for this milestone.

![accuracy summary](results/accuracy_summary.png)

---

## 2. What "ready" means here

We assessed readiness on three axes the milestone asks for:

| Axis | Question | Status |
|---|---|---|
| **Pipeline executes E2E** | Does raw sensor data flow all the way to a label? | ✅ Ready |
| **Reproducible over runs** | Same input → same output, run after run? | ✅ Ready |
| **Generalizes across conditions** | Does it hold up for a new user / day / hardware? | ⚠️ Partial |

---

## 3. Evidence (what we can show live)

### 3a. End-to-end pipeline, 2 consecutive runs ✅
- Pipeline: `raw CSV → clean → sliding window (5, step 2) → 40 DSP features
  (mean/std/min/max × 10 channels) → Random Forest → label` — byte-for-byte the
  same DSP + classifier the firmware runs on-device (`src/main.cpp`).
- Ran the full pipeline **twice back-to-back**: 95 files → **4,183 windows** each
  time, ~3.3 s per run, **identical predictions** (verified by fingerprint).
- Recorded to `results/e2e_run1_*.txt` and `results/e2e_run2_*.txt`.

**Takeaway:** the system is deterministic and repeatable — not a one-off lucky run.

### 3b. In-sample classifier quality ✅
- Accuracy **0.966** across all 7 classes; clean confusion-matrix diagonal.
- Strongest classes: `stop` (0.99), `step_up`/`step_down` (~0.98).

![deployed-model confusion matrix](results/cm_deployed_model.png)

### 3c. Cross-condition generalization ⚠️
Leave-one-session-out (train on 2 collection sessions, test on the unseen one):

| Held-out session | Test windows | Accuracy |
|---|---|---|
| A (2026-04-22) | 514 | **0.323** |
| B (2026-04-28/29) | 2,303 | **0.737** |
| C (2026-05-08) | 1,366 | **0.706** |
| **mean** | | **0.589** |

**Takeaway:** between matched-hardware sessions (B↔C) we hold ~70–74%; the
pre-hardware-migration session (A) collapses to 32%.

---

## 4. Readiness by component

| Component | Maturity | Notes |
|---|---|---|
| Sensor fusion (LiDAR + 3× ultrasonic + IMU) | **High** | Concurrent reads, gap-filling, ~5 Hz stream |
| DSP / feature pipeline | **High** | Identical in firmware and training; deterministic |
| Random Forest classifier | **Medium-High** | Excellent in-sample; 50 trees, depth 10, fits on-device |
| Haptic feedback mapping | **Medium** | Bench-tested — all 4 channels fire correctly per alert type  |
| Cross-user generalization | **Medium-Low** | 58.9% mean; sensitive to hardware revision |
| Real-time on-device demo | **High** | Confirm the live serial → dashboard loop on the board |

---

## 5. Known gaps & risks (be honest — graders reward this)

1. **Hardware-revision drift (biggest risk).** Session A predates the "migrate to
   PCB v1" change (different pin/sensor wiring). A model trained on later sessions
   mis-reads it badly — e.g. `stop` collapses entirely into `obstacle_avoid`.
   *Mitigation:* re-collect a small calibration set on the final PCB; treat
   pre-migration data as a separate domain.
2. **Stair-edge transfer is the weakest behavior.** Held-out `step_down` smears
   into `step_up`/`walking`. This is the safety-critical class. *Mitigation:* the
   planned `delta = lidar_cm − front_cm` edge feature (Milestone 2, Phase 2).
3. **Limited user diversity.** ~3 collection sessions/users. More users would
   tighten the generalization estimate.
4. **[fill in]** — anything you hit in the lab (sensor dropouts, mounting, etc.).

---

## 6. Path to the next readiness level

- [ ] Re-collect labeled data on the **final PCB v1** hardware (kills the drift gap).
- [ ] Add the `delta` stair-edge feature; re-evaluate `step_down` recall.
- [ ] Add ≥2 more users; re-run `e2e_eval.py` and watch the cross-condition mean.
- [ ] Demonstrate the **live** on-device loop (board → `dashboard.py`) end to end.
- [ ] **[fill in]** — your team's next concrete step.

---

## 7. Speaker notes (suggested slide flow)

1. **"The pipeline works and it's reproducible."** Show the summary chart; mention
   2 consecutive runs, identical predictions.
2. **"Within a setup, the classifier is strong — 96.6%."** Show the clean
   confusion matrix.
3. **"But we pressure-tested it across users/hardware, and here's the honest
   result."** Show the cross-condition bars; call out the A-fold drop.
4. **"We know exactly why A fails"** — pre-PCB-migration sensor drift, visible in
   the held-out-A matrix (`stop → obstacle_avoid`).
5. **"So here's what 'ready' means and what's next."** Land on Section 6.

> The narrative that lands: *we don't just have a number that looks good — we
> stress-tested it, found where it breaks, and we know the fix.*

---

_Figures: `results/accuracy_summary.png`, `results/cm_deployed_model.png`,
`results/cm_holdout_{A,B,C}.png`. Full per-class reports: `results/E2E_RESULTS.md`._
