# SAC long-run follow-up: policy regression and near-zero-speed behavior

**Date:** 2026-10-17 (JST)  
**Status:** interim analysis based on user-provided terminal summaries, not final run completion  
**Repository branch:** `feature/long-train-offline-20261016`  
**Earlier snapshot:** [11,000-step interim evidence](sac-longrun-interim-11000.md)

> **Scope and evidence boundary.** Numerical values below were transcribed from local WSL output supplied during the experiment. The full `episode-01.jsonl`, checkpoints, and local candidate-selection JSON were not accessible to this GitHub operation and have **not** been uploaded here. The committed CSV and SVG figures are derived summaries, not replacements for original raw files. Do not claim a full independent replay or full three-hour completion from this snapshot.

## 1. Main finding

After a successful deterministic 3 m forward-goal episode at 10k training steps, the 15k and 20k checkpoints both timed out at 120 simulated seconds. The failures were **not** repetitions of the initial 5k wandering behavior. Both approached the goal quickly in the first 20–50 steps but subsequently exhibited little sustained progress and substantially slower commanded linear motion.

The 15k policy reached its closest recorded distance **0.731840 m at step 37** and finished near **0.797469 m**. The 20k policy got substantially closer, **0.406332 m at step 28**, just **0.056332 m outside** the 0.35 m success radius, and later ended **1.455757 m** from the goal. Its distance *rebounded*; the data do not establish whether this was due to heading choice, oscillation, control mapping, or learned action values. **Neither evaluation reached the arrival threshold.**

![Evaluation final distance comparison](figures/sac-final-distance.svg)

## 2. Frozen-policy evaluation evidence

| Checkpoint | Outcome | Steps | Sim time (s) | Path (m) | Closest (m) | Final (m) | Episode return |
| ---: | :--- | ---: | ---: | ---: | ---: | ---: | ---: |
| 5,000 | timeout | 600 | 120 | 20.858 | 2.186830 | 5.193285 | -10.388094 |
| **10,000** | **success** | **47** | **9.4** | **2.694** | **0.333471** | **0.333471** | **+14.861014** |
| 15,000 | timeout | 600 | 120 | 2.648 | 0.731840 | 0.797469 | -1.625 |
| 20,000 | timeout | 600 | 120 | 5.272 | 0.406332 | 1.455757 | -2.972982 |

- Same fixed indoor scene, initial state, forward 3 m goal, deterministic evaluation policy and seed 7. One episode per checkpoint; **0/1 vs 1/1 are observed outcomes, not general success probabilities**.
- The 5k baseline precedes gradient updates; the 10k policy had undergone SAC updates. Four evaluations support **non-monotonic single-scenario performance**, not generalization or statistically robust learning outcomes.
- The 20k evaluation ended with `evaluation_completed` at 2026-10-17 08:33:55 UTC, with manager state reporting 20k valid steps; the subsequent Chunk 22 was launched. Training was ongoing at the latest supplied state.
- In the latest previously shared training summary, each completed 1,000-step chunk after warmup reported 1,000 gradient updates; **15,000 gradient updates had been reported by 20k**.

## 3. Segment-level investigation of the two failed trajectories

Each row below is a summary of **50 actual `step` records**, not a replay of all individual records. A 'near-zero' sample is defined *for this analysis* as `abs(ack.v) < 0.01 m/s`; this is separate from the environment's `stalled` flag.

| Episode | First 50: mean `|v|` | First 50: end distance | Steps 51–100: mean `|v|` | Steps 51–100: near-zero | Last 100: mean `|v|` | Last 100: near-zero |
| :--- | ---: | ---: | ---: | ---: | ---: | ---: |
| 15k | 0.24016 m/s | 0.7549 m | 0.00286 m/s | 100% | 0.00286 m/s | 100% |
| 20k | 0.43620 m/s | 1.3462 m | 0.01467 m/s | 56% | 0.00925 m/s | 52% |

![Mean absolute linear speed by 50-step interval](figures/sac-speed-by-interval.svg)

**15k failure structure.** First 50 actions approach from 3 m to 0.755 m; the minimum occurs at step 37 (0.73184 m). From steps 51 through 600, *every* 50-step bin reports 100% near-zero commanded linear speeds, with mean speed magnitudes around 0.001–0.003 m/s. Goal distance drifts gently from 0.756 m to 0.797 m. The mean reward of these later bins is near -0.010 per step. This is prolonged near-stationary control despite `stalled=false` in reported tail records; no causal conclusion about the stalled detector can be made without inspecting its definition.

**20k failure structure.** The goal distance reached **0.406332 m at step 28**, but by the end of step 50 it had increased to 1.3462 m. For steps 51–600, mean `|v|` by bin is about 0.008–0.015 m/s and the near-zero fraction is 48–62%. Distance stays broadly in the 1.44–1.46 m range in later bins. This supports **slow persistent behavior with mixed near-zero commands**, not the stronger claim that *every* later action is near-zero. The underlying reason for the early distance rebound remains unknown.

![Selected goal-distance observations](figures/sac-goal-distance-selected.svg)

The closest-point chart is **not** a complete trajectory: it connects the initial state, observed minimum, and reported 50-step endpoints. The connecting segments are visual guides only. Raw JSONL should be used for detailed path/heading analysis.

### Reproducible binned values

See [50-step observation CSV](data/sac-failure-50step-15000-20000.csv) and [evaluation summary CSV](data/sac-evaluations-5000-20000.csv). Plot files are **code-generated SVG**, not AI images; the repository includes an independent Matplotlib reproduction script at [plot_sac_diagnostics.py](scripts/plot_sac_diagnostics.py). Both input CSVs are condensed user-supplied summaries.

## 4. Candidate-best checkpoint: identity established by SHA256

Locally generated marker: `research/candidate-best-10000.json` (on the WSL host; **not** uploaded by this commit).

Two distinct 10k-step checkpoint directories were observed:

| Directory | Model matches evaluation | Replay matches | Manifest matches |
| :--- | :---: | :---: | :---: |
| `step-10000-1c2ad374` | **yes** | **yes** | **yes** |
| `step-10000-bc2de5b9` | no | yes | no |

Thus the **uniquely verified evaluation checkpoint** for the 10k success is **`step-10000-1c2ad374`**, with model SHA256:

`26ea4c6455a9f85ea9aeb1a3c473351115b91b04fa733fd7e731e7c9d13dd3b7`

The other folder has the same replay hash but different model and manifest. This establishes **file identity**, not why two models were generated at the same numbered step. Preserve both until provenance is fully investigated. Do not update `checkpoints/latest.txt` or overwrite a model to mark a candidate.

## 5. Working hypotheses and remaining work

**Directly supported:** two later checkpoints timed out in the same simple goal-reaching benchmark; 15k experienced prolonged near-zero speed after approximately step 50; 20k came close to the success radius then moved further away, with persistent low-speed late behavior; the 10k evaluation-matching model is uniquely hash-identified.

**Not yet established:** whether the near-zero commands reflect the action-to-velocity mapping, a learned local behavior, reward shaping, critic value error, turn control, or detection thresholds. Last observed 15k action component around -0.5 corresponded to `ack.v` near zero, but a specific mapping formula requires source audit. Reported `stalled=false` is not on its own proof of a detector bug.

**Follow-up after this run:** inspect fixed source revision's action mapping, reward/termination/stall logic and logged actions; plot *full* truth-pose trajectories; compare all later checkpoints under multi-seed / initial-state / goal-direction tests; preserve the best measured checkpoint separately from latest; separately evaluate unknown-map coverage and collision/contact ground truth. Maintain checkpoint, manifest, run identity, trace, evaluation and source hashes in any final immutable archive.

## 6. Provenance and plot reproducibility

- Run folder: `/home/user111/AI-Training/runs/sac-longrun-20261017`.
- Report source: terminal excerpts of local `evaluations/*/evaluation.json`, `episode-01.jsonl` segment summaries, checkpoint SHA256 comparison, and `long-run-state.json`.
- Checkpoint evaluation scene: `indoor_fixed`; maximum 600 steps, environment limit 120 s, success threshold 0.35 m.
- Last observed progression: 20,000 valid steps, 15,000 reported gradient updates, the manager subsequently started Chunk 22. **Not a statement of final run duration or results.**
- To regenerate graphs from committed CSV on a Python environment with matplotlib: `python3 scripts/plot_sac_diagnostics.py` from this directory. The code writes code-generated SVG/PNG plots into `figures/generated/`, leaving the committed SVGs unchanged.
