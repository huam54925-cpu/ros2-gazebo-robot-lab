# SAC long-run interim evidence — 2026-10-17

> **Interim snapshot, not a completed three-hour run.** This record transcribes observed console outputs supplied during the live WSL experiment. It is not an upload of raw files, trajectories, checkpoints, or a final independent audit. Last quoted manager state: 11,000 valid steps (2026-10-17 07:31:30 UTC), session 2, training phase, chunk 13 active.

## Provenance

- Repository branch: `feature/long-train-offline-20261016`; local source: `/home/user111/AI-Training/offline-bundle-20261017/source/training/local_drive`.
- Run path: `/home/user111/AI-Training/runs/sac-longrun-20261017`.
- Profile: `formal`; device argument `cuda`; seed 7; 1,000 steps per training chunk; `learning_starts=5000`.
- Evaluation reports identified under `evaluations/step-5000-seed-7-session-2-chunk-6/` and `evaluations/step-10000-seed-7-session-2-chunk-11/`.
- Evaluation code revision reported: `84361e802f3860237d8636e494dc7dd168c19995`; simulation image `sha256:7fae8a467efcf70a9a8956da5150ddbc624f6e3ecc785f513d717760ecdf5ad1`.
- Evaluation configuration: indoor_fixed scenario; fixed starting state and forward 3 m goal; deterministic policy; one episode at seed 7; arrival threshold 0.35 m; maximum 600 steps or 120 simulated seconds. This is *not* an unknown-map exploration benchmark.

## Training progression from finished chunk reports

| Chunk | Start step | End step | Gradient updates in chunk | Actor max parameter change | Final replay size |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 2 | 0 | 1,000 | 0 | 0 | 1,000 |
| 3 | 1,000 | 2,000 | 0 | 0 | 2,000 |
| 4 | 2,000 | 3,000 | 0 | 0 | 3,000 |
| 5 | 3,000 | 4,000 | 0 | 0 | 4,000 |
| 6 | 4,000 | 5,000 | 0 | 0 | 5,000 |
| 7 | 5,000 | 6,000 | 1,000 | 0.0369407721 | 6,000 |
| 8 | 6,000 | 7,000 | 1,000 | 0.0407877266 | 7,000 |
| 9 | 7,000 | 8,000 | 1,000 | 0.0574289300 | 8,000 |
| 10 | 8,000 | 9,000 | 1,000 | 0.049741 | 9,000 |

The first five recorded 1,000-step chunks served as replay warmup. Four later chunks reported 4,000 gradient updates in total and nonzero actor parameter changes. These changes prove updates were executed, not convergence or generalization. Checkpoints and replay buffers were found in the bind-mounted run directory; for example `checkpoints/step-1000-1b15b2d7/{manifest.json,model.zip,replay.pkl}`. The completed chunk-3 report showed `initial_steps=1000` and `final_replay_size=2000`; its `resumed_steps=0` field should not be equated with failure without reading code.

## Frozen-policy evaluation comparison (single episode per checkpoint)

| Metric | Step 5,000 baseline | Step 10,000 |
| --- | ---: | ---: |
| Outcome | timeout | **success** |
| Episode steps | 600 | **47** |
| Simulated time | 120.0 s | **9.4 s** |
| Episode return | -10.388094244970507 | **14.8610136440115** |
| Truth-pose XY path length | 20.858219769460366 m | **2.694423753257497 m** |
| Initial goal distance | 3.0000003553435164 m | 3.0000003553435164 m |
| Minimum goal distance | 2.1868301542606314 m | **0.3334710016401533 m** |
| Final goal distance | 5.193284865591013 m | **0.3334710016401533 m** |
| Stalled steps | 0 | 0 |
| Checkpoint unchanged by evaluation | true | true |

The 5,000-step policy is the pre-gradient baseline. At 10,000 steps the deterministic policy reached the 0.35 m success radius after 47 transitions. This is clear **within-scenario improvement** but only one run per model, with fixed scene/goal/seed: no reliable success-rate estimate or cross-scenario robustness claim follows. Neither evaluation reports ground-truth contact/collision metrics. The displayed path length sums XY truth-pose motion over returned 0.2-second transitions; reset and cleanup motion are excluded.

Evaluation reports quote `learning_enabled=false`: this refers to learning being disabled **during evaluation**, not to the separate training process. Evaluation checkpoints' recorded SHA256 values were unchanged before versus after evaluation.

## Live-run status and caveats

- Last supplied manager state: `{"chunks":13,"last_eval_step":10000,"sessions":2,"phase":"training","valid_steps":11000,"active_container":"ldtrain-44645c569350-13"}`, updated `2026-10-17T07:31:30+00:00`.
- Last supplied checkpoint pointer: `step-11001-c412b79a`, one greater than manager's `valid_steps=11000`; reason not yet established.
- Evaluations completed at 5,000 (06:56:49 UTC) and 10,000 (07:25:28 UTC).
- Simulator uses Mesa llvmpipe EGL CPU software rendering. Trainer container was bounded to 6 GiB RAM; simulation container to 2 GiB RAM. Resource snapshots were below these limits, but these are not proof of unlimited runtime stability.
- Session 1 started at 06:11 UTC and session 2 at 06:25 UTC; reason for first session's ending and exact three-hour budget accounting remain unverified.
- The live experiment can progress after this snapshot. Do not treat the data here as final or as all artifacts present on the WSL host.

## Reproducibility and pending checks

1. Preserve original `reports/`, `events.jsonl`, `evaluations/*/evaluation.json`, and selected `episode-01.jsonl` in a final immutable archive, together with hashes and run identity. **These raw source files were not uploaded by this documentation commit.**
2. After the scheduled run ends, verify final state, all chunk reports, checkpoint/replay hashes, and the unexplained one-step checkpoint suffix.
3. Repeat deterministic assessments with multiple independent seeds, initial positions, goals, and maps; include obstacle avoidance and collision/contact ground truth.
4. Evaluate map coverage and exploration efficiency separately; current 3 m goal task is a basic navigation-control benchmark, not a SLAM/exploration validation.
5. Check critic losses, entropy coefficient, policy stability, and post-10k evaluation trend; avoid claiming generalization from the current two episodes.
