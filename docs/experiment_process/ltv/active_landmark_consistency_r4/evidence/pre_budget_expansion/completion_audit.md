# R4 completion audit — not a completion claim

Current result: **development evidence supports C02; full PASSIVE_READY_EUROC remains unproven**. No frozen_config.json exists and no formal confirmation has run. This table records the pre-expansion checkpoint. The user subsequently authorized100 real replays; formal work can now continue, but the missing evidence is still missing until executed.

| Requirement | Current evidence | Assessment |
|---|---|---|
| Baseline258b0b6; prior evidence/user files preserved | evidence/baseline_identity.json; evidence/protected_checkpoint.json; git diff | Core/acceptance/parent ledger checks pass; final full protected-tree audit still required |
| Thin module reuses existing history and SeedEstimator | LtvActiveConsistency.cpp, pipeline/manager integration; independent review | Implemented; no new frontend/backend/NIS/gain/ready mechanism |
| Past-only fit, current holdout, gauge and no future leakage | Pure tests1/2, independent17/18; integration29 | Passed for tested contracts |
| First failure skips actual correction and keeps slot/P ownership | Actual-core events24/25; controlled lifecycle/integration29 | Passed; no external mean/P patch |
| Two evaluable failures retire; neutral missing/failed-fit behavior | Lifecycle tests7/8,29; C01/C02 native live audits | Passed; identity guard and once-seed retained |
| Default OFF equivalent to baseline | Full historical cache30:1921 exact non-wallclock comparisons | Core/adapter state/P/slot/events/ready passed; full native OFF logging/trajectory and repeat matrix not yet complete |
| Current production code matches compiled runtime | evidence/current_production_sources_match_build.json |141 current production source files match build21 |
| 76.75s/27537 exact causal counterexample | event_76_75.md; actual cached full x/P oracle | Old/new V error0.251884/0.090604m/s; single-camera snapshot intervention, not whole-trajectory equality |
| 78.90s/29636 exact causal counterexample | event_78_90.md | Old/new V0.119713/0.108943m/s; new eta remains slightly worse than prediction, disclosed |
| Actual target lifecycles in selected version | full_C02_target_lifecycles.md, analysis56 |27537 retires74.30s;29636 never admitted. C01 behavior is not substituted for C02 |
| Whole-sequence benefit and transient cost | Native32/36/39/40; evaluations34/37/41/42; analysis56 | C02 both development sequences meet unchanged contract. Shared descriptive large-correction counts fall; startup maxima do not improve. Not all11 evidence |
| Source-specific seed reliability | Synthetic47/52 and offline50/53 |Stereo3568 writes, reliable10=99.44%; temporal1794,96.77%; opportunities98.29/98.36%. These are two development conditions, not every frozen condition/seed |
| Severe ready/coverage/accuracy in tested synthetics | Offline50/51/53 | Tested normal/recovery conditions pass these output checks; not a global synthetic claim |
| Recovery total contract | Synthetic49, offline51 | **Not closed**:30.1–31.5s supplied-label interval interrupts and evaluator retains failure; later31.7–60s interval passes with35.35–60s wide-good interval. No label, warmup or threshold change |
| Performance and finite-run bounded resources |600s synthetic48, offline54 |12001camera/120002IMU; zero invariant failures; whole branch mean13.474ms/P9514.344ms. Finite stress result, not universal memory proof |
| Freeze all actual source/config/evaluator dependencies | budget.source_identity; test55 | Guard fixtures pass; old hardening and standalone math dependencies added. Formal immutable freeze not created |
| All11 native NEW, at least8 B-valid and matched R3B comparison | evidence/budget_feasibility.json | **Missing**:11 post-freeze ON,9 exact-config OFF predecessors; do not relabel older methods as R3B |
| Deterministic full repeats and logging ON/OFF numeric equality | acceptance.json engineering | **Not executed** for selected formal matrix |
| No G/V injection or main-state intrusion | Native complete engineering outputs and evaluator engineering | Zero injection/main parity in development; formal matrix still untested |
| Formal final report and remote delivery | plan_live.md; decision_log.md; candidate_comparison.md | Evidence retained; overall final report/commit/push not yet completed |

Budget lower bound is30 real in total:8 already counted +11 frozen ON +9 missing matched OFF +2 repeats. It assumes all old B identities can be reused; additional failures or B gaps could require more. The current cap is24, leaving16. No budget is increased by silence, goal continuation, or changing directory/process/agent. See evidence/budget_feasibility.json and the pending user choice.

No new stage, G/V feedback, gain search, FeatureInitializer adapter or real-data ATE is authorized or started by these results.
