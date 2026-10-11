# Landmark causal gate development evidence

Status: **5% predictive-benefit FAIL retained; B3 risk-qualified engineering ON actually ran and subsequently STOPPED for reproducible complete V2_03 local ATE failure**. Neither forecast risk success nor the tiny global ATE improvement grants engineering/statistical qualification.

The frozen gate contract retains point-estimate improvement ≥5%, positive lower endpoint of a 95% paired 1-second-block bootstrap interval (2000 draws, seed 20261005), p95/p99 and bad-angle fraction ≤1.05× strong past-bearing geometry, mature supply ≥20%, paired availability ≥90%, and normal/recovery tail limits. The primary comparison uses the original LTV/strong-geometry valid pair; weak-anchor availability never filters primary support. Three-way support is descriptive only. Every candidate's all-available predictions and ungated denominators remain in `gate_v1_results.json`.

| Sequence | Gate | Mature supply | Gain vs strong geometry | 95% block interval | Admission failure |
|---|---|---:|---:|---|---|
| V2_02 | B0 ready/mature | 70.99% | 41.14% | [-13.37%, 61.18%] | interval includes zero |
| V2_03 | B0 ready/mature | 45.72% | 0.335% | [-4.95%, 7.38%] | gain, interval, recovery p99 |
| V2_02 | B1 age 0.2–0.5 s | 13.89% | 57.45% | [8.69%, 79.79%] | supply, recovery p95 |
| V2_03 | B1 age 0.2–0.5 s | 9.32% | 4.78% | [-59.70%, 47.22%] | supply, gain, interval, normal p99 |
| V2_02 | B2 B1 + relative agreement | 13.51% | 58.30% | [9.64%, 81.09%] | supply |
| V2_03 | B2 B1 + relative agreement | 8.895% | 29.68% | [0.616%, 54.98%] | supply |

B2 relative agreement is `norm(ltv-main)/max(norm(main),0.1m)≤0.02`, computed from two pre-camera point predictions without the current bearing or GT. It is a dimensionless engineering model-disagreement score, not a camera innovation or a calibrated NIS. B2 p95/p99 ratios are 0.733/0.818 (V2_02) and 0.802/0.813 (V2_03). Every required tail and bad-angle check passes, while supply fails. This supports investigating supply mechanism; it does not grant ON admission. V2_03 B2 normal gain interval still spans zero, so uniform improvement across phases has not been proved.

B0's V2_03 recovery p99 ratio is 1.05393, above 1.05. B1's V2_02 recovery p95 is 1.12484 and V2_03 normal p99 is 1.13073. These failures are retained, not hidden in aggregate averages. Startup has explicit statistics even though ready-only gates often exclude it; its ungated risk remains visible.

Source audit finds a plausible supply-design defect: `VioManager` adds the current clone at line 421, forms the LTV context before visual correction, and marginalizes the oldest clone only at line 737. `max_clones=11` therefore allows twelve pre-camera clones, or a 0.55 s oldest-live span at 20 Hz. B1/B2's fixed 0.5 s cutoff may exclude normal live-window supply; large absolute timestamps also have sub-microsecond rounding. A descriptive age/tail audit precedes any B3 execution. B3 is separately preregistered with a source-derived 0.55 s horizon and 1 µs time tolerance; main-agent freeze and the original acceptance checks are mandatory. No admission threshold, mature denominator, comparator or confidence rule changes.

Original run: `b_gate_v1_dev_historical_20261008T104150_b4de5501`, source `1f97261`, exit 0. `gate_v1_manifest.json` records source/config/process/lock identity, source and binary immutability, runtime resources, and historical reuse. Raw input identities include exact complete cache, prediction CSV and feature log SHA256, plus historical compiled prediction-source equality. Full raw results remain at `/home/he/output/ltv_three_track_20261008_1827/tasks/task_b/results/`; the absolute path is a local reproducibility location, not a public artifact.

No independent landmark truth exists. Statistical calibration is **NOT_CLAIMED**. Native ON is implemented and a registered 40s command has exited; actual application and main-state effects await the dedicated audit. Complete matched ON nondegradation, ON benefit and production-style performance are **NOT_YET_EVALUATED**. Unknown main/seed/Observer/anchor/visual cross blocks and native covariance-chart reset limitations remain explicit. They are separate from the predictive-quality admission decision.


## B3/B4 result and explicit amended engineering admission

B3 repaired nominal live-window supply: mature coverage 65.07%/36.77%, with all original risk/subgroup tests passed. V2_02 gain is 56.20% (CI [3.96%,75.92%]); V2_03 gain is 4.0328% (CI [0.00885%,12.15%]), below the unchanged 5% predictive-benefit criterion. Strict previously consumed camera0 quality B4 reduces mature supply to 40.96%/21.44%; gains are 57.69% and 3.7910%, with V2_03 CI [1.159%,9.455%]. Both sequences' risk conditions remain passed. Every result and denominator is preserved in gate_B3_results.json and gate_B4_results.json.

Explicit user steering, frozen by the main agent in engineering_on_admission_amendment.yaml, allows risk-qualified engineering ON without turning the 5% predictive FAIL into PASS. Select B3: greater supply and no extra strong-geometry computation for the unsupported B4 temporal mechanism. Approximate noise, 2-point/frame supply and normalized prior-information budget were separately registered before ON. Native JPL/FEJ finite differences and twelve related checks passed; the original diagnostic OFF and scalar B3 shadow OFF are zero-tolerance exact on the complete original start-to40s prefix. This prefix proof is not full replay qualification.
