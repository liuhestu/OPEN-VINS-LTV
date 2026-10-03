# Feature readiness Passive study

Independent LTV admission, one-time seeds and management; OpenVINS remains the read-only source. G/V injections are disabled. The authoritative limits, split and criteria are in `docs/ltv/feature_readiness_passive/{goal.md,protocol.json,acceptance.json}`. Historical standalone experiments are not rerun.

From the repository root:

```bash
python3 scripts/ltv_feature_passive/study.py prepare
python3 scripts/ltv_feature_passive/study.py calibrate
python3 scripts/ltv_feature_passive/study.py develop --scene REGULAR --seed 43 --lifetime 0.5 --method SEED_HYBRID
python3 scripts/ltv_feature_passive/study.py freeze --evidence /absolute/development_evidence.json
python3 scripts/ltv_feature_passive/study.py validate synthetic --scene REGULAR --seed 101 --lifetime 0.5
python3 scripts/ltv_feature_passive/study.py validate real --sequence V1_03_difficult --mode B
python3 scripts/ltv_feature_passive/study.py report --manifest /absolute/report_manifest.json --out /absolute/new_report_directory
python3 scripts/ltv_feature_passive/study.py resume
```

These are interfaces, not claims that all phases have already executed. Consult append-only `attempts.jsonl` under `/home/he/output/ltv_feature_readiness_passive` for actual commands, attempts, status, source hashes and logs. `plan_live.md` records current progress. Calibration reuses completed diagnostic attempt9; it does not silently spend another 8160 solves. Development real replay is `real_run.py SEQUENCE MODE --source STEREO_THEN_TEMPORAL` and is restricted to the three named development sequences.

Every full/short run and retry is reserved before launch. Two inherited file locks limit heavy concurrency. BLAS/OMP are single-threaded. Failed/interrupted directories are preserved; never delete them to retry. `resume --reconcile` marks only demonstrably interrupted children terminal without refunding budget or restarting an estimator. A live child or owner is left untouched. Reserved but unstarted records require review. Completed confirmation identities may be reused only after checking every sealed artifact; partial simulations are not continued from an unverified numerical checkpoint. A retry is a new counted attempt.

`artifacts.py` copies the runner, cache replayer and all resolved dynamic libraries into a content-addressed runtime, verifies its contents and actual loader resolution. Configuration trees, sensor CSVs, synthetic numerical wrapper and library hashes form the identities. Confirmation requires an immutable `frozen_config.json` and fails on source/artifact changes. GT is opened only by the evaluator; neither the real runner nor cached inputs access it.

Synthetic outputs retain full x/P before and after camera events, causal seed diagnostics and separate offline labels. Real outputs include exact state/P/FEJ/feature-set audit digests, actual injection counters, raw/current feature opportunity records, branch readiness, sensor-consumption counts, and the deterministic adapter cache. Large artifacts remain external. Uncertainty is a directional risk score with stated pose/bearing correlation approximations, not a calibrated confidence probability.

Post-freeze logging correction: real seed/lifecycle statistics require the SHA-bound exact-cache overlays described in [diagnostic recovery](diagnostic_recovery/README.md). Original frozen runner logs are incomplete for these supplemental fields; raw v/eta and readiness are unaffected. Use the corrected final manifest, not preliminary management tables.
