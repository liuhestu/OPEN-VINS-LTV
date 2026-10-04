# Passive LTV hardening study

Current status: development, **not final EuRoC acceptance**. See
`docs/ltv/passive_hardening_v2/plan_live.md` and the independent checkpoint audit.
Historical 64 standalone experiments are read-only and are not rerun here.

The optional path maintains a bounded candidate/history pool, performs one legal
landmark mean write per identity, and separates observer health from seed
quality. Initial warmup preserves the original zero-state trajectory. With R3,
1–14 current constraints preserve finite internal state but never declare it
ready; prolonged complete observation loss or physical faults stop the observer.
Recovery requires supply and confirmation again. There is no main-VIO velocity
or gravity bootstrap and no G/V fusion. Main OpenVINS propagation, visual updates,
state/P/FEJ and frontend are unchanged.

## Build and tested development use

Run from the repository root. Dependencies are the existing ROS Humble/OpenVINS
build environment, C++14, Eigen, OpenCV, Python3 and NumPy/SciPy. ROS setup supplies
build dependencies; these tools do not launch ROS or a bag player.

```bash
bash scripts/ltv_passive_hardening/configure_native.sh
python3 scripts/ltv_passive_hardening/budget.py build --identity native_build -- \
  cmake --build /home/he/output/ltv_passive_hardening_v2/build \
  --target run_ltv_feature_passive replay_ltv_feature_cache -j1
python3 scripts/ltv_passive_hardening/budget.py status
```

All builds/tests/full runs use the shared ledger and two-slot scheduler. BLAS is
single-threaded. A failed or repeated full run consumes another budget unit.
Do not put an outer budget lock around a tool that already invokes `budget.run`.
Inputs and large outputs live outside the repository at
`/home/he/output/ltv_passive_hardening_v2`.

`real_develop.py NAME SEQUENCE --r3 --runtime ABSOLUTE_IMMUTABLE_RUNTIME` copies
study configuration and runs a native sensor-only replay. It supports only the
three predeclared development sequences. Use a new NAME; existing output is
never overwritten. `engineering_short.py NAME --r3 --hardened-only` runs at most
10 seconds and verifies receipts, main-state parity and exact cache replay.
Each stores an identity, command, exit code and output directory in the ledger.

Four study-only YAML booleans enable the reviewed R3 path; the fifth keeps the separate R3B grace candidate disabled:

```yaml
ltv_passive_hardening_enabled: true
ltv_hardening_initial_warmup: true
ltv_hardening_health_readiness: true
ltv_hardening_preserve_constrained_state: true
ltv_hardening_ready_soft_grace: false
ltv_enable_gravity: false
ltv_enable_velocity: false
```

Hardening defaults OFF. To return to the previous managed Passive method, set
all five hardening booleans false. To disable LTV entirely, set `ltv_enabled` and
`ltv_feature_readiness_enabled` false. Neither change requires altering main
estimator formulas. Production configuration files are not modified by these
tools; generated configuration copies have full SHA manifests.

## Confirmation and resumption

After all development gates and independent review, the primary agent uses
`freeze_candidate.py confirmation_1 --runtime PATH --wrapper PATH`. This archives
source, configuration and identities before opening seeds201/202. Do not execute
it simply to bypass the final-reserve guard. The second and last confirmation,
if needed, uses the documented revision process and seeds301/302.

`real_confirm.py NAME SEQUENCE B|P_PREV|P_NEW` requires the frozen manifest. It
supports all11 declared EuRoC sequences, runs actual native estimation and never
loads ground truth. Run B/P_PREV/P_NEW on identical inputs/configuration; offline
`real_evaluate.py` checks exact main state/P/FEJ/trajectory and evaluates physical
errors. A green single-sequence result is not the all11 contract.

Estimation and evaluation remain separate for synthetic inputs too:
`synthetic_inputs.py` freezes inputs and separate labels, `synthetic_runner.py`
reads only estimator inputs, and `synthetic_evaluate.py` reads labels offline.
Between-camera snapshots are held state with an explicit cursor, not freshly
propagated200Hz estimates. Historical midpoint inputs use an explicit bridge and
have a new integration identity; do not claim old numerical equivalence.

To resume, inspect `plan_live.md`, the authoritative `attempts.jsonl`, each run's
`run.json`/`attempt.json`, and the actual process handle. Never restart a live
process because a polling call timed out. Completed runs with matching identity
are reused. A retry uses a new output name and another counted attempt. There is
no arbitrary mid-observer numerical restart: an interrupted full run is retained
as incomplete and any full replay is counted anew.

The final deliverable must cover raw/ready accuracy, fixed-denominator coverage,
source-stratified seeds, recovery, all11 native parity, deterministic repeats,
resource bounds and runtime. No claim of localization or ATE improvement follows
from Passive trajectory equality. No command here authorizes G/V injection.

## R3B development diagnostic

`real_develop.py NAME SEQUENCE --r3-grace --runtime PATH` enables the explicitly
registered two-packet/0.10s readiness grace. `synthetic_runner.py` uses mode
`P_NEW_GRACE`; `engineering_short.py NAME --r3-grace` checks OFF and GRACE.
This candidate is not accepted: V2_03 still lacks a continuous five-second
joint-ready episode. Do not widen these settings or describe it as a validated
release. The observer state/P and gains are unchanged.

`soft_grace_report.py --source features.jsonl --errors error_curves.npz --out PATH`
audits held packets using exact event identities and existing evaluator support.
Missing references stay null; held-only accuracy never replaces full ready
acceptance. Freezing a subsequently accepted grace candidate would require an
explicit `--ready-soft-grace`; the freeze command otherwise records false.
