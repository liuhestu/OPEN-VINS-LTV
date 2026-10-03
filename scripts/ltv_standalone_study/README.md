# Standalone first-layer LTV study

Run from any working directory with Python 3, NumPy, SciPy, Matplotlib, g++ (C++14), and Eigen 3. No ROS is loaded or linked.

```bash
python3 scripts/ltv_standalone_study/run_study.py all
```

The individual commands are `freeze`, `verify`, `fixed`, `discretization`, `gains`, `lifecycle`, `initialization`, `diagnostic`, `report`. The `diagnostic` command runs the declared 120 s C0 continuous extension and preserves the 0–60 s comparisons. Use `--out /absolute/external/directory` before the command to select an output directory. Default: `/home/he/output/ltv_standalone_convergence_v2`.

`freeze` verifies the frozen repository manifest and saves the standalone protocol and experimental diff. `verify` builds the real core and diagnostic copy as standalone shared libraries, then runs short algebra, truth, flow, lifecycle, hook-scope and parity checks. Full simulations require an unchanged verification code identity. All BLAS thread counts are set before importing NumPy. The driver holds an exclusive scheduling lock and uses at most two simulation children; ledger writes are serialized. Every full attempt is reserved in `ledger.json` before starting its subprocess, including failures. Successful identical identities resume without spending another run. Never delete failed reservations to reclaim budget.

The instrumented copy adds a hook after lifecycle processing and before correction, plus covariance floor counters. It has no numerical repair or retuning. `CORE` always links the real production `ltv_observer.cpp`; it is not a Python approximation. Only explicitly labelled initializations use truth or pose priors. Normal sensors carry no world point coordinates.

Output per run:

- `config.json`, `code.json`, ledger command/exit code, stdout/stderr and input SHA.
- `trace.npz`: common 200 Hz state, slot IDs, vector/axis/landmark errors and diagnostic counters.
- `camera.npz`: both camera sides, including before lifecycle and after correction.
- `matrices.npz`: full P, states and IDs each second and around lifecycle events; reference P also saved at every common sample for precision comparison.
- `gain_diagnostics.npy`: time, norm(K), norm(K_L r), norm(K_v r), norm(K_eta r), norm(P_vL), norm(P_etaL), landmark/velocity/gravity diagonal-block norms.
- `gain_landmark_contributions.npy`: individual norm(K_i r), NaN padding for absent slots.
- `observations.json`, `lifecycle.json`, `seeds.json`: separate observation/slot ages, last visibility/deletion, right censoring and one-shot seed schedules.
- `metrics.json`: all mandated intervals and thresholds. NaN angles in raw traces mean undefined, never a zero-error observation.

Reference implementations integrate the complete covariance including cross blocks. Neither clips eigenvalues. The split dynamics use the exact commuting rotation/nilpotent transition and exact polynomial process-noise integral for the specified common scalar V blocks. Frozen correction uses a linear solve, not an explicit inverse. No P scaling is used.

The source protocol is `docs/03_LTV_独立收敛_离散化与增益参数验证_Codex执行方案.md`; generated report and compact evidence live under `docs/ltv/standalone_convergence_v2/`. Do not feed diagnostic configurations back into production automatically.

Raw array schema:

- States are `[ell_1 ... ell_N, v, eta]`. `trace.x` / `matrices.x` pad on the right to 96 with NaN; determine N from nonnegative IDs. For PAPER, v/eta occupy indices 48:54, not the last six padded entries. IDs pad to 30 with -1. Full P pads to 96×96 with NaN outside the active square.
- `values`: velocity error norm, eta error norm, eta direction error in degrees (NaN if undefined), eta magnitude error, estimated speed, true speed.
- `landmarks`: 3D error norm, relative 3D error, signed ray distance, transverse/projected residual norm, distance to camera center, signed along-ray error.
- `axes`: three velocity error components followed by three eta error components, all estimate minus truth.
- `diag`: last camera substep count, reset reason enum, native camera update time in ms, cumulative eigenvalue-floor count, cumulative spectral lift norm, minimum negative pre-floor eigenvalue. The last three fields are available only for EXPERIMENTAL; zero placeholders in CORE are not evidence that no floor was applied.
- `matrices.tags`: `before_event` precedes lifecycle processing; `after_event` follows lifecycle, hook, and correction; `second` follows that physical second's camera event; `continuous` is an instantaneous continuous solution. The pre-establishment t=0 record is for auditing only; common x(0)/P(0) are the post-establishment states.

Offline report artifacts keep their provenance explicit. `events_verified.json` resolves the first nonzero correction-subflow event, including initially observed points lost before the second camera call. `joint_outcomes.json` distinguishes first held interval start, confirmation after at least 0.10 s, terminal holding, observation censoring and residence censoring. `initialization_snapshots_reconstructed.npz` reconstructs the hook's before/after state and shared P by exact lifecycle reindexing and frozen seed inputs; it is not a new observer run. `camera_correction_deltas.npz` subtracts lifecycle and seed writes from actual saved camera endpoints to isolate the applied net correction.

After generating the report, run `python3 scripts/ltv_standalone_study/audit_delivery.py` to verify the completed run identities, stage budgets, frozen selector, reference checks, all saved initialization covariance snapshots, report links and table widths, and authorized source SHA values. It writes `delivery_audit.json` and `delivery_sha256.json` without running a simulation. Historical execution deviations are recorded in the report's `decision_log.md`.

`python3 scripts/ltv_standalone_study/paper_review.py` performs an independent offline C0 paper-inspired scene review. It requires the existing study runs, does not invoke any simulator, and writes common/native-horizon curves, prefix-window excitation, strict CORE failure events and source/artifact SHA records under `paper_baseline_review/`. The original delivery manifest remains a historical snapshot; the new directory carries its own provenance.
