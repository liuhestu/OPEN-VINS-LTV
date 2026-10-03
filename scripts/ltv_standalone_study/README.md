# Standalone first-layer LTV study

Run from any working directory with Python 3, NumPy, SciPy, Matplotlib, g++ (C++14), and Eigen 3. No ROS is loaded or linked.

```bash
python3 scripts/ltv_standalone_study/run_study.py all
```

The individual commands are `freeze`, `verify`, `fixed`, `discretization`, `gains`, `lifecycle`, `initialization`, `report`. Use `--out /absolute/external/directory` before the command to select an output directory. Default: `/home/he/output/ltv_standalone_convergence_v2`.

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
