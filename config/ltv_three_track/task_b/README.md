# B3 frozen reproduction configuration

Engineering decision: STOP after reproducible V2_03 local ATE failure. Do not enable production fusion from this configuration.

`fixed_c0/` preserves the three original C0 YAML bytes, with production confirmation20, G/V OFF and max_slam0. New approximation flags are default OFF in native code. The independent experimental runner selects the switch after native parsing:

- `run_ltv_gv_production ... L_OFF`: same B3 gate/anchor scalar diagnostics, fusion OFF.
- `run_ltv_gv_production ... L_ON`: actual independently enabled approximate landmark update; G/V remain OFF.
- `run_ltv_gv_evaluation ... L_ON`: same numerical update with full matrix capture; never use that timing as production performance.

Arguments are estimator_config.yaml, original ASL `mav0` input root, unique output directory, mode, and optional registered prefix seconds (40 in safety checks; omit for full input). Use only the matching task-local frozen install and clean ROS underlay. The manifests preserve exact commands, hashes and runtime source5cdedc0; do not mix other-task overlays or rebuild live artifacts.

The model/gate/information limits are in `docs/ltv/three_track_validation/task_b/engineering_approx_contract.json`. Native support remains an experimental reproduction facility, without qualified engineering, benefit or statistical-consistency claims. Future parameters or correlated models require a new candidate and fresh affected checks.
