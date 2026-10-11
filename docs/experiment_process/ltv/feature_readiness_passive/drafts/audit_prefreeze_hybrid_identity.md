# Independent pre-freeze review: hybrid source and run identity

Static inspection only; no new tests, observer runs, cache replay, dataset scans or confirmation result access.

## Hybrid implementation

`STEREO_THEN_TEMPORAL` is an explicit option validated by LtvOptions and mapped to a distinct pipeline enum in LtvAdapter. The pipeline constructs current stereo first, checks the same numerical geometry/ray/condition/parallax/residual/risk bounds, and builds temporal geometry only if stereo fails those bounds. The same manager subsequently applies history, identity, visibility and capacity rules. Source choice is causal and contains no GT or future-lifetime access. Existing-source construction shares its prior arithmetic ordering. A static review cannot prove old-source bitwise equivalence after recompilation; use registered tests/cache evidence.

The hybrid probe does not include manager history/capacity or independent-check policy in its `stereo_pass` predicate. With the currently declared `require_independent_check=false`, that is consistent: history applies to either source and there is no independent-check admission branch. If a later configuration enabled independent-check gating, the fallback claim would need re-review. Do not tune such a change after confirmation starts.

Temporal covariance retains the full current clone joint P and explicitly marks bearing/state correlation as an approximation. Stereo uses additional right-camera data; no claim of an independent OpenVINS-free information source follows from its success. Held-out historical residuals are diagnostics only in this candidate and do not imply they participated in admission.

## Identity gaps to fix before freezing

1. `develop.synthetic` currently keys/reuses runs by shared-library hash, input hash, method and risk but not by `synthetic/run.py` hash. That Python wrapper constructs covariance and controls scheduling, so it is part of the numerical implementation. `run.json` already records `runner_sha`; include and compare it in the reuse identity. A changed evaluator can be rerun offline on the same numerical arrays; a changed numerical wrapper cannot silently reuse them as its own execution.
2. `real_run` records the runner binary, `libov_msckf_lib.so`, main estimator YAML and sensor CSVs. Add all loaded project libraries and the complete actual configuration/calibration/mask file set to the frozen real identity. Hashing only estimator_config.yaml cannot detect changed camera calibration or mask input referenced by that YAML.
3. Do not relabel B/P_OLD produced by an older binary as runs of a rebuilt hybrid binary. Preserve their original identity. Same-source mathematical expectations are not executed equivalence evidence. Either run the frozen B/P_OLD identities within budget or document the exact scope of an executed equivalence proof without claiming same-binary identity.
4. Freeze immutable copies/paths of numerical executables and libraries, not merely mutable build paths with an initial hash. Running a new binary against a replaced shared library changes actual identity. Stage launcher should verify frozen bytes and candidate identity before every attempt and record them at reservation.
5. Separate numerical-estimation identity from evaluator/report identity. Re-evaluation with corrected support/metadata checking is an offline analysis attempt, not a fresh observer simulation or a license to alter the original numerical provenance.

These are provenance/contract requirements. They do not establish or refute the scientific performance of the new candidate.
