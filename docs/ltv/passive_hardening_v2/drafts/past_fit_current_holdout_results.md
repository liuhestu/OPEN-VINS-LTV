# Two fixed past-fit/current-holdout results

The two user-selected IDs/events were evaluated under the saved input-only design. Build300 passed; fabricated tests301 and boundary/current-context tests303–304 passed. Input-support attempt302 used an incorrect lower-window epsilon; its output is preserved and marked `TOOL_HISTORY_BOUNDARY_ERROR` in `holdout_support_302_correction.json`. No actual geometry fit used that selector. Corrected support305 uses exactly `t−1.0 <= s < t`, with the separate1e-9 tolerance used only to match a past physical timestamp to a current clone. Geometry306 reserved and consumed exactly two actual solves.

## Support before fitting

Each target current context contains12 poses, including the execution pose. For27537 there are20 past cam0 observations in the fixed1s window, of which11 have a pose in this single current version;9 older clones are absent. For29636 there are18 past observations,11 supported and7 missing. The11 included rays span0.5s in each case. All missing clone timestamps are saved. They are never replaced by previous-version pose means.

This is incomplete1s pose support, not a complete1s triangulation claim. The already saved design uses **every** eligible past ray in the fixed1s window, requiring enough current coherent clone support; the main agent clarified that missing some clones does not prohibit fitting the remaining eligible rays. No more favorable time window or point subset was selected after errors were seen. Current cam0 and all current/right/future rays are excluded from fitting. Context pose timestamps are finite and unique, execution-pose time matches the target, and rotations/positions are validated. The source cache is unchanged; no observer or P replay is invoked here.

## Results

| Quantity |27537 at76.75s|29636 at78.90s|
|---|---:|---:|
| Coherent past rays / actual span |11 /0.5s|11 /0.5s|
| Maximum center baseline |0.44731m|0.20375m|
| Point-line normal matrix condition |788.807|547.414|
| Past median angular residual |0.0019143rad|0.104601rad|
| Past maximum angular residual |0.0023655rad|0.175399rad|
| Current held-out angular residual |0.350263rad|0.297001rad|
| Actual LTV pre-correction angle |0.349214rad|0.163584rad|
| Past/current nonpositive signed rays |0 /false|0 /false|
| LTV-minus-fit along current ray |0.31612m|3.44084m|
| LTV-minus-fit transverse |0.11156m|0.52852m|

For27537, the coherent past observations are internally consistent under one static-point/current-VIO-pose model, while the current held-out ray differs by approximately0.350rad. The independently fitted past point and actual LTV predicted point have nearly the same current angular disagreement. This supports a current observation/model inconsistency beyond a purely LTV point-state explanation. Together with the297 actual-path contribution, it explains how an inconsistent observation can dominate the harmful shared correction. It still does not prove frontend identity corruption: current pose-prior error, timing/calibration effects or violation of the static-point assumption are not independently excluded.

For29636, the past rays already have large residuals under the single static-point model. The normal equation is numerically identifiable but the fitted point does not describe the past observations well. Its approximately0.515m current range is therefore **not** a trustworthy point location or GT and cannot be used to label the LTV point's3.44m along-ray difference as true distance error. This event supports sustained observation/pose/static-model inconsistency; it does not isolate a single current ray or distinguish mistracking from pose/model error.

No new seed-validity threshold was applied. `FIT_DIAGNOSTIC_ONLY` means a finite nondegenerate mean solution, not an accepted seed, calibrated confidence or operational readiness. Signed ray and residual values are reported without outcome-dependent deletion. The current pose/clones come from the same OpenVINS frontend/estimator and are correlated with image history, not independent external truth. No GT was opened and no uncertainty probability is claimed.

## Evidence

- Context exporter: `diagnostics/export_holdout_context.cpp`.
- Selection/fit tool and fabricated tests: `diagnostics/past_fit_holdout.py`, `diagnostics/test_past_fit_holdout.py`.
- Corrected immutable support: `/home/he/output/ltv_passive_hardening_v2/diagnostics/R3B_holdout_support_02/support.json` and `contexts.jsonl`.
- Actual two-solve results: `/home/he/output/ltv_passive_hardening_v2/diagnostics/R3B_holdout_fits_01/results.json`.

The integer target receipts were fixed by the user; the original-cache camera double mapping is unique for these recorded events as independently audited previously. The exporter does not claim a generic injective conversion for arbitrary adjacent nanosecond timestamps. No candidate registration, point deletion, new seed gate or R3C work was performed.
