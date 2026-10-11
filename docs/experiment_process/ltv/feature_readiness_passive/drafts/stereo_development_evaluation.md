# Corrected STEREO development evaluation

These are development results, not confirmation evidence. Corrected evaluator attempts87/89/91 and independent exact three-mode audits88/90/92 exited0. No observer replay was added for this evaluation. Existing results were preserved; new directories are `/home/he/output/ltv_feature_readiness_passive/real_metrics/<sequence>_STEREO_dev_v2/`, with full numeric/identity/source evidence in metrics.json and component curves in error_curves.npz. Combined summary: `real_metrics/stereo_development_corrected_summary.json`.

|Sequence|Raw opportunity tracks / admitted|Admission fraction|NEW G / V coverage|V RMSE on NEW V-ready support OLD→NEW|Raw V RMSE OLD→NEW|Sequence output target|
|---|---:|---:|---:|---:|---:|---|
|V1_01_easy|4114 /1094|26.59%|96.86% /97.64%|0.50156→0.18580 m/s|0.53399→0.26918 m/s|Met, subject to reference limitation|
|V2_02_medium|6204 /1038|16.73%|71.49% /72.51%|0.84972→0.22650 m/s|0.90667→0.27853 m/s|Met|
|indoor_forward_3|3110 /330|10.61%|0% /0%|Unavailable: no ready samples|6.56680→2.29549 m/s|Not met: insufficient coverage|

Opportunity counts are from all raw initialized current observations, before manager capacity, final geometry and risk screening. STEREO opportunities require3 consecutive cam0 observations spanning0.1s and current right-camera observation. Manager-only counters remain separate; they omit eligible capacity-dropped tracks and also include some histories without stereo availability, so they are not the final denominator. No real landmark GT exists; the admission fractions are not seed reliability percentages.

All three sequences consumed complete registered inputs, had finite current raw outputs on100% of initialized camera packets, no stale/invalid-ready timestamps, exact B/P_OLD/P_NEW audit/trajectory/unmatched-camera output equality, and zero actual auxiliary/G/V submissions. Both v and eta raw non-degradation conditions passed. UZH raw eta improved3.84607→0.98150m/s², but mean active/mature counts were only4.18/3.41, below the unchanged15-point requirement; raw improvement cannot establish ready usability.

EuRoC reference support was99.29% (V1) and99.38% (V2); UZH v3 velocity support was58.46% with the unchanged0.1s derivative and fixed gap mask. V1_01's known attitude-reference limitation remains: its body-frame G/V comparison is conditional on that reference. Main OpenVINS V RMSE remained substantially smaller (raw0.0490/0.0389/0.1369m/s), so these results do not establish that LTV is a superior or independent information source. No ATE improvement is claimed.

This supports retaining the stereo source for near-range real features while testing the already-authorized hybrid source to address UZH coverage and far-point synthetic screening. It does not justify lowering coverage, feature-count, or reliability targets. Main agent owns final method choice and confirmation freeze.
