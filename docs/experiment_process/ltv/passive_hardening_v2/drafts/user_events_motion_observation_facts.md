# User-selected events: actual motion and observation facts

Analysis289 reads the unchanged original R3B V2_03 cache/features and aligned276 prediction evidence. It extracts the user-specified integer receipts1413394964105760512 and1413394966255760384 plus the fixed inclusive±0.5s neighborhoods (21 packets each). Time origin is the first initialized physical IMU time1413394887.3557606. No GT file, full observer, covariance recursion, new candidate or threshold search is used. All selected packets, not just large-residual IDs, are retained.

## Exact-event facts

| Quantity | init+76.75s | init+78.90s |
|---|---:|---:|
| Actual correction interval |0.05000019073486328s|0.04999971389770508s|
| Actual midpoint IMU substeps |10|10|
| Corrected gyro norm across interval endpoints |1.11008–1.21974rad/s|1.06510–1.41158rad/s|
| Integrated gyro norm over actual substeps |0.0582258rad|0.0607625rad|
| Corrected specific-force norm |7.50603–11.55081m/s²|6.71250–10.63208m/s²|
| Raw cam0 observations / actual managed corrections |51/23|77/27|
| Retained slots before/after |30/30|30/30|
| Actual births / active retirements this event |0/0|5/5|
| Eligible pre-existing angular population |23|22|
| Prediction angular P95 |0.0115545rad|0.00529141rad|
| Maximum prediction angle |0.349214rad, ID27537|0.163584rad, ID29636|
| Points above0.02rad in angular population |1|1|
| Signed Δv, body coordinates, m/s |[0.054045,0.108407,0.122902]|[0.061881,0.027729,0.088655]|
| Norm Δv / correction rate |0.172563m/s /3.45124m/s²|0.111615m/s /2.23232m/s²|
| Signed Δη, body coordinates, m/s² |[0.019041,0.068973,0.010849]|[0.052384,0.013976,0.054595]|
| Pool ready / G ready / V ready |true/false/false|true/false/false|

Specific force is not gravity-subtracted world acceleration. Integrated angular-speed norm is an angular path-length diagnostic, not an exact finite attitude increment. The recorded motion is finite and explicitly propagated; these measurements alone do not establish an IMU fault or prove discretization caused the error.

At76.75s there is no same-packet birth or removal to explain the correction as an immediate seed write. There are28 candidate seed records, none admitted, and all23 corrected observations refer to existing states. The P95 ordering intentionally omits the single largest angle here: for23 eligible points it takes the22nd sorted value. The largest value is therefore compatible with a small reported P95 without any logger inconsistency. This does not identify that point's contribution to shared velocity correction.

ID27537 was admitted from STEREO3.30s earlier, with relative parallel risk0.0490607, sigma_parallel0.198546m, seed range4.04694m, condition6073.45, parallax0.0256640rad, seed fit residual0.00235153rad and available11-observation held-out maximum0.00411650rad. At the selected event its predicted angle decreases from0.349214 to0.216458rad, while its landmark update is approximately[−0.37094,−0.74410,−2.16412]m. This establishes a large existing-point state correction with remaining angular mismatch. The seed's approximate covariance/risk is not a persistent correctness guarantee and is not an online GT label.

At78.90s the actual five births are29731,29513,29733,29712,29645 (three stereo, two temporal); the five active removals are28894,28964,29037,29086,29087. The largest pre-existing angle is ID29636, admitted TEMPORAL_POSE0.45s earlier with relative risk0.0459870, range3.80500m, condition15818.77, parallax0.0232320rad and fit residual0.00109767rad; no held-out check was available. Its angle falls0.163584→0.0914996rad. The five current newborns are excluded from the pre-existing22-point P95 population.

## Fixed-neighborhood context and interpretation limits

Within76.75±0.5s there are33 actual births,32 active retirements and59 never-admitted candidate TTL events. Gyro norm spans0.08849–1.42299rad/s, specific force5.81752–14.94184m/s². Within78.90±0.5s there are42 births,42 active retirements and39 candidate TTL events; gyro norm0.48826–1.75418rad/s and specific force5.80651–14.45208m/s². Candidate TTL is kept separate from active slot deletion. Exact original/bias-corrected endpoints, midpoint inputs, biases, calibration, before/after IDs, all candidate confidence fields and every corrected point are preserved in the output.

The main agent's independently evaluated same-event true velocity error change at76.75s (0.106985→0.251884m/s) shows that the actual correction moved a comparatively better prediction away from the reference. This tool explains the **observable update circumstances**: a0.172563m/s coupled correction, existing-point angular inconsistency, substantial turning during the interval and no same-frame seed/lifecycle change. It does not prove which point or input caused that harmful direction. In general `||e+delta||²−||e||²=2e·delta+||delta||²`; a reduced landmark bearing residual does not guarantee that this expression is negative for shared velocity. The smaller reference change reported at78.90s (0.119124→0.119713m/s) likewise cannot be inferred from correction magnitude alone. These reference numbers are supplied by the main event review, not recomputed or used to select points here.

A point-wise causal attribution would require the actual correction operator/full cross-covariance and update ordering, or a separately authorized intervention; neither is reconstructed here. ID27537's large residual is correlated evidence, not a proven cause, and its removal is not justified by this analysis. Both packets are already G/V-not-ready in the tested R3B output. No R3C readiness proposal is advanced by these facts.

Evidence: `/home/he/output/ltv_passive_hardening_v2/diagnostics/R3B_user_motion_events_01/summary.json` and `neighborhoods.jsonl`. The latter retains all original per-event observations, candidate records, admission-source records, biases, calibrated IMU endpoints and signed point/shared corrections. Direction-difference/angle diagnostics must not be confused with the core projection innovation. Source/archive SHA identities are saved in the summary; output selection is fixed by the user's two integer receipts and±0.5s only.
