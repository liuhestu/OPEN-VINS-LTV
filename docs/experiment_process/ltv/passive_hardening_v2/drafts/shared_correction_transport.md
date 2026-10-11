# Fixed one-second shared correction transport

**Correction:** analysis280 is `TOOL_WINDOW_ORIGIN_ERROR` for its registered-window statistics. It used first raw packet rather than first initialized physical IMU time; all prior fixed-window values and interpretations are withdrawn. Its full-epoch/Phi values are unchanged. Original source and report are archived; original `_01` output was not overwritten. Analysis282 in `_02` uses origin1413394887.3557606 (first initialized `imu_time`) and `relative_time=target-origin`,5.75s later than raw start. Tests281/283 cover the nonzero preinitialization prefix and nonzero camera/IMU offset, including exact selected endpoints.

Analysis282 is a descriptive offline diagnostic. The input is the same R3B V2_03 cache and the signed correction reconstruction validated in276; neither existing source nor tested observer was modified. No GT is read, no readiness rule is created, and no window/threshold is searched.

For shared state `[v,eta]`, each actual IMU substep has

`F(dt,omega) = [[I-dt[omega]x, dt I], [0, I-dt[omega]x]]`.

The tool obtains omega using actual cached raw IMU, current calibration/bias and endpoint averaging. Substep matrices multiply on the left in physical order. With camera transition `Phi_k`, each older signed correction contribution is updated by `Phi_k contribution`, then the current delta is appended without further propagation. This includes the transport of previous eta correction into current velocity. Acceleration is affine and cancels in the perturbation transition; it is not mistakenly added to a correction contribution.

The window is exactly `(t−1.0s,t]`, using actual cached target timestamps. Its duration is fixed once from the existing20-frame/20Hz confirmation duration; it does not require20 fictitious evenly spaced packets. Each delta is an instantaneous camera update at its recorded timestamp, even when the preceding inter-camera interval is irregular. A full window requires at least1.0s of the current uninterrupted physical segment. Startup, pause, epoch/bootstrap/current-state transition and camera gap beyond the frozen0.20s limit break continuity; IMU substeps beyond the actual0.05s limit are errors. These limits are verified against the supplied run YAML. This differs deliberately from an offline availability-episode plot that may split at1.5 times median cadence.

For each current frame, `shared_transition.jsonl` stores the complete6x6 Phi, signed sum vector, norm of the sum, sum of individual transported norms, maximum transported single correction, maximum original single correction and maximum original single-frame correction rate. Velocity and eta are reported separately due to their different units. The prefix before a complete window is preserved but excluded from complete-window aggregate statistics.

Test279 passed three independent checks: noncommuting matrix product order; eta-only correction contributing to velocity; and shared coordinate rotation consistency. Analysis282 covered1805 camera transitions,1786 complete windows. There were115 startup pauses and one first-state anchor; no later physical break. Homogeneous eta prediction agrees with the already validated276 mean prediction to `8.88e-15`. Signed components remain a mathematical reconstruction, not an independently logged component-wise live oracle;276's scalar norms/dt and P95 provide the actual-log cross-check.

| Fixed population | Complete windows | Median norm of accumulated v correction | Median sum of v contribution norms | Maximum accumulated v correction | Maximum single-frame v correction rate |
|---|---:|---:|---:|---:|---:|
| Whole available epoch |1786|0.12952 m/s|0.30565 m/s|10.08923 m/s|9.17033 m/s²|
| Early legal-pool interval5.80–13.95s |164/164|0.17438 m/s|0.32532 m/s|0.34720 m/s|2.20552 m/s²|
| Late legal-pool interval72.15–81.70s |192/192|0.16115 m/s|0.43144 m/s|0.36301 m/s|3.45124 m/s²|

| Fixed population | Median norm of accumulated eta correction | Median sum of eta contribution norms | Maximum accumulated eta correction | Maximum single-frame eta correction rate |
|---|---:|---:|---:|---:|
| Whole available epoch |0.06799 m/s²|0.16072 m/s²|4.49411 m/s²|5.11155 m/s³|
| Early legal-pool interval |0.09478 m/s²|0.18130 m/s²|0.19032 m/s²|1.26085 m/s³|
| Late legal-pool interval |0.09007 m/s²|0.23996 m/s²|0.23678 m/s²|2.09411 m/s³|

The two registered legal-pool intervals have similar median accumulated correction norms; the late interval has larger summed contribution norms and greater maximum single-frame activity. These intervals are relative to initialization, not raw startup. A vector sum smaller than a sum of norms merely indicates direction differences/cancellation after coordinate transport. It is **not** evidence of true-state accuracy, reliable readiness or permission to relax thresholds. These statistics also do not attribute shared-state correction to individual landmarks.

Evidence: `/home/he/output/ltv_passive_hardening_v2/diagnostics/R3B_V2_03_shared_transition_02/`. Its summary binds original cache/features, actual YAML, decoder,276 evidence and the new tool SHA. `actual_camera_geometry.jsonl` additionally preserves current cache calibration and managed measured bearings for independent post-angle computation; no nominal extrinsics are substituted. Original prediction direction-minus-measurement residual must not be called the core's projection innovation. Newborn premeans and changes rely on cached seed/lifecycle semantics and were excluded from the live pre-existing-slot P95 cross-check.
