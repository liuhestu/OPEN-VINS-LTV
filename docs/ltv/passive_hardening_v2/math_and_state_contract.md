# Development contract: delayed zero startup

This is a candidate under development, not an accepted scientific result. The baseline C0 continuous equations, discrete core correction and all Riccati parameters remain unchanged. No G/V submission is enabled. The new path is controlled by `ltv_passive_hardening_enabled` and defaults OFF.

Before coherent startup, the adapter consumes valid IMU time brackets and gathers causal feature histories, but does not propagate an unobserved zero-gravity state. The manager evaluates eligible seeds without admission. After consecutive adequate supply, a staged manager admits seeds and a staged core starts at that same physical IMU time; the first correction has zero duration. Core x/P/slots, manager, ID map and same-event readiness acknowledgement are committed together. Shared velocity and gravity remain zero in this candidate. No pose-aided shared-state initialization is implemented.

The atomic boundary is the camera admission/correction transaction, following an already validated and committed IMU prediction. It is not whole-event rollback of consumed IMU input. A rejected camera transaction throws an explicit run failure; it must not commit births, consume seeds, or partially change camera-corrected x/P. The preserved prediction and input are needed for failure diagnosis. Such failures cannot count as normal scientific outcomes or silently continue.

Pre-correction angular diagnostics use the existing point slots only: ray = ell_B - p_BC; measured direction = R_BC bearing_C. Points within 0.1 m do not establish valid angular diagnostics. The 95th-percentile prediction angle and pure camera-correction increments ||delta v||/delta t, ||delta eta||/delta t are evaluated before declaring readiness. They are causal health indicators, not error guarantees. Calibration and context camera 0 must agree.

All physical-time transitions are in IMU time (camera time plus offset). A duplicate camera request publishes neither ready nor current raw state. The event that requests dormancy retains its actual current raw prediction/correction for diagnosis; the following event resets the whole observer epoch before further propagation and resumes collection. The readiness policy survives quality-triggered epoch replacement so its 30-second cooldown cannot be bypassed. Explicit parent reset and physical-input fault have separate semantics.

Raw availability and management-event availability are separate. Cold collection still logs current candidates/opportunities/reasons; unavailable raw vectors are explicit null. Complete cache evidence includes the new health state only when hardening is enabled, preserving legacy byte identity when OFF.

Outstanding proof: integrated tests, zero-start interventions on development inputs, actual recovery precision, seed-source calibration and bounded resource behavior. A pure policy unit-test pass does not establish these outcomes.
