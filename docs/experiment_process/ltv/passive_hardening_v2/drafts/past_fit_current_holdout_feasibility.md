# Past-ray fit / current-ray holdout: input-only feasibility

This is a proposed diagnostic only. It does not implement a new seed gate, change the estimator, delete a point or register a candidate. No additional fitting run or confirmation data has been opened for this design.

## Cache sufficiency

The original type2 cache records a complete `FeaturePipelineContext`: current epoch/version/time, execution-pose index, all available cloned pose means `(R_WB,p_WB,t)`, their matching full joint covariance, camera extrinsics and every current observation `(feature_id,camera_id,bearing,match_valid)`. Earlier type2 records supply the two IDs' earlier bearings and physical context timestamps. Thus a causal same-ID past/current measurement consistency diagnostic is possible **when the chosen current context still contains enough past clone poses**. The current diagnostic decoder reads this context but does not export it; a new dedicated exporter would be needed. Existing276/294 decoders and evidence must remain unchanged.

Do not concatenate earlier optimized pose means/P from different versions. At each target receipt resolve all past observation times against the **single current-event pose context**, using the existing1e-9 timestamp resolution rule. Missing/marginalized clones must be listed as unavailable, not reconstructed from stale means. A cache that has observations but insufficient contemporaneous pose support yields `NOT_IDENTIFIABLE`; it does not authorize longer windows or future frames. The exact available same-ID clone counts for these two targets still require the authorized new context extraction; source-level sufficiency is not a claim that both actual fits have already passed.

## Fixed minimum diagnostic

Use only IDs27537 and29636 at their user-selected events and, if extended, the already fixed±0.5s packet neighborhoods. Fix the past window to the existing history1.0s: observations at `t−1s <= t_j < t`, same epoch and valid cam0 match, whose physical pose times exist in the current context. Never include the current cam0 ray in fitting, and never add future rays. Keep every eligible past ray, without rejecting them using GT or residual outcomes. Exclude the current right ray as well, so the measurement holdout is explicit.

Form world camera centers `c_j=p_WB_j+R_WB_j*p_BC` and directions `b_j=R_WB_j*R_BC*normalized(z_j)`. Fit one static point by the same unweighted point-to-line normal equation used by the existing seed mean model: `A=sum(I-b_j*b_j^T)`, `rhs=sum((I-b_j*b_j^T)c_j)`, `A X=rhs`. Use the existing solver's numerical validity/rank conventions; record sample count, actual baseline/time span, singular/eigenvalues, condition, past angular residual distribution and signed ray distances. Do not invent a new favorable confidence threshold. A singular or numerically invalid fit is reported, not repaired by choosing a better subset.

Transform the fitted world point to the current camera through the actual execution pose/extrinsics and compare its predicted unit ray to the current held-out bearing. Also compare it with the actual LTV pre-correction landmark estimate from276, recording camera-centered along-ray/transverse displacement and both current-angle residuals. This comparison has two useful but conditional patterns:

- A internally consistent, well-conditioned past-ray fit predicts the held-out current ray while the LTV point does not: evidence favors LTV point-state disagreement relative to the chosen VIO pose/bearing model.
- A consistent past fit and LTV prediction disagree with the current ray, or past-fit residuals themselves become large: evidence is compatible with measurement association inconsistency, pose-prior error, or violation of the static-point model. It does not uniquely diagnose frontend mistracking.

Neither pattern proves point truth. A poorly conditioned past fit or uncertain correlated poses may leave the alternatives unresolved. Report missing support and residual trajectories rather than turn the fit into a binary operational gate.

## Prior dependence and validation

Pose means/joint covariance come from the same main OpenVINS estimator/frontend and are not independent ground truth. They are the context captured at the original bridge invocation, not a new reoptimized trajectory. The current ray is excluded from this point fit, but that does not make the VIO pose prior statistically independent of the tracked feature history. Do not claim an independent external validation or calibrated failure probability. If uncertainty is reported later, use the actual full joint pose covariance/cross blocks, preserving gauge behavior; do not substitute independent pose blocks.

Before actual extraction, add fabricated checks for known static-point reprojection, current-ray exclusion, nonzero extrinsics, missing-clone refusal, a displaced current ray, and a geometrically degenerate past history. Bind the original cache/current context/time mapping and record all selected IDs before fitting. This remains a small offline geometry analysis, not full observer/P replay. It should follow the completed actual-path correction decomposition and cannot replace it with a supposed leave-one-out result.
