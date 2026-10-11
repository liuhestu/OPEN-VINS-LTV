# Ownership and scheduling

Main: protocol, acceptance, total ledger, formal configuration, existing core/adapter/options/VioManager bridge, final integration and results. Only main launches or grants heavy tasks through budget.py.

A /root/geometry: new LtvSeedEstimator and LtvSeedUncertainty, scripts/ltv_feature_passive/geometry and geometry contract draft. No changes to estimator P or acceptance criteria.

B /root/manager: new LtvFeatureHistory, LtvFeatureQuality, LtvLandmarkManager, own test_ltv_feature_manager.cpp. No existing bridge edits.

C /root/audit: independent read-only inspection and new audit scripts/drafts. Never changes tested formulas to obtain PASS.

All share one filesystem with disjoint file ownership. No agent launches replay, simulation, compile, or MC without main grant; maximum two heavy tasks including main. No push. All attempts/retries count in the same ledger.
