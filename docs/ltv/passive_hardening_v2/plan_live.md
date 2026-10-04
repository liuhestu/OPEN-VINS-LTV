# Execution plan and live checkpoint

Status: DEVELOPMENT_IN_PROGRESS. This checkpoint is not final engineering acceptance.

Historical six supplemental experiments E1–E6 are complete at a8c56c8; historical total64, new6, new failures0. Those results, production configuration and C*=C6 are unchanged.

A0 protocol and acceptance are frozen. A1 live receipts, atomic lifecycle integration, readiness and independent rejection tests pass. R1 is rejected as a final candidate: raw regression and FAST coverage/repeated transients fail despite improved ready accuracy. All negative evidence is retained.

A2 R2 bounded initial zero warmup builds and passes adapter and independent policy tests (92/94/97/98). Full REGULAR P_PREV/P_NEW_WARMUP runs100/101 and offline tests/evaluations102–104 are complete. Full default-OFF x/P/ID/time parity105 is exact. R2 REGULAR raw metrics equal P_PREV; this is one development condition, not final confirmation. Temporal source, recovery, FAST, native R2 and final all11 evaluation remain pending.

A2 bounded-resource integration remains pending; the identity guard is standalone tested but unused. A3 final freeze, unseen seeds, all11 native EuRoC, repeats, recovery, 600s memory and performance checks remain pending. A4 full acceptance audit remains pending. G/V injection remains zero; hardening defaults OFF.

Budget: synthetic6/80, real1/72, short_real6/24, geometry0/30000, candidate2/12. Final reserves24synthetic/40real remain intact. No full simulation is active at this checkpoint. The ledger snapshot records commands and exit codes through106. Live authoritative ledger: /home/he/output/ltv_passive_hardening_v2/attempts.jsonl. Do not duplicate completed runs.

Next: evaluate R2 in FAST, native development and predefined recovery, then implement bounded-resource behavior with default-OFF equivalence. Use the immutable R2 runtime67a97cc62d86f380e7910e42b4ad4195a6677c8057ff691b746a9078393b638b and wrapper02 for this source identity. New code requires a new build/snapshot. Source must not change during builds.
