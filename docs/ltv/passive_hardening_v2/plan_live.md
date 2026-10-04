# Execution plan and live checkpoint

Status: DEVELOPMENT_IN_PROGRESS. This is not final engineering acceptance.

Historical E1–E6 are complete at a8c56c8 (remote main): total64, new6, new failures0. Historical results and C*=C6 are unchanged.

R1 delayed startup and R2 repeated restart have retained negative evidence. R3 candidate123 preserves finite constrained state when 1–14 observations remain, while declaring not ready. Zero-observation timeout, physical faults and fixed norm guards still stop the observer. Gain/P0 and main OpenVINS equations are unchanged; no main-VIO velocity/gravity bootstrap is used. Hardening defaults OFF, G/V injection remains zero.

R3 full REGULAR and FAST development runs preserve all raw support and exactly match P_PREV raw metrics; repeated restart peaks disappear. Native V2_02 has exact main state/P/FEJ/trajectory parity, complete logging, zero injection and passes the single-sequence output contract. Dedicated TEMPORAL development accepts1794 seeds, 96.767% reliable and98.355% opportunity acceptance; no same-input P_PREV exists for that case, so raw regression is untested. Recovery retains the initial interrupted supply segment; the later stable segment meets the five-second-within-ten-second contract. Do not relabel the conservative all-segment summary as passing.

Correctness evidence includes full OFF x/P/IDs/time parity156, assertions-enabled integrated tests148, high-ID cache145, full-state heavy/light diagnostic parity155 and live typed-event conservation151. Independent assertions-enabled pose/GV/FEJ-gauge/joint covariance tests172/176 and composite seed Jacobian/gauge177/178 pass against cd3. Earlier Release assertion-disabled tests are explicitly insufficient proof, retained in the ledger. The repaired 600s resource run159 and audit168 cover actual retired-ID rejection5859 times; bounded records/samples/maps/IMU pass, mean/P95 construction+ABI13.164/13.923ms. This is development evidence, not frozen final performance acceptance.

Logger-only native runtime cd3bc7f4896d85fd629845e50a87ee56e5768d15672a8cfcbca6cc7760971123 adds actual per-point correction IDs/substeps. Wrapper04 build173, ten fixtures174 and complete x/P/slot plus corrected-ID heavy/light parity175 pass against that runtime. Native new-field short schema verification remains pending. Negative-only heavy-tail profile tests171 pass; normal frozen input SHA is unchanged. Historical midpoint inputs have explicit new-identity bridges166/167, not yet full observer runs.

Budget through178: synthetic16/80, real3/72, short_real8/24, geometry0/30000, candidates3/12. Final reserves remain intact. Authoritative ledger: /home/he/output/ltv_passive_hardening_v2/attempts.jsonl.

Pending: native latest logger schema check; V2_03 development; historical pressure regression; formal immutable source/config freeze; unseen confirmation seeds; all11 native EuRoC and representative repeats; metadata-selected UZH support review; final source-stratified, recovery and resource confirmation; complete acceptance audit and final report. No final all11 or unseen-seed success is claimed. Continue the active goal after this remote development checkpoint, without starting G/V fusion.
