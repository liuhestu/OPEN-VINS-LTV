# Append-only decisions

1. New authorization supersedes prior stopping boundary only within passive_hardening_v2. Baselinef76f32f preserved as archive and tracked SHA before edits. New branch feature/ltv-passive-hardening-v2. Three role agents active, one ledger/two heavy slots. Acceptance section10 fixed before new outcomes; recovery sustained5s must finish within10s. Physical G-health norm8.0–11.5 frozen. EuRoC11 folders locally present; metadata audit pending. Old budget/evidence untouched.
2. Independent preflight clarified mandatory reporting flags and explicit time-correlated OU pose noise; no numerical acceptance threshold changed and no new experiment has begun. Prior evidence shows REGULAR0.5s never reaches15observed, while FAST1s firstVready has6.632m/s error and6.25% severe-ready fraction. Test delayed start and shared-state readiness separately; do not assume accurate seed solves either issue.

## Logger checkpoint and initial integration

Logger-only native comparison attempts17/18 passed on a fixed 10-second V1_01 prefix: numerical cache and main outputs exact, 200 live receipts complete without overlay. Checkpoint 5dd19ce pushed. This does not complete hardening. Attempts19–21 rechecked receipt/budget/log evidence. Synthetic CSR input test22 passed. Independent readiness test24 exposed stale delayed bootstrap acknowledgement; manager revision3 rejects that entry point, unit25/26 and independent28/29 pass without weakening the counterexample. Configuration27 passed. Build30 is the first integrated native build and is still in progress as of this entry. No full candidate has run.

## A1 integrated checks and first mechanism candidate

Build30 failed in new cache schema serialization (temporary string passed to mutable reference); fixed locally, build32 passed. Adapter33, readiness34, controlled/manager/pipeline/timing35–38 passed. Short39 OFF preserved old full numerical cache byte-for-byte. Short41 and replay42 retained numerical exactness but lacked new logger fields: source had been edited while runner compilation was in flight, leaving a stale object; the initial checker skipped absent fields. The false overall interpretation is explicitly superseded by evidence/integration_short_01_correction.json. Mandatory schema checks replaced the skip. Forced runner rebuild43, short44, replay45 and strict public-schema48 all pass: 200 receipts, 5 cold managed events, 83 current raw rows; main state/audit/trajectory exact, no injection. The failed first engineering interpretation and all consumed short runs remain counted.

Synthetic wrapper46/fixture47 passed without evaluator labels present. Independent rejection49/50 passed: invalid context/calibration does not mutate observer/manager; camera rejection after valid IMU prediction preserves exactly that predicted x/P without seed/slot/admission commit.

First development candidate registered at51: R1_DELAYED_HEALTH_C0_ZERO_START. Frozen development input52 uses EuRoC-calibrated near points, REGULAR motion, 0.5s identity lifetime, seed42 and shared OU priors. Endpoint IMU input differs explicitly from the old midpoint direct-core input; no cross-identity numerical reuse is claimed. P_PREV/P_NEW full comparisons have now begun under the shared two-slot budget. This is development, not final confirmation.

## Development checkpoint through attempt105

R1 REGULAR53/54, FAST77/78 and native V1_01 run67 completed. R1 raw eta regression (REGULAR/native) and FAST low coverage/repeated peak reject it as a final candidate. Full negative timelines and same-support comparisons are retained. Real reuse is supported by exact parsed sensor inputs, complete non-hardening configuration equality and main state/P/FEJ/trajectory equality, not by matching sequence names.

R2 candidate99 preserves the initial legacy zero-state trajectory while gating ready independently. Builds85/92 and tests91/94/97/98 pass. Test90 initially expected the DORMANT trigger frame to erase current raw; corrected to the frozen contract: actual trigger-frame state is retained, subsequent propagation stops. No deadline or accuracy tolerance was relaxed.

Full REGULAR100/101 completed; evaluator102 (12 tests), analyses103/104 and full OFF numerical parity105 pass. R2 raw metrics equal baseline and ready absolute metrics pass in this one condition; Temporal source and recovery are not established. The evaluator treats identical undefined-angle sets as comparable only with common finite evidence, preserves every undefined time, and still rejects NEW-only undefined values. Prior evaluation results remain unchanged.

Checkpoint budget6 synthetic/1 real/6 short real/2 candidates. No final acceptance claim: bounded memory, all11 native, unseen seeds, recovery and runtime remain outstanding.

Historical closeout tests rechecked as attempt106: exit0, no new full simulation. Three independent test sources were clang-formatted after execution; implementation source is unchanged.

## R2 wider development evidence and resource implementation

Previous goal turn made progress: verified/pushed3389ae2 checkpoint; no final acceptance claim. Full FAST107 and native V1_01 run108 completed; analyses109/110 establish native single-sequence success but FAST failure. V1 raw2800/2800 exactly matches P_PREV, V-ready RMSE0.055919/P950.098186, G/Vcoverage81.929%/79.107%, joint20.75s. G-reference exception stays diagnostic. FAST raw28.643%, G/Vready4.496%/3.747%, joint2.20s and repeated zero bootstrap at0/30/60s fail coverage and raw regression. Thus R2 is not a final candidate.

Resource implementation is a required engineering correction, not a gain or acceptance change: hardening-only bounded history21samples/track,256totaltracks,512observations/packet,2cameras,32contextposes; IMU buffer4096 with explicit overflow fault. Manager uses nondeleting1MiB identity guard plus bounded exact live records and typed retirement events. Adapter maps only retained IDs; internal IDs monotonic, core uses high-water mark only by explicit opt-in. Old mode retains its set semantics. Header-level history111/112, core113/114 and manager115/116 tests passed; integrated build and independent review remain pending. Puremanager600s churn test is not a fullobserver600s resource acceptance.

Independent recovery input117 was generated without estimator output, then fullruns118/119 completed. Evaluation pending. No unseen confirmation seeds were opened.

### Release test-contract correction

Inspection of generated flags found CMake Release uses -DNDEBUG. Several inherited assert-based tests (including controlled/history/manager) could report success without exercising assertions. Independently compiled115/116 and126/127 had no -DNDEBUG, and readiness/adapter use always-on require checks, so those proofs remain valid. Integrated assert-based results124/131 are not sufficient as correctness evidence. The test directory now adds -UNDEBUG only to test targets, preserving production flags; rebuild and active-assert rerun required. All original attempts remain recorded, not relabeled as full proof.

## R3 preserve constrained state: first complete result

Candidate123 was registered before R3 runs: preserve finite current x/P with1–14 current constraints while ready remainsfalse; no-observation1s, physical faults and fixed100m/s/50m/s² norm limits still enterDormant. No pose-aided V/G copy and no gain/P0 change.

Build129 and always-on policy/adapter tests passed; frozen runtime132 is a62bdfabaf666de8b7b06a00ec65d394fc9fc9d3d53fe91813be534fd3764a70. Native short134/136 and cache135/137 pass; OFF full numerical cache remains byte-identical, main state/audit/trajectory exact. Wrapper138,10 shortfixtures139 and3s streaming sanity140 passed. Highuint64 cache independent143/145 passed66 events including typedTTL/retirement/rejected oldIDs and exactreplay; build142 missedOpenCV include, command-only repair retained.

FullFAST141/evaluation146: all1201raw current, raw v/eta/angle metrics exactlyP_PREV, onlyt0bootstrap. At14.85s11observations nowDegraded and retainedx/P, no30/60srestart. Severe ready0; coverageG22.648%/V21.898%, jointmax2.30s. Scope mapping fromoriginal§10.2 vs§10.4 retained: EuRoC5s/coverage targets are extra synthetic diagnostics, severe/recovery remain synthetichardrequirements. No historical JSON or threshold altered.

Assert-enabled test rebuild144 and full8-test rerun148 PASS. Rebuilt production librarySHA is identical to frozena62b library; only test flags changed. NativefullV2_02 run147 completed2348packets/2266initializedraw, precision evaluation pending. Full600s streaming resource149 now running; cannot claim long-run resource acceptance from3s fixture.

## R3 development checkpoint through171

REGULAR152/153 and evaluation157 have complete raw support and exact baseline raw metrics. Full OFF parity156 and full-state diagnostic parity155 pass. Native V2_02 evaluation150 and typed log audit151 pass the one-sequence contract and event conservation. This does not establish all11 acceptance.

Resource149/154 had valid bounds/performance but missed retired-ID rejection because reappearing IDs were still coasting. Input fixture158 corrected the coverage, full retry159 counted normally; audit168 finds5859 guard rejects, all known retired IDs, with all bounds and runtime limits passing. Both attempts remain in the budget.

Recovery160/evaluation169 retains the30.1–31.5 interrupted supply episode. Stable supply starts31.7, wide joint interval35.35–40.35 completes within41.7 deadline; the strict all-segment summary remainsfalse. TEMPORAL162/evaluation170 accepts1794 seeds, reliability96.767%, opportunity acceptance98.355%, no stereo opportunities. It has no same-input P_PREV and does not claim raw regression pass.

Historical input bridges165–167 preserve archived inputs, truth and original midpoint timestamps, with explicitly declared endpoint guard samples; they create a new Adapter integration identity, not an exact historical reproduction. Negative-only heavy-tail tests171 preserve normal input SHA. Native logger163/runtime164 adds actual corrected IDs and substeps; wrapper rebuild and fixtures are required before use. No formal confirmation data have been examined.

Wrapper04 build173 and ten input-isolation/receipt/logging fixtures174 pass against immutable cd3. Test175 verifies exact complete x/P/slot plus corrected-ID/substep equality with heavy diagnostics on/off. These are short correctness tests, not new full simulations or final acceptance.
