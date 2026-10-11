# Independent review of the post-freeze diagnostic omission

This reviews a tool defect in the new runner's diagnostic branch. It does not authorize changing the frozen observer, selection scheme, uncertainty formulas or thresholds. No new computation or confirmation GT access was performed for this review.

## Root cause and responsibility

The runner's `feature_row()` emits its supplemental manager data only when `f.camera_ns == camera_ns`. The right side is the original integer ASL timestamp; the left side comes from `llround(t * 1e9)` in LtvAdapter, after the original nanoseconds were converted to floating-point seconds. At absolute dataset timestamps, that round trip does not preserve every integer nanosecond. The guard can therefore suppress legitimate current-frame diagnostics. This guard was introduced in the independent runner work; earlier tests verified whole-file repeatability but did not test nanosecond round-trip disagreement. Equal defective logs do not prove diagnostic completeness.

The rejected branch does **not** control observer execution or native visual updates. It is reached after both have executed. In the runner, v/eta, readiness, availability, physical target time, observer time, raw tracker ID sets, state/observed/mature counts and native audit/trajectory output are all emitted outside that guard. Inside it are retained IDs, cumulative manager counters, seed/source/uncertainty records, lifecycle records and retirement records. Those supplemental real-data summaries are incomplete and cannot support their previously reported counts until repaired.

Synthetic metrics use their separate observer harness/event writer and do not use this integer-time guard. Thus synthetic seed reliability is not affected by this code path. Frozen real G/V criteria use the unconditional fields; their non-change must still be demonstrated by field-by-field overlay comparison and exact re-evaluation equality, rather than merely asserted from static inspection.

## Safe repair boundary

Keep the frozen runtime and original feature logs/caches/manifests intact. Create a separately identified recovery executable that reads the original cache and links the exact frozen libraries. An isolated copy of the cache reader may add a **read-only** diagnostic callback. Its original exact replay comparison must remain active for every event and continue comparing state, full P, snapshots, IDs, births and rejected seed diagnostics. The callback cannot alter options or estimator state. Save the patch, executable, linked-library resolution and input cache SHA.

Join recovered process/pause events to original camera packets using causal event order, actual double camera time and epoch/sequence/version, with uniqueness and completeness checks. Never require an integer→double→integer timestamp round trip to equal the original integer. Preserve original `camera_ns` as the exported row key. A bounded physical-time tolerance may diagnose the original quantization difference, but must not fit or change sensor time offsets, merge adjacent frames or match arbitrarily many rows.

Write corrected supplemental data to a new overlay or complete derived log with explicit provenance. Do not silently replace original files or pretend the recovered fields were captured live. Keep unconditional estimator/native fields exactly unchanged. Updated reporting may point to recovered manager counts while keeping original raw G/V curve evidence and its SHA.

## Required tests before six full recovery replays

1. A fabricated nanosecond timestamp not exactly representable as double seconds must reproduce the old guard omission; a neighboring camera event must remain separately matched.
2. No process event can be attached to two camera rows, and no available process event can be omitted. True pause/stale events must not inherit the previous seed list.
3. A budgeted existing short real cache must pass the original exact replay comparator, then recover manager records for both old-guard-pass and old-guard-fail packets. Fields that already existed must agree numerically with recovered values.
4. Per frame, retained-ID count must equal observer state-feature count. Every birth must have a current observation and exactly one `(epoch, feature_id)` write/admission event. Per-epoch cumulative admitted counters must match the reconstructed unique admission events, including reset boundaries.
5. Report active retirement separately from never-admitted candidate TTL expiration. Original manager `retired_ids` covers active removals while candidate TTL tombstones also exist in manager state; a final or change-event snapshot must capture these rather than perpetuate a second completeness gap.
6. Original and derived logs must have identical packet keys/order and identical unconditional v/eta/readiness/timing/raw-ID/native fields. Recomputed scientific G/V decisions, coverage and errors must be exactly unchanged; only declared supplemental management fields may differ.
7. Missing/truncated cache, failed exact replay or unmatched events cannot yield a repaired PASS. Preserve failures; every complete cache observer replay counts against the full real budget, and every short replay against its applicable budget.

## Budget and interpretation

Six remaining full real slots can cover the six final primary P_NEW caches. Existing prescribed repeat checks already established byte-identical original caches between original and repeat. For repeated sequences, equality of corrected deterministic diagnostics can therefore be inferred from the identical cache bytes and same recovery implementation, but this is not an additional executed full recovery repeat. Label that distinction.

A truthful final report must distinguish the completed algorithm validation from this subsequently repaired instrumentation omission, identify which management counts were corrected, and state that no outcome-driven parameter change followed confirmation. If recovery cannot satisfy completeness and unchanged-G/V checks within budget, retain the unaffected output evidence but mark the affected feature-management evidence incomplete; do not manufacture counts.

## Independent recovery implementation review

Read-only source comparison confirms `Binary`, context/calibration/seed decoding, full `evidence()` and `same()` are unchanged from `LtvPassiveCache.cpp`; only the anonymous namespace closes at a different location. The recovery replay body is identical after removing the diagnostic callback, which executes strictly after each exact comparison. Complete-cache footer and trailing-byte checks remain. No numerical/runtime source edit is part of this repair.

The overlay replaces only retained IDs, opportunity/admission counters, retirement IDs, seeds and tracks. It checks unchanged G/V, readiness, epoch/sequence, cursor, mature/state/observed counts and causal raw frontend IDs; all other original fields survive a deep-copy with explicit non-management equality assertions. Cumulative unique admissions must match the manager counter, retained identities must be admitted and match the core active count. Candidate TTL and admitted removals are emitted from the complete manager map, not the filtered per-frame track list; monotonic cumulative sets and disjoint populations are checked. Reset is explicitly excluded from ordinary retirement counts.

An interface defect was found before overlay use: dispatcher binding records do not carry the standalone `sourcehash` key initially required by `verify_binding`. Main and geometry agreed to resolve this by validating the existing build-proof file and its source/binary/freeze/runtime identities, without modifying the running dispatcher or recovery executable. The final proof must validate hashes and identity equality, rather than mere presence of provenance strings. This review does not itself execute recovery tests or claim their outcomes.

## Delivery recommendation

This round can be delivered as completed research **if** all six exact recovery runs and their overlays pass, source/runtime/cache/output provenance is bound, and unchanged scientific G/V evidence is explicitly checked. Retain frozen original sources and evidence. Make the recovery command and overlay selection part of the authoritative reproducible reporting workflow; provide the logging-only patch as a future correction, with a clear statement that it is not the executable used for confirmation.

An unapplied patch alone is insufficient, and the frozen live logger cannot be described as fixed. The repaired product for this round is the frozen capture-plus-exact-recovery pipeline. Its raw supplemental management logs remain known-incomplete; only the verified overlay supports corrected counts. Do not change selection, thresholds, scientific outcomes or the original frozen protocol. If any replay or completeness check fails, the affected management evidence remains incomplete even though unaffected G/V evidence remains valid.
