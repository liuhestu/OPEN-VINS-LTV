# Post-freeze management diagnostic recovery

The frozen runner's supplemental logger compared integer camera nanoseconds with
the result of a double round trip. This omitted seed/lifecycle diagnostics on
many real frames. The full cache and unconditional raw v/eta, readiness, main
state audit and trajectory remain intact. Original source, runs, metrics and
selection evidence are preserved; they must not be used for complete real
management statistics without this recovery.

`recover.cpp` is a read-only decoder/replayer linked against the selected immutable
runtime. It retains the frozen full-state/P/slot/seed/event comparator and emits
JSON only after each exact comparison. An explicit cache terminator is mandatory.
It does not recompile the estimator, alter selection, read GT, or change gates.
`provenance.json` records the source-copy audit. The unapplied
`runner_logging_only.patch` documents the logger correction for future builds;
it is not an alternative numerical result for this frozen study.

Execution from repository root is recorded in the append-only study ledger:

```bash
python3 scripts/ltv_feature_passive/diagnostic_recovery/build_recovery.py
python3 scripts/ltv_feature_passive/diagnostic_recovery/recover_all.py --binary /absolute/recover_SOURCE_SHA
python3 scripts/ltv_feature_passive/diagnostic_recovery/merge_overlay.py \
  --run /absolute/original_P_NEW_run --metrics /absolute/original_metrics.json \
  --recovered /absolute/diagnostics.jsonl --provenance /absolute/exact_report.json \
  --binding /absolute/binding.json --out /absolute/new_overlay_directory
```

The six full replays consume six full-real budget entries. They are not free
analyses. `recover_all.py` refuses restart or overwrite; failure requires explicit
ledger reconciliation, and there is no refund. The merge itself is offline
analysis, must still use the shared scheduler, and never re-reads GT. It matches
original-input double time plus epoch/sequence exactly, preserves all protected
raw fields, checks cumulative births/retained counts, and rebuilds only management
metrics using the frozen evaluator. Error arrays are copied byte-for-byte.

Candidate TTL expiry before admission and active-landmark removal are distinct
counts. Epoch reset is reported separately. A repeated original cache that is
byte-identical implies identical recovered diagnostics for this deterministic
reader; this implication is not an extra executed full replay.

Original defective summaries remain historical evidence, not final management
results. The corrected final manifest explicitly points to the overlays and their
SHA-bound exact-cache recovery. No observer parameter or confirmation criterion
is changed by this repair.
