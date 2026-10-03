# Independent scheduler and acceptance review

Static review of `budget.py`, `protocol.json` and `acceptance.json`; tests authored but not yet executed. Main agent controls compute authorization.

Execution update: main authorized scheduler test attempt `0005` under the authoritative ledger. `test_budget.py` exited 0; five tests passed. Main added child `pass_fds` inheritance before this execution. The killed-parent test demonstrated one slot remained occupied while the child survived; short-duration validation, durable failed-start count, capacity and third-slot rejection also passed. Test ledgers were isolated temporary roots, so no synthetic/real budget was consumed. PID-incarnation recovery, reserve preservation and actual runner input-duration enforcement remain integration responsibilities.

The protocol preserves the required 48 synthetic / 36 real / 16 short / 25000 geometry / 6 candidate limits, 16 synthetic and 20 real confirmation reserve intentions, development/confirmation split, C0, real repeats, injection OFF, two heavy slots, fixed OU/shared-noise specification and final scientific thresholds. Acceptance explicitly distinguishes engineering, seed reliability and same-support real improvement/coverage. These are contracts, not passing evidence.

Scheduler issue: its original slot FD belongs only to the scheduling parent. `Popen` normally closes it in the child. Killing that parent releases its lock while the observer/replay child can remain alive; a later launch can then exceed two heavy tasks. The original code also promises explicit running-PID reconciliation in its docstring but does not inspect unfinished events before launch. Retain the lock in the child (`pass_fds`) and record PID incarnation, or implement an equally robust reconciliation mechanism. Any build wrapper must wait on grandchildren; daemonized computation cannot escape slot ownership.

Additional checks for main-owned dispatcher:

- Reserve counts are durable before launch and failed starts retain usage. A slot admission failure before reservation is not a simulation attempt.
- `short_real`'s declared duration alone does not prove actual ≤10 s input. The runner must compute/log last−first input span and evaluator must enforce it.
- Confirmation reserve is currently protocol intent, not enforced by generic budget.py. Stage orchestration must deny development reservations that consume the final 16 synthetic / 20 real slots absent exact-identity reusable final evidence.
- General CLI has `argparse.REMAINDER`: use/document options before positional kind or change parsing so explicit identity is not consumed as command text.
- `units` must be positive integers. Programmatic fractional/NaN/infinite counts should be rejected, not only CLI-parsed integers.
- Explicit reconciliation must preserve interrupted attempts. Restart-from-beginning counts again; no incomplete result reuse. Same PID without process starttime/boot identity is weak evidence of the original child.

Authored independent `audit/test_budget.py` uses temporary ledgers only and includes an orphaned-child slot test. It does not start observer simulations or read datasets. Run only after main authorizes a compute slot. Its orphan test currently expects the inherited-lock design and should be adapted if an equivalent different safety design is chosen; do not weaken the underlying no-third-heavy-child requirement.

Minimal main-state audit additions can live in runner/VioManager diagnostics only. Enumerate public state families with sorted map keys, type IDs and value/FEJ dimensions; include full covariance and clones. Capture pre-update and post-update MSCKF feature ID order, SLAM update/init order before cleanup, tracker observations and trajectory per camera event. Assert actual gravity/velocity accepted/submitted counts zero in startup and every event. This establishes stronger scope than the existing post-cleanup tracker digest or IMU-only ValueDiagnostics record without changing any estimator formula.
