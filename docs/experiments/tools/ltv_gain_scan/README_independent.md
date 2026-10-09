# EuRoC independent G/V weights

Run `test_independent.py`, then `independent.py prepare /unique/coord`,
`independent.py run /unique/coord`, `audit_independent.py /unique/coord`, and
`independent.py publish /unique/coord`, with OPENBLAS_NUM_THREADS=1,
OMP_NUM_THREADS=1 and PYTHONDONTWRITEBYTECODE=1.

The seven information weights are 1,2,3,4,5,6,8. G and V independently use
sigma_g=10 degrees/sqrt(G), sigma_v=1 m/s/sqrt(V). Zero in an identity means
that branch is disabled, not a zero noise variance. Landmark stays disabled.
Native math, Observer Q/V/P0, gates and update order are unchanged.

Exactly 100 prior EuRoC identities are reused (OFF, G/V at 1/2/4, diagonal GV
at 1/2/4). Preparation verifies sensors and copies the original frozen OFF
supports. New run/config identities explicitly carry both axes. No historical
tier is replayed. Prior data used four OpenCV threads; new runs use the verified
single-thread driver. No timing comparison mixes these settings.

New single tiers first run V2_02, V2_03 and MH_05. Runtime/support failure or
ATE strictly over 10% above OFF or the same-mode 1x reference blocks expansion.
After full singles, all ten cases must meet the same checks to admit a branch
weight into the Cartesian product. Each new joint pair is separately screened
against OFF and GV(1,1). Existing diagonal results are reused. Unstarted cases
are explicit BLOCKED entries; completed representative measurements remain.

Eight dynamic worker queues have separate CPU affinity, ROS domain/namespace,
registry, process group, output, log and temp directory. Shared replay slot
locks live at /home/he/output/ltv_shared_replay_locks. Source, verified artifact,
configs and input supports are shared read-only. The coordinator alone writes
indices. Failed preparation attempts are retained in their original directory.

All seven tiers and all 49 possible pairs are recorded in
`docs/euroc_tune_results.md`, with improving metrics and qualifying summary
parameter labels in bold. Evidence lives under
`docs/euroc_tune_results/gv_independent/`. Maximum new full replays: 540.
Screen PASS means permission to expand, not engineering acceptance. Historical
prediction FAIL and engineering conclusions are retained. The independent audit
checks historical reuse, inputs, effective weights, actual threads, gates,
position ATE and the eight-process limit.

User amendment: after the initial four-worker single stage, eight workers each
receive two distinct logical CPUs (16 total). A verified coordinator SIGINT
drained all initial jobs; 21 completed FINISH records were recovered before
resuming without any duplicated full identities. Eight simultaneous control
probes independently checked affinity, ROS domain, namespace, temp and logs.

Additional user request: once the base seven-tier stage completes, run
`independent.py extra /same/coord` to screen and extend G/V at 10 and 16 only
(maximum 40 new full replays). The 7x7 GV grid is unchanged. The final report
shows all seven original singles and the two explicitly appended high tiers.
