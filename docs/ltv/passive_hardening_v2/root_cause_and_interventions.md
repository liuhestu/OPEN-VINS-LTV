# Development evidence and interventions

This document records development evidence, not final acceptance. Historical standalone64 results and C*=C6 remain unchanged. This task uses unchanged C0 gains and zero G/V injection.

## R1: delayed zero startup plus health-based availability

The naked pre-seed integration hypothesis motivated delaying the whole observer until coherent reliable seed supply. Engineering tests show that this removes unobserved integration during collection and supports atomic seed/core/manager commit, explicit unavailable raw, bounded shortage-to-dormancy timing and separate physical faults. The first public-schema validation was too permissive; its failure and repair are retained in decision_log.md. The new native runner and exact replay are validated without overlay.

| Development condition | Finding | Limitation / decision |
|---|---|---|
| EuRoC-calibrated near REGULAR, lifetime0.5s, seed42 | NEW V-ready RMSE0.02543m/s; severe V-ready0; stereo accepted3560, reliable99.44% | This largely reflects stricter ready support, not improved raw dynamics: on the same NEW ready support, PREV is already about0.02506m/s. NEW raw velocity peak6.590m/s vs PREV6.522m/s; raw eta/angle regress. Temporal51 samples cannot establish its source contract. |
| Near FAST, lifetime1s, seed42 | Severe ready errors removed, ready V RMSE0.0538m/s | Raw available only27.89%; V-ready3.75%, longest joint2.20s. Bootstrap at0.2/30.2s; repeated velocity peak10.436m/s at32.2s. All three raw comparisons regress on common support. No favorable support trimming is treated as raw improvement. |
| Actual complete V1_01,2912 input camera packets | Initialization denominator2800; NEW V-ready coverage78.96%, reference109.55s, RMSE0.05516/P950.09820m/s; longest joint20.75s; main state/fullP/FEJ/trajectory exact to historical B/P_PREV after input/config equivalence proof | NOT_MET: raw eta RMSE0.94529→1.02864 exceeds permitted0.05 increase. Only five initial collecting packets have no NEW raw; they remain in coverage denominator. V1 gravity-angle reference remains diagnostic as predeclared. |

REGULAR and FAST are newly identified near/FOV/endpoint-IMU inputs, not numerical reuses of the historical far/midpoint inputs. Their geometry and noise identities are retained. Both normal runs saved2402 full P snapshots; finite state, symmetry and PSD checks passed. No recovery stage was present, so neither run establishes the ten-second recovery target. Full timelines are in figures/R1_REGULAR and figures/R1_FAST.

R1 is rejected as a final candidate. It demonstrates that readiness can suppress misleading declarations, but delaying zero initialization does not remove the velocity/gravity transient; full quality reconstruction can repeat it. This is evidence against equating reliable landmark seeds with a reliable current shared state.

## R2: bounded initial zero warmup (development condition verified)

The next minimal intervention preserves the legacy observer's initial complete causal x/P/slot evolution while independently withholding ready until observer-health checks pass. It does not copy main-VIO velocity/gravity, change C0/Riccati equations, loosen accuracy/coverage thresholds, or repeat landmark writes. Initial warmup is a single explicit event, uses the same pending-request/same-event acknowledgement transaction, and records the quality bootstrap clock immediately. Inadequate observations for the fixed one-second interval still lead to DORMANT; epoch changes cannot re-enable unseeded initial warmup, and physical-fault recovery still requires seed supply.

The causal test is exact whole-state/covariance/slot agreement with the legacy initial trajectory under the same input. The scientific test is whether the R1 raw-regression failure disappears without sacrificing truthful ready and required coverage. Adapter tests and independent tests pass. Full REGULAR runs100/101 have identical raw v/eta/angle RMSE (1.100285825m/s, 1.584542047m/s², 7.775139508°); default-OFF x/P/ID/time parity105 is exact over the full domain. NEW G/V ready coverage is91.840%/90.258%, with G-angle RMSE0.10266°, V RMSE0.02506m/s, longest joint54.15s and severe-ready0. Both methods have the same undefined angle at t=0,0.05,0.10s; the remaining1198 common finite values support the comparison. Evaluation explicitly preserves these undefined times. Stereo3568 accepts are99.439% reliable, opportunity acceptance98.292%; Temporal49 remains insufficient. These are development observations, not unseen confirmation. Recovery, FAST, native R2 and later quality reconstruction remain outstanding.

## Resource correction still required

Long-run manager tombstones, the adapter ID map and core admission history remain unbounded in the current running candidate. A fixed1MiB non-deleting identity guard has been implemented and tested independently, but it is not yet wired into these structures. Its possible false-positive rejection must be reported; zero false-negative duplicate protection cannot be replaced by assuming frontend IDs are monotonic. Full600s bounded-memory and timing evidence remains missing.
