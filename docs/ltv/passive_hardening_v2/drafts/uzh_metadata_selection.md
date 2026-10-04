# UZH input-only selection and synthetic matrix gaps

The extension list is frozen as **indoor_forward_3, indoor_forward_6**. This establishes input eligibility only, not UZH performance. The historical indoor_forward_6 insufficient-ready-support result remains unchanged. If this round also lacks the required ready/reference intersection, the conclusion remains `UZH_NOT_ESTABLISHED` regardless of total input reference duration.

Selection used a fixed priority announced before this audit: the two already supported sequences indoor_forward_3 and indoor_forward_6, then the other previously release-verified names in lexical order; take the first two eligible sequences. No candidate errors, NEW logs, readiness output, or confirmation synthetic labels were read. Only existing GT timestamps, GT file SHA, sensor CSV timestamps, image existence and historical v3 release provenance were inspected. No solver, derivative fit, full replay or heavy job was executed.

The JSON companion records all paths, current sensor/GT hashes, release URLs and candidates. Current GT bytes match the six historical remote-v3 SHA records. This rechecks local identity against that verified release; it does not claim a fresh online release lookup or independent ground truth.

| Sequence | Conservative fixed-window reference duration, s | Eligible |
|---|---:|---|
| indoor_forward_3 | 49.398 | yes; selected |
| indoor_forward_6 | 30.098 | yes; selected |
| indoor_45_14 | 36.898 | yes; not selected |
| indoor_45_2 | 49.298 | yes; not selected |
| outdoor_forward_1 | 42.498 | yes; not selected |
| outdoor_forward_5 | 17.498 | no |

Duration calculation splits ordered finite GT timestamps at gaps greater than 0.010000001 s, erodes each contiguous segment by 0.05 s at both ends, requires at least 11 samples and intersects with the camera/IMU sensor span. This is conservative timestamp support for the fixed 0.1 s cubic derivative; it does not estimate velocity or evaluate G/V errors. Endpoint erosion also avoids crediting unsupported derivative windows. All indexed camera images exist. Full replay must still verify sensor delivery, initialization and the actual reference/ready intersection at the unchanged time offset. No camera offset, GT window, gravity or error criterion is adjusted by this selection.

## Formal synthetic matrix: concrete gaps before confirmation

| Registered condition | Current implementation | Required action |
|---|---|---|
| Near stereo REGULAR/FAST, 0.5/1 s | New generator supports all combinations, seeds 201/202 behind new freeze SHA | Freeze explicit input identities; never substitute these for far pressure |
| Temporal REGULAR/FAST, 1 s | TEMPORAL emits only cam0, same correlated pose and bearing noise | Run both registered scenes with explicit temporal opportunity denominator |
| Far pressure REGULAR, 0.5 s | Historical bridge preserves old far points, bearing/noise and midpoint timing, but its profiles reference historical seed101 | Add a separate confirmation far-input producer; do not relabel seed101 as201/202 and do not modify near generator geometry |
| Bad match plus heavy tail | WRONG_MATCH now combines old wrong-ID rule with independent 2% t3 tangent perturbations; test171 passed | Freeze profile/source SHA and generate only after final freeze |
| Stereo dropout, invalid time | STEREO_DROPOUT and TIME_ERROR present; asynchronous actual_ns explicit | Verify dispatcher scene/lifetime mapping is predeclared, not selected after results |
| Normal15/weak15/recover30 | RECOVERY present; fixed reliable-supply labels separate | Evaluate each continuous supply segment; do not replace reliable supply by visibility |
| Resource600s | Streaming fixed 42/43 workload, aged retired-ID reappearance, far TTL candidates and missing15–30 s; development159/168 verified | Freeze final wrapper/runtime and run final resource evidence; see seed issue below |

For pressure201/202, the smallest faithful addition is an independent new generator entry point that reuses the archived far `geometry(REGULAR)`, `ids_at`, analytic truth and original five `SeedSequence(seed).spawn(5)` noise streams without modifying old files. It should retain 30 distant fixed world points, old .11 m synthetic baseline and signed bearings, original OU/jitter and bearing/IMU scales, and 0.5 s staggered identities. Then apply the existing timestamped CSR bridge contract, keeping original midpoint samples at their actual half ticks plus explicitly copied boundary brackets. This creates a new discretization identity, not numerical equivalence to old midpoint propagation. Freeze geometry/noise/timing/source hashes; test a development seed against the old generator before accepting new confirmation seeds. GT and lifetime labels remain evaluator-only. New pressure seed permission must validate this task's frozen_config SHA, not the historical task's freeze. The old generator only allows42/43/101/102, so passing201/202 directly is currently rejected.

The resource workload is a boundedness/performance test, not a new independent precision sample. Its far unadmitted TTL points, expired-ID reappearance and no-observation interval are predeclared adversarial branches, with required nonzero TTL, active retirement and guard rejection; no accuracy failure in these branches may be silently deleted. Resource600s is not itself listed as a blanket scientific negative exemption in protocol.json: only the weak interval is an existing negative condition. The current resource CLI accepts42/43 only. Before final execution, the dispatcher must explicitly resolve whether the registered two confirmation seeds also apply to resource, or freeze resource as the separate deterministic engineering workload already defined. If201/202 are required, a narrowly scoped seed/freeze validation extension is missing; do not silently count42 as201. No change or run was performed in this audit.
