# Actual Observer error-source shadow

The implemented read-only shadow changes neither Observer mean nor Riccati
`P_R`. Enable it with `LTV_JOINT_SHADOW_PATH=/new/absolute/path.csv`.
It writes the summary CSV plus `.matrices.jsonl` offsets and `.matrices.bin`
lossless matrices. Existing files are rejected. Diagnostics share their stream
across the hardened adapter's transactional Observer copies and epoch replacements.
All emitted records have `valid_unconditional=false` and
`transaction=observer_attempt_uncommitted`. The hardened adapter stages a copied
Observer and may reject its attempt. Matrix/anchor/statistical state belongs to
that Observer object and rolls back with the copy; only the append-only sink is
shared. Logs can therefore retain rejected attempts. LC-07 coverage must join
successful feature receipts/time/epoch and cannot call these rows “applied” or
“committed” merely because they exist.

## Actual discrete maps

The production IMU mean uses explicit Euler: `F = I + dt A`.
Its corrected acceleration/angular-rate perturbation map has acceleration `dt I`
in velocity rows and angular-rate `dt skew(x_j)` in every point/velocity/gravity
triplet. Each source is shared across all triplets. A persistent corrected-input
offset sensitivity is propagated as `T_next = F T - B`, not reset at every IMU
step. This is a unit source sensitivity, not a physical bias covariance: the
adapter's calibration, bias injection and interpolation still need source maps.

At camera substep j, with nominal Riccati gain held fixed,
`F_j = I - K_j C` and the tangent-bearing source map is `K_j D_j`.
For point displacement `v = p_BC - x_j`, camera bearing u and body bearing z,
`D_j = (-z.dot(v) I - z v^T) R_BC (I - uu^T)`.
The SAME source is retained throughout a camera event:
`B_total_next = F_j B_total + K_j D_j`.
The emitted camera Gram is `B_total B_total^T`, not a sum of independently
sampled substep covariances. The source units and unit covariance are explicitly
diagnostic; `q_landmark` is not silently identified with physical bearing noise.

The generic joint covariance map includes `D=Cov(old error, source)`:
`F Sigma F^T + B Q B^T + F D B^T + B D^T F^T`.
An independent explicit augmented covariance multiplication checks this map.
In the actual camera diagnostic only the isolated fixed-initial-condition source
contribution is mapped, with zero *conditional* initial error and unit tangent
source covariance. Unknown unconditional blocks are never set to zero.

`P_R` uses its own Euler Riccati update and eigenvalue floor. It differs from the
actual mean-error map covariance. In particular neither `F Sigma F^T` nor
repeated-frame source cross terms equal the current Riccati Euler equation.
Gain-schedule perturbations, Riccati eigenvalue sanitization derivatives, true
trajectory discretization error and nonlinear higher-order terms are absent
from the present conditional map; unconditional covariance remains BLOCKED.

## Lifecycle and anchor contribution

Each lifecycle record contains the actual permutation/retirement matrix M and
retained local IDs. `T_next = M T`; birth rows have zero sensitivity only under
the explicit fixed-seed conditional experiment. They are not a claim that the
physical seed is independent of input or the main state. At first retained point appearance, its then-current 3x6 source map initializes
a diagnostic anchor with `lag_available=false`. At each camera the saved
previous-camera map and physical time are used to emit aa/tt/at. Only AFTER that
output does an explicit `anchor_replace` event save the current map/time as the
next diagnostic anchor. This rolling previous-camera anchor supplies nonzero
cross-time source contributions; it is not yet connected to the candidate
residual's mature primary anchor. Retirement erases it, and attempting to resurrect a retired
ID in the same epoch is rejected. Reset clears all anchors/retired identities;
epoch replacement clears identity bookkeeping. Camera records contain full
anchor and current maps and their unit-source `Sigma_aa`, `Sigma_tt` (including
cross-point blocks), and `Sigma_at`. These are partial known-source contributions;
`C_xa/C_xt` and physical total Sigma remain UNKNOWN. First-birth anchors often have zero conditional map because the seed is held
fixed; they have no valid lag snapshot. Later rolling replacements retain the
accumulated source map. Zero first-birth contributions do not prove physical zero
anchor uncertainty. The independent reader checks that every available anchor
map/time matches the preceding replacement snapshot, rather than merely checking
the aa/tt/at formulas against each other.

Run `python3 read_shadow_matrices.py /absolute/path.csv` to decode matrices and
independently recompute every camera Gram and anchor/cross-point block. Sparse
records contain native uint32 row/column indices and float64 values; dense
records use native float64 column-major layout. The audited runtime platform is
little-endian x86_64. IMU records include physical start time and dt. Logging
cost and binary size must be reported separately from the default-disabled path.

## LC status and minimum dependencies

| Item | Executed scope | Remaining block and minimum extension |
|---|---|---|
| LC-01 | Seed geometry is unchanged; main agent's native JPL seed test supplies actual finite differences and controlled source mapping. | Real historical pose/bearing/extrinsic/time joint covariance and main/seed cross must be exported before seed total Sigma or `C_x_seed` is valid. Seed marginal diagnostics alone cannot supply these blocks. |
| LC-02 | Actual Euler IMU input derivative; fixed-gain repeated-bearing camera derivative; persistent corrected offset source; full generic correlated-source map. | Raw IMU endpoint IDs/weights and calibration Jacobians; main propagation/visual measurement/nullspace/compression/reset mappings; main/Observer and reused bearing cross; gain-schedule sensitivity including eigenvalue sanitization. |
| LC-03 | Actual lifecycle M and conditional source anchor/current/cross-point blocks recorded throughout live points. Independent permutation and saved-row covariance checks. | Initialize full seed/main/anchor joint covariance; propagate real source correlations through main visual corrections/clone events; connect experimental conditional anchor identity to candidate residual's chosen mature anchor. |
| LC-06 | 1000 IID native-production Observer one-IMU-step draws, seed 20261008, fixed initial seed, corrected inputs, sigma 1e-4; native input FD. Camera finite differences freeze nominal gains. | This is one controlled amplitude and one local linear fragment, not the full frozen LC-06 matrix, not MC of actual gain sensitivity, and not real landmark statistical calibration. |
| LC-07 | Newly instrumented production source can emit every actual IMU map, camera composed map, lifecycle and conditional anchor block. | Full V2_02/V2_03 executions and matched OFF equality remain an integration/runtime check; no run of this module can make the missing unconditional blocks valid. Independent landmark truth is absent. |

Minimum safe conclusion is to preserve shadow and STOP landmark ON. More matrix
logging does not resolve unknown covariance or provide independent landmark truth.
