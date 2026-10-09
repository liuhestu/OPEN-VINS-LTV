# UZH-FPV reference and gain scan

Run from this repository with Python 3, NumPy, the ROS Humble underlay, raw UZH
archives and EuRoC ASL inputs. Preparation copies a clean HEAD snapshot, verifies
all estimator dependency sources against the independently built EuRoC artifact,
copies those identical libraries and recompiles the scalar replay drivers. The
existing artifact location comes from `docs/euroc_results_data/identity.json`.
Every preparation/build attempt is retained. Build and replay use an exclusive /
shared lock; replay also acquires one of four global slots.

```bash
OPENBLAS_NUM_THREADS=1 python3 docs/experiments/tools/ltv_gain_scan/test_study.py
OPENBLAS_NUM_THREADS=1 python3 docs/experiments/tools/ltv_gain_scan/study.py prepare /tmp/new_ltv_scan
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 docs/experiments/tools/ltv_gain_scan/study.py run /tmp/new_ltv_scan
OPENBLAS_NUM_THREADS=1 python3 docs/experiments/tools/ltv_gain_scan/audit.py /tmp/new_ltv_scan
OPENBLAS_NUM_THREADS=1 python3 docs/experiments/tools/ltv_gain_scan/study.py publish /tmp/new_ltv_scan
```

`--euroc` and `--uzh` override input roots for preparation. UZH archives must
contain `left_images.txt`, `right_images.txt`, `imu.txt`, `img/` and
`groundtruth.txt`. Camera and IMU seconds convert to integer nanoseconds with
Decimal rounding; every original row is retained. Image symlinks point at the
hashed original files. Full input is never truncated. This is synchronous raw
sensor replay through the estimator, not a ROS transport benchmark.

UZH physical settings come from the corresponding `config/ltv_value/uzhfpv_*`
configuration. Only `ltv_*` settings are inherited from the current EuRoC C0 LTV
block; sensor calibration/noise, initialization and visual tracking remain UZH
specific. Mode switches are set by the existing driver. Every group disables
native SLAM. An enabled G/V/L branch gets its independent 1/2/4 tier; disabled
branches retain their reference weight. No estimator algorithm is edited.

The coordinator first runs seven 40-second serial smoke cases (six reference
modes and L_GV 4×), then repeats them with four isolated workers and requires
byte-identical trajectory, replay counters and effective options. Each worker
has a distinct four-CPU affinity, namespace, ROS domain, registry, task directory,
ROS log and temporary directory. The corrected scalar driver verifies one OpenCV thread after VioManager
construction; BLAS/OpenMP use one thread. The completed original full matrix
actually used four OpenCV threads within each four-CPU affinity, because the
constructor reset the earlier OpenCV setting. This deviation is explicitly
recorded in runtime_threading.json; seven post-fix regression cases have actual
thread count one and byte-identical trajectories/counters. Original timings
are not single-thread performance measurements.
Memory dispatch pauses below 4 GiB available. Every run gets a unique output
directory and process group. No implicit process termination occurs.

Each sequence's full OFF must pass input/effective-mode/finite-output checks and
have valid GT position support before ON can run. The evaluation support is
saved once in `support/*.npz`. EuRoC uses nearest GT within 20 ms, bounded by GT
extent; UZH uses seconds and linear position interpolation, with no extrapolation
or interpolation over gaps larger than 10 ms. ON output must cover every frozen
OFF timestamp within 1 µs and initialize within 1 µs. ATE uses rigid SE(3)
position alignment without scale, independently checked with SVD and Horn.

The five active representative sequences run all five ON modes at 2×, then 4×.
Each mode/tier is screened independently against OFF and its same-mode 1×
reference; a strict increase greater than 10% blocks expansion. Missing/invalid
support or a nonpositive reference denominator blocks screening. Only passing
mode/tier pairs extend to the remaining seven EuRoC and three UZH sequences.
MH_04 and indoor_forward_3 are excluded prospectively. After the user amendment,
indoor_forward_7 is excluded from subsequent gain runs and comparison because
its already completed 16 cases had zero G/V/L applications. The original six-sequence
reference report and these historical runs remain, while the active gain report
contains 15 sequences and 240 configurations. The original six-representative
screening and the amended five-representative screening are both archived. The raw index retains all
256 original independent identities, including explicit unstarted BLOCKED and
unused-reference SKIPPED rows. PASS is only permission to expand, never
engineering acceptance; historical 5% prediction FAIL and engineering STOP
remain unchanged.

`run` can resume completed full identities from its own index and reuses its
saved successful smoke gate. It does not automatically retry failed estimator
cases. Infrastructure fixes may be followed by at most one identical replay
retry, retaining both attempts; no parameter tuning or support trimming is
allowed to rescue a tier. The independent audit rehashes all sensors/images/GT,
checks effective config/binary hashes, recomputes ATE and gate decisions, and
checks identities, skipped expansion and output directory uniqueness. Original
logs and process registries remain under the coordination directory; the report
contains compact JSON/CSV evidence, configuration copies and hashes.

After publishing the completed results, archive lossless estimated timestamps and
positions and recheck all valid runs against the committed frozen support:

```bash
OPENBLAS_NUM_THREADS=1 python3 docs/experiments/tools/ltv_gain_scan/archive.py archive /tmp/new_ltv_scan
OPENBLAS_NUM_THREADS=1 python3 docs/experiments/tools/ltv_gain_scan/study.py publish /tmp/new_ltv_scan
OPENBLAS_NUM_THREADS=1 python3 docs/experiments/tools/ltv_gain_scan/archive.py verify docs/euroc_tune_results
OPENBLAS_NUM_THREADS=1 python3 docs/experiments/tools/ltv_gain_scan/archive.py verify docs/uzhfpv_results_data
```

The `verify` action needs only the repository archives, NumPy and this tool: it
does not read original sensors/GT or the coordination directory. Full estimated
position streams use lossless float64 NPZ compression; the evaluation manifest
also retains hashes of the original trajectory, effective options, replay and
fusion receipts. ATE applies only to matched frozen GT support. UZH GT may cover
a small fraction of complete input; tail diagnostics refer to the final 10% of
matched support and never extrapolate past the available GT.

Published configurations share identical calibration and mask files through
relative `_shared/` symlinks; every tier estimator YAML and file content hash
remains explicit. Raw execution configurations remain immutable and unchanged.
