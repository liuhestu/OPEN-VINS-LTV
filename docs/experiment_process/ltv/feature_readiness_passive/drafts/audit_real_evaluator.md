# Independent real evaluator review

Read-only review of `real_evaluate.py`, its referenced logging interfaces and the frozen goal. No confirmation results, GT arrays, replay or numerical experiments were inspected/executed in this review.

## Supported contracts

- EuRoC classification explicitly includes V1_03. Official CSV slices are position 1:4, quaternion xyzw reordered from wxyz columns 4:8, and world velocity 8:11. The reference rotates velocity and negative world gravity into physical body coordinates with the quaternion transpose.
- Native OpenVINS quaternion coefficients are interpreted as Hamilton body-to-world, equivalent to transposing the native JPL world-to-body matrix. Native velocity is then rotated to body coordinates. No fitted global rotation, scale or time shift is introduced.
- UZH velocity uses the same fixed 0.1 s cubic least-squares derivative, nine-sample minimum, full-window boundaries, and 0.010000001 s gap threshold as the historical v3 helper. Unsupported windows stay NaN. Gravity magnitude is 9.81 / 9.8065 according to the prescribed dataset settings.
- The denominator for output readiness is all initialized B camera packets. NEW/OLD are joined by camera nanoseconds; stale/current physical IMU times are checked. Missing/not-ready periods are retained in the curves, not silently cut out of packet coverage.
- OLD/NEW/OpenVINS error summaries on a given support use the same packet mask. Ready sample coverage and reference-supported seconds are separate; final-packet elapsed weight is zero. The 20% improvement and `max(5%, absolute bound)` degradation rules are implemented as specified, including common-raw nondegradation.
- No real landmark correctness fraction is invented. Absence of landmark GT is explicit.

## Required denominator correction

`manager_metrics` currently derives `opportunities` only from logged manager track `ever_opportunity`. This is not the independent denominator required by the goal: raw tracker candidates that exceed the 256-candidate cache or otherwise never obtain a manager-history entry are excluded before the final risk gate. The runner already captures every raw cam0/stereo ID before capacity selection, so derive a parallel opportunity set from that event stream under the frozen minimum history/source rule, and retain the manager set as a diagnostic only. Report never-cached and no-opportunity tracks explicitly. For gaps, apply the same contiguous-history rule; do not sum scattered frames. Be explicit about unstarted epoch-0 packets and resets so arbitrary log defaults do not create fake track births.

## Evidence-strength checks to retain

- `engineering()` should tie metadata `camera_packets` to actual audit/features row counts, and `output_rows` to actual trajectory rows; currently its integrity check compares audit to features but does not tie their common count back to replay metadata. Existing independent `audit/passive_outputs.py` already checks the audit/trajectory counts and effective OFF options and can be reused.
- Every comparison quantity independently excludes unavailable GT. Report OLD/NEW reference sample counts and seconds on the actual improvement support, particularly when eta has GT but UZH differentiated velocity does not. NEW-only coverage seconds must not be mistaken for the OLD/NEW common evaluated seconds.
- Packet-based longest-not-ready gap does not capture a timestamp gap between two ready packets. Report sensor/output packet gaps separately or explicitly label this metric as gaps on the sampled readiness sequence.
- Startup configuration checks and actual receipt-zero counters are complementary. CSV counters alone are insufficient evidence of all effective settings; archive effective options and their hashes with the run evidence.

These are implementation/evidence findings, not effect conclusions about the development or confirmation sequences.
