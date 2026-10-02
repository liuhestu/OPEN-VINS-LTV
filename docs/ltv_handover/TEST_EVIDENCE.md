# 测试证据（本次执行与历史分开）

本次独立临时目录编译，不触碰workspace build/install。`validation/results.json`记录完整argv/cwd/退出码/日志；原测试源码不修改，CMake注册位置`vins/CMakeLists.txt:87`至test块末尾。33个C++测试合并为一个独立gtest binary执行；这些是本次源代码重编译结果，不是旧colcon产物。59个Python测试只运行小unittest，未调用runner主程序启动数据集。

构建命令在results中分别为compile_observer/compile_fixture/compile_existing_cpp_tests，flags C++14/O1/DNDEBUG/Wextra/Wpedantic；每条构建退出0、stderr为空。没有完整ROS/ament包build、完整ctest、启动节点、topic/RViz或EuRoC/KITTI/RealSense重放。本任务为只读审计，测试范围不等于全系统验收。

## 33个已有C++ LTV测试（每条均本次执行）

每条命令均为results中的`existing_cpp_tests` argv（带`--gtest_output=xml:...`），进程退出0，产物`validation/existing_cpp_tests.xml`、对应stdout/stderr。名称描述就是覆盖契约，不额外宣称理论保证。

| 测试名 | 原路径:line | 覆盖内容 | 本次执行/退出码 |
|---|---|---|---|
| `LtvGravityFactor.MatchingDirectionsHaveZeroResidual` | `vins/test/test_ltv_gravity_factor.cpp:64` | MatchingDirectionsHaveZeroResidual（完整源码已读） | 是 / 0 / XML case记录 |
| `LtvGravityFactor.ResidualIgnoresGravityMagnitudeAndPosition` | `vins/test/test_ltv_gravity_factor.cpp:74` | ResidualIgnoresGravityMagnitudeAndPosition（完整源码已读） | 是 / 0 / XML case记录 |
| `LtvGravityFactor.OppositeDirectionsHaveExpectedNorm` | `vins/test/test_ltv_gravity_factor.cpp:85` | OppositeDirectionsHaveExpectedNorm（完整源码已读） | 是 / 0 / XML case记录 |
| `LtvGravityFactor.AnalyticJacobianMatchesRightPerturbation` | `vins/test/test_ltv_gravity_factor.cpp:92` | AnalyticJacobianMatchesRightPerturbation（完整源码已读） | 是 / 0 / XML case记录 |
| `LtvGravityFactor.RejectsInvalidConstructionInput` | `vins/test/test_ltv_gravity_factor.cpp:110` | RejectsInvalidConstructionInput（完整源码已读） | 是 / 0 / XML case记录 |
| `LtvProjection.IsSymmetricIdempotentAndOrthogonal` | `vins/test/test_ltv_observer.cpp:37` | IsSymmetricIdempotentAndOrthogonal（完整源码已读） | 是 / 0 / XML case记录 |
| `LtvObserver.StateDimensionsFollowFeatureCount` | `vins/test/test_ltv_observer.cpp:50` | StateDimensionsFollowFeatureCount（完整源码已读） | 是 / 0 / XML case记录 |
| `LtvObserver.ImuPropagationUsesBodyFrameApparentAcceleration` | `vins/test/test_ltv_observer.cpp:67` | ImuPropagationUsesBodyFrameApparentAcceleration（完整源码已读） | 是 / 0 / XML case记录 |
| `LtvObserver.FeatureCompactionPreservesIdentity` | `vins/test/test_ltv_observer.cpp:87` | FeatureCompactionPreservesIdentity（完整源码已读） | 是 / 0 / XML case记录 |
| `LtvObserver.TimestampBackwardResetsObserver` | `vins/test/test_ltv_observer.cpp:113` | TimestampBackwardResetsObserver（完整源码已读） | 是 / 0 / XML case记录 |
| `LtvObserver.AdaptiveCameraSubstepsPreserveProductionScaleCovariance` | `vins/test/test_ltv_observer.cpp:130` | AdaptiveCameraSubstepsPreserveProductionScaleCovariance（完整源码已读） | 是 / 0 / XML case记录 |
| `LtvCsvLogger.DisabledLoggerDoesNotOpenFile` | `vins/test/test_ltv_observer.cpp:166` | DisabledLoggerDoesNotOpenFile（完整源码已读） | 是 / 0 / XML case记录 |
| `LtvGravityQualityGate.AcceptsHealthySnapshot` | `vins/test/test_ltv_quality_gate.cpp:27` | AcceptsHealthySnapshot（完整源码已读） | 是 / 0 / XML case记录 |
| `LtvGravityQualityGate.ReportsEveryRejectedCondition` | `vins/test/test_ltv_quality_gate.cpp:36` | ReportsEveryRejectedCondition（完整源码已读） | 是 / 0 / XML case记录 |
| `LtvGravityQualityGate.DisabledInnovationThresholdAlwaysPassesInnovation` | `vins/test/test_ltv_quality_gate.cpp:55` | DisabledInnovationThresholdAlwaysPassesInnovation（完整源码已读） | 是 / 0 / XML case记录 |
| `LtvSnapshotWindow.StartsAndClearsInvalid` | `vins/test/test_ltv_snapshot.cpp:28` | StartsAndClearsInvalid（完整源码已读） | 是 / 0 / XML case记录 |
| `LtvSnapshotWindow.MarginOldMatchesEstimatorPermutation` | `vins/test/test_ltv_snapshot.cpp:47` | MarginOldMatchesEstimatorPermutation（完整源码已读） | 是 / 0 / XML case记录 |
| `LtvSnapshotWindow.MarginSecondNewestMatchesEstimatorPermutation` | `vins/test/test_ltv_snapshot.cpp:60` | MarginSecondNewestMatchesEstimatorPermutation（完整源码已读） | 是 / 0 / XML case记录 |
| `LtvVelocityFactor.MatchingVelocityHasZeroResidual` | `vins/test/test_ltv_velocity_factor.cpp:92` | MatchingVelocityHasZeroResidual（完整源码已读） | 是 / 0 / XML case记录 |
| `LtvVelocityFactor.RotationTransformsWorldVelocityIntoBodyFrame` | `vins/test/test_ltv_velocity_factor.cpp:103` | RotationTransformsWorldVelocityIntoBodyFrame（完整源码已读） | 是 / 0 / XML case记录 |
| `LtvVelocityFactor.ResidualIgnoresPositionAndBiases` | `vins/test/test_ltv_velocity_factor.cpp:114` | ResidualIgnoresPositionAndBiases（完整源码已读） | 是 / 0 / XML case记录 |
| `LtvVelocityFactor.AnalyticJacobiansMatchEffectiveLocalJacobians` | `vins/test/test_ltv_velocity_factor.cpp:132` | AnalyticJacobiansMatchEffectiveLocalJacobians（完整源码已读） | 是 / 0 / XML case记录 |
| `LtvVelocityFactor.RejectsInvalidInput` | `vins/test/test_ltv_velocity_factor.cpp:153` | RejectsInvalidInput（完整源码已读） | 是 / 0 / XML case记录 |
| `LtvVelocityOracleGate.DefaultsAreDisabled` | `vins/test/test_ltv_velocity_oracle_gate.cpp:26` | DefaultsAreDisabled（完整源码已读） | 是 / 0 / XML case记录 |
| `LtvVelocityOracleGate.DisabledGatePassesThroughWithoutLoadingMask` | `vins/test/test_ltv_velocity_oracle_gate.cpp:36` | DisabledGatePassesThroughWithoutLoadingMask（完整源码已读） | 是 / 0 / XML case记录 |
| `LtvVelocityOracleGate.UsesSelectedFrozenDecisionColumn` | `vins/test/test_ltv_velocity_oracle_gate.cpp:46` | UsesSelectedFrozenDecisionColumn（完整源码已读） | 是 / 0 / XML case记录 |
| `LtvVelocityOracleGate.RequiresExactTimestampAndBaseEligibility` | `vins/test/test_ltv_velocity_oracle_gate.cpp:69` | RequiresExactTimestampAndBaseEligibility（完整源码已读） | 是 / 0 / XML case记录 |
| `LtvVelocityOracleGate.MissingMalformedAndInvalidColumnFailClosed` | `vins/test/test_ltv_velocity_oracle_gate.cpp:87` | MissingMalformedAndInvalidColumnFailClosed（完整源码已读） | 是 / 0 / XML case记录 |
| `LtvVelocityQualityGate.InclusiveThresholdsPass` | `vins/test/test_ltv_velocity_quality_gate.cpp:32` | InclusiveThresholdsPass（完整源码已读） | 是 / 0 / XML case记录 |
| `LtvVelocityQualityGate.ReportsIndependentAndCombinedRejections` | `vins/test/test_ltv_velocity_quality_gate.cpp:47` | ReportsIndependentAndCombinedRejections（完整源码已读） | 是 / 0 / XML case记录 |
| `LtvVelocityQualityGate.NanAndInfinityFailClosed` | `vins/test/test_ltv_velocity_quality_gate.cpp:84` | NanAndInfinityFailClosed（完整源码已读） | 是 / 0 / XML case记录 |
| `LtvVelocityQualityGate.ResetRejectsExactlyTenFollowingHealthyFrames` | `vins/test/test_ltv_velocity_quality_gate.cpp:101` | ResetRejectsExactlyTenFollowingHealthyFrames（完整源码已读） | 是 / 0 / XML case记录 |
| `LtvVelocityQualityGate.ConflictAndInvalidConfigurationFailClosed` | `vins/test/test_ltv_velocity_quality_gate.cpp:126` | ConflictAndInvalidConfigurationFailClosed（完整源码已读） | 是 / 0 / XML case记录 |

## 已有Python小测试（59项，均本次执行）

每个模块实际命令为`python3 -B /absolute/repo/vins/test/<module>.py -v`，cwd=独立/tmp build，`PYTHONDONTWRITEBYTECODE=1`；完整绝对路径/cwd见results。每个方法的OK及模块计数在`validation/<module>.stderr.log`。这些测试中合成GT仅用于已有evaluator/gate工具测试，**observer golden没有使用GT**。

| 模块/方法 | 原路径:line | 覆盖契约 | 执行/退出码/产物 |
|---|---|---|---|
| `test_stage7_tuning.test_override_and_old_default` | `vins/test/test_stage7_tuning.py:22` | test_override_and_old_default | 是 / 0 / `test_stage7_tuning.stderr.log` |
| `test_stage7_tuning.test_cache_identity_paths_and_binary` | `vins/test/test_stage7_tuning.py:35` | test_cache_identity_paths_and_binary | 是 / 0 / `test_stage7_tuning.stderr.log` |
| `test_stage7_tuning.test_budget_counts_failures_reserves_and_hard_limit` | `vins/test/test_stage7_tuning.py:46` | test_budget_counts_failures_reserves_and_hard_limit | 是 / 0 / `test_stage7_tuning.stderr.log` |
| `test_stage7_tuning.test_degenerate_gates_keep_bypass_checks` | `vins/test/test_stage7_tuning.py:64` | test_degenerate_gates_keep_bypass_checks | 是 / 0 / `test_stage7_tuning.stderr.log` |
| `test_stage7_tuning.test_missing_frames_rejected_and_support_exact` | `vins/test/test_stage7_tuning.py:82` | test_missing_frames_rejected_and_support_exact | 是 / 0 / `test_stage7_tuning.stderr.log` |
| `test_stage7_tuning.test_missing_hard_frame_recomputes_baseline_too` | `vins/test/test_stage7_tuning.py:94` | test_missing_hard_frame_recomputes_baseline_too | 是 / 0 / `test_stage7_tuning.stderr.log` |
| `test_stage7_tuning.test_successful_skip_and_interrupted_attempt` | `vins/test/test_stage7_tuning.py:111` | test_successful_skip_and_interrupted_attempt | 是 / 0 / `test_stage7_tuning.stderr.log` |
| `test_stage7_tuning.test_group_delta_is_ratio_of_means` | `vins/test/test_stage7_tuning.py:140` | test_group_delta_is_ratio_of_means | 是 / 0 / `test_stage7_tuning.stderr.log` |
| `test_stage7_tuning.test_metrics_match_old_evaluator_and_tails` | `vins/test/test_stage7_tuning.py:146` | test_metrics_match_old_evaluator_and_tails | 是 / 0 / `test_stage7_tuning.stderr.log` |
| `test_run_stage6_joint.test_effective_config_covers_all_modes_and_frozen_parameters` | `vins/test/test_run_stage6_joint.py:49` | test_effective_config_covers_all_modes_and_frozen_parameters | 是 / 0 / `test_run_stage6_joint.stderr.log` |
| `test_run_stage6_joint.test_joint_diagnostics_require_gate_and_fixed_velocity` | `vins/test/test_run_stage6_joint.py:99` | test_joint_diagnostics_require_gate_and_fixed_velocity | 是 / 0 / `test_run_stage6_joint.stderr.log` |
| `test_run_stage6_joint.test_joint_v_gate_diagnostics_require_all_conditions` | `vins/test/test_run_stage6_joint.py:115` | test_joint_v_gate_diagnostics_require_all_conditions | 是 / 0 / `test_run_stage6_joint.stderr.log` |
| `test_run_stage6_joint.test_joint_rejects_gate_bypass_and_oracle_use` | `vins/test/test_run_stage6_joint.py:142` | test_joint_rejects_gate_bypass_and_oracle_use | 是 / 0 / `test_run_stage6_joint.stderr.log` |
| `test_run_stage6_joint.test_joint_v_gate_rejects_velocity_gate_bypass` | `vins/test/test_run_stage6_joint.py:159` | test_joint_v_gate_rejects_velocity_gate_bypass | 是 / 0 / `test_run_stage6_joint.stderr.log` |
| `test_run_stage6_joint.test_gravity_only_requires_gated_gravity_and_no_velocity` | `vins/test/test_run_stage6_joint.py:171` | test_gravity_only_requires_gated_gravity_and_no_velocity | 是 / 0 / `test_run_stage6_joint.stderr.log` |
| `test_run_stage6_joint.test_velocity_only_requires_fixed_velocity_and_no_gravity` | `vins/test/test_run_stage6_joint.py:188` | test_velocity_only_requires_fixed_velocity_and_no_gravity | 是 / 0 / `test_run_stage6_joint.stderr.log` |
| `test_run_stage6_joint.test_transaction_publish_and_failure_preservation` | `vins/test/test_run_stage6_joint.py:208` | test_transaction_publish_and_failure_preservation | 是 / 0 / `test_run_stage6_joint.stderr.log` |
| `test_run_stage6_joint.test_baseline_sha_must_match_stage6` | `vins/test/test_run_stage6_joint.py:228` | test_baseline_sha_must_match_stage6 | 是 / 0 / `test_run_stage6_joint.stderr.log` |
| `test_verify_snapshot_timestamps.test_reports_equal_and_divergent_grids_without_rejecting_divergence` | `vins/test/test_verify_snapshot_timestamps.py:32` | test_reports_equal_and_divergent_grids_without_rejecting_divergence | 是 / 0 / `test_verify_snapshot_timestamps.stderr.log` |
| `test_verify_snapshot_timestamps.test_rejects_non_monotonic_grid` | `vins/test/test_verify_snapshot_timestamps.py:41` | test_rejects_non_monotonic_grid | 是 / 0 / `test_verify_snapshot_timestamps.stderr.log` |
| `test_build_stage5_oracle_mask.test_world_to_body_rotation` | `vins/test/test_build_stage5_oracle_mask.py:19` | test_world_to_body_rotation | 是 / 0 / `test_build_stage5_oracle_mask.stderr.log` |
| `test_build_stage5_oracle_mask.test_builds_strict_zero_and_point_zero_two_masks` | `vins/test/test_build_stage5_oracle_mask.py:26` | test_builds_strict_zero_and_point_zero_two_masks | 是 / 0 / `test_build_stage5_oracle_mask.stderr.log` |
| `test_build_stage5_oracle_mask.test_gt_tolerance_and_ineligible_rows_fail_closed` | `vins/test/test_build_stage5_oracle_mask.py:46` | test_gt_tolerance_and_ineligible_rows_fail_closed | 是 / 0 / `test_build_stage5_oracle_mask.stderr.log` |
| `test_stage5_cache.test_imu_monotonicity_fails_closed` | `vins/test/test_stage5_cache.py:23` | test_imu_monotonicity_fails_closed | 是 / 0 / `test_stage5_cache.stderr.log` |
| `test_stage5_cache.test_sha256_and_csv_are_deterministic` | `vins/test/test_stage5_cache.py:27` | test_sha256_and_csv_are_deterministic | 是 / 0 / `test_stage5_cache.stderr.log` |
| `test_stage5_cache.test_imu_bracketing_drops_only_unsupported_boundaries` | `vins/test/test_stage5_cache.py:35` | test_imu_bracketing_drops_only_unsupported_boundaries | 是 / 0 / `test_stage5_cache.stderr.log` |
| `test_stage5_cache.test_imu_bracketing_margin_covers_time_shift` | `vins/test/test_stage5_cache.py:44` | test_imu_bracketing_margin_covers_time_shift | 是 / 0 / `test_stage5_cache.stderr.log` |
| `test_evaluate_euroc_final.test_one_to_one_matching_is_inclusive_and_does_not_reuse` | `vins/test/test_evaluate_euroc_final.py:35` | test_one_to_one_matching_is_inclusive_and_does_not_reuse | 是 / 0 / `test_evaluate_euroc_final.stderr.log` |
| `test_evaluate_euroc_final.test_common_support_accepts_exactly_99_percent` | `vins/test/test_evaluate_euroc_final.py:42` | test_common_support_accepts_exactly_99_percent | 是 / 0 / `test_evaluate_euroc_final.stderr.log` |
| `test_evaluate_euroc_final.test_common_support_rejects_below_99_percent` | `vins/test/test_evaluate_euroc_final.py:52` | test_common_support_rejects_below_99_percent | 是 / 0 / `test_evaluate_euroc_final.stderr.log` |
| `test_evaluate_euroc_final.test_metrics_use_independent_ate_and_common_baseline_gauge` | `vins/test/test_evaluate_euroc_final.py:61` | test_metrics_use_independent_ate_and_common_baseline_gauge | 是 / 0 / `test_evaluate_euroc_final.stderr.log` |
| `test_evaluate_euroc_final.test_markdown_contains_only_three_result_tables` | `vins/test/test_evaluate_euroc_final.py:103` | test_markdown_contains_only_three_result_tables | 是 / 0 / `test_evaluate_euroc_final.stderr.log` |
| `test_run_euroc_final_experiment.test_batch_manifest_freezes_all_modes_and_hashes` | `vins/test/test_run_euroc_final_experiment.py:23` | test_batch_manifest_freezes_all_modes_and_hashes | 是 / 0 / `test_run_euroc_final_experiment.stderr.log` |
| `test_run_euroc_final_experiment.test_expected_pair_consumption_counts_five_modes` | `vins/test/test_run_euroc_final_experiment.py:41` | test_expected_pair_consumption_counts_five_modes | 是 / 0 / `test_run_euroc_final_experiment.stderr.log` |
| `test_run_euroc_final_experiment.test_failed_batch_is_preserved_with_evidence` | `vins/test/test_run_euroc_final_experiment.py:54` | test_failed_batch_is_preserved_with_evidence | 是 / 0 / `test_run_euroc_final_experiment.stderr.log` |
| `test_audit_stage6b_common_alignment.test_orientation_metrics_apply_supplied_common_alignment` | `vins/test/test_audit_stage6b_common_alignment.py:30` | test_orientation_metrics_apply_supplied_common_alignment | 是 / 0 / `test_audit_stage6b_common_alignment.stderr.log` |
| `test_audit_stage6b_common_alignment.test_common_alignment_exposes_candidate_gauge_difference` | `vins/test/test_audit_stage6b_common_alignment.py:40` | test_common_alignment_exposes_candidate_gauge_difference | 是 / 0 / `test_audit_stage6b_common_alignment.stderr.log` |
| `test_audit_stage6b_common_alignment.test_alignment_delta_reports_angle_and_rpy` | `vins/test/test_audit_stage6b_common_alignment.py:54` | test_alignment_delta_reports_angle_and_rpy | 是 / 0 / `test_audit_stage6b_common_alignment.stderr.log` |
| `test_audit_stage6b_common_alignment.test_relative_change_fails_closed_for_zero_or_nonfinite_input` | `vins/test/test_audit_stage6b_common_alignment.py:60` | test_relative_change_fails_closed_for_zero_or_nonfinite_input | 是 / 0 / `test_audit_stage6b_common_alignment.stderr.log` |
| `test_diagnose_stage6c_mh01.test_pitch_error_sign_uses_established_rpy_convention` | `vins/test/test_diagnose_stage6c_mh01.py:22` | test_pitch_error_sign_uses_established_rpy_convention | 是 / 0 / `test_diagnose_stage6c_mh01.stderr.log` |
| `test_diagnose_stage6c_mh01.test_trailing_rmse_and_sustained_onset_use_inclusive_boundaries` | `vins/test/test_diagnose_stage6c_mh01.py:30` | test_trailing_rmse_and_sustained_onset_use_inclusive_boundaries | 是 / 0 / `test_diagnose_stage6c_mh01.stderr.log` |
| `test_diagnose_stage6c_mh01.test_gate_segments_count_frames_and_duration` | `vins/test/test_diagnose_stage6c_mh01.py:44` | test_gate_segments_count_frames_and_duration | 是 / 0 / `test_diagnose_stage6c_mh01.stderr.log` |
| `test_diagnose_stage6c_mh01.test_post_optimization_velocity_residual_uses_body_frame` | `vins/test/test_diagnose_stage6c_mh01.py:54` | test_post_optimization_velocity_residual_uses_body_frame | 是 / 0 / `test_diagnose_stage6c_mh01.stderr.log` |
| `test_diagnose_stage6c_mh01.test_imu_aggregation_uses_each_camera_interval` | `vins/test/test_diagnose_stage6c_mh01.py:63` | test_imu_aggregation_uses_each_camera_interval | 是 / 0 / `test_diagnose_stage6c_mh01.stderr.log` |
| `test_diagnose_stage6c_mh01.test_imu_aggregation_fails_closed_on_empty_interval` | `vins/test/test_diagnose_stage6c_mh01.py:82` | test_imu_aggregation_fails_closed_on_empty_interval | 是 / 0 / `test_diagnose_stage6c_mh01.stderr.log` |
| `test_analyze_euroc_asl_velocity.test_constant_velocity_statistics_and_position_check` | `vins/test/test_analyze_euroc_asl_velocity.py:38` | test_constant_velocity_statistics_and_position_check | 是 / 0 / `test_analyze_euroc_asl_velocity.stderr.log` |
| `test_analyze_euroc_asl_velocity.test_body_velocity_uses_transpose_of_world_body_rotation` | `vins/test/test_analyze_euroc_asl_velocity.py:57` | test_body_velocity_uses_transpose_of_world_body_rotation | 是 / 0 / `test_analyze_euroc_asl_velocity.stderr.log` |
| `test_analyze_euroc_asl_velocity.test_time_series_csv_contains_official_and_body_velocity` | `vins/test/test_analyze_euroc_asl_velocity.py:70` | test_time_series_csv_contains_official_and_body_velocity | 是 / 0 / `test_analyze_euroc_asl_velocity.stderr.log` |
| `test_evaluate_vins_euroc.test_uzh_eight_column_reader` | `vins/test/test_evaluate_vins_euroc.py:33` | test_uzh_eight_column_reader | 是 / 0 / `test_evaluate_vins_euroc.stderr.log` |
| `test_evaluate_vins_euroc.test_uzh_reader_rejects_wrong_field_count` | `vins/test/test_evaluate_vins_euroc.py:47` | test_uzh_reader_rejects_wrong_field_count | 是 / 0 / `test_evaluate_vins_euroc.stderr.log` |
| `test_evaluate_vins_euroc.test_uzh_reader_rejects_nan` | `vins/test/test_evaluate_vins_euroc.py:55` | test_uzh_reader_rejects_nan | 是 / 0 / `test_evaluate_vins_euroc.stderr.log` |
| `test_evaluate_vins_euroc.test_uzh_reader_rejects_non_increasing_time` | `vins/test/test_evaluate_vins_euroc.py:63` | test_uzh_reader_rejects_non_increasing_time | 是 / 0 / `test_evaluate_vins_euroc.stderr.log` |
| `test_evaluate_vins_euroc.test_uzh_reader_rejects_zero_quaternion` | `vins/test/test_evaluate_vins_euroc.py:74` | test_uzh_reader_rejects_zero_quaternion | 是 / 0 / `test_evaluate_vins_euroc.stderr.log` |
| `test_evaluate_vins_euroc.test_uzh_reader_rejects_non_unit_quaternion` | `vins/test/test_evaluate_vins_euroc.py:82` | test_uzh_reader_rejects_non_unit_quaternion | 是 / 0 / `test_evaluate_vins_euroc.stderr.log` |
| `test_evaluate_vins_euroc.test_uzh_reader_rejects_no_bag_overlap` | `vins/test/test_evaluate_vins_euroc.py:90` | test_uzh_reader_rejects_no_bag_overlap | 是 / 0 / `test_evaluate_vins_euroc.stderr.log` |
| `test_evaluate_vins_euroc.test_json_output_includes_position_and_rotation_p95` | `vins/test/test_evaluate_vins_euroc.py:98` | test_json_output_includes_position_and_rotation_p95 | 是 / 0 / `test_evaluate_vins_euroc.stderr.log` |
| `test_evaluate_vins_euroc.test_official_csv_reads_velocity_by_header_name` | `vins/test/test_evaluate_vins_euroc.py:132` | test_official_csv_reads_velocity_by_header_name | 是 / 0 / `test_evaluate_vins_euroc.stderr.log` |
| `test_evaluate_vins_euroc.test_position_difference_is_fallback_for_constant_velocity` | `vins/test/test_evaluate_vins_euroc.py:151` | test_position_difference_is_fallback_for_constant_velocity | 是 / 0 / `test_evaluate_vins_euroc.stderr.log` |
| `test_evaluate_vins_euroc.test_alignment_rotation_changes_velocity_but_not_translation` | `vins/test/test_evaluate_vins_euroc.py:163` | test_alignment_rotation_changes_velocity_but_not_translation | 是 / 0 / `test_evaluate_vins_euroc.stderr.log` |

## 已有但本次没有执行的其他测试

| 模块 | 内容/本次未执行原因 | 注册/状态 |
|---|---|---|
| test_uzhfpv_config.py | UZH标定、mask与launch分类；目标是EuRoC源核心交接，未启动ROS/launch依赖 | CMake注册；未执行，退出码/本次产物无 |
| test_audit_uzhfpv_time_offsets.py | UZH时间补偿/信号相关；非源observer递推或T2配置 | CMake注册；完整源码已读，未执行，退出码/本次产物无 |
| test_feature_masks.cpp | VINS frontend mask，需vins_lib/ROS/OpenCV | CMake注册；非LTV核心，本次未执行 |
| test_stereo_synchronizer.cpp | 双目pair分类，不是LTV observer | CMake注册；本次未执行 |

## 新建独立验证产物

| 验证 | 命令标识/退出码 | 实际产物/结论 |
|---|---|---|
| 初版源observer fixture | generate_fixture + repeat_fixture / 0 | 初版168事件，两次实际输出完全一致；旧stdout留在validation，最终golden以后续174事件为准 |
| 最终source_core副本fixture | compile_core_copy/compile_final_fixture/compile_copy_fixture/generate_final_fixture/verify_copy_fixture/compare_core_copy / 0 | 174事件源与副本的完整state/P、slot/reset/substeps、重建matrices逐字节相同；最终golden及manifest |
| neutral compaction | supplemental Python full old_index mapping / 0 | validation/compaction_check.json；非零交叉项全部映射，最大保留P误差2.00e-11 |
| Eigen alias probe | compile_alias_probe + alias_probe / 0；另以同flags编译最终扩大probe | 非对称P对称化最大差9；old-state/P传播和P校正差0。退出0表示探针执行成功和乘法旧值检查通过，**不表示alias表达式安全** |
| SHA核验/参数/链接审查 | 最终audit自检 | SOURCE_MANIFEST源/副本SHA和validation/package_verification.json；只证明审计范围一致 |

## 旧实验和没有证据的内容

`docs/stage_test_result.md`、notune、tuned中的通过/失败为历史记录。本次只读实际manifest/effective YAML/报告与现有binary SHA，未重新运行这些轨迹，不能把旧记录写为本次结果。测试覆盖projection、维度、单步bias、基本compaction、timestamp backward、production量级P、disabled logger、factor局部Jacobian、gate/window/oracle；没有自动证明真实PE、动态集合稳定性、所有传感器边界、核心reset通知与cooldown完整性、目标OpenVINS H/FEJ、独立测量R或ATE收益。
