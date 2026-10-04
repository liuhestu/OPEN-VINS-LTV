#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/he/open_vins_ltv_ws/install/setup.bash
set -u
cmake -S /home/he/open_vins_ltv_ws/src/open_vins_ltv/ov_msckf -B /home/he/output/ltv_active_landmark_consistency_r4/build -DCMAKE_BUILD_TYPE=Release -DBUILD_LTV_PHASE0_TESTS=ON
cmake --build /home/he/output/ltv_active_landmark_consistency_r4/build --target run_ltv_feature_passive replay_ltv_feature_cache test_ltv_active_consistency test_ltv_active_lifecycle test_ltv_feature_pipeline test_ltv_manager_bounded test_ltv_adapter_hardened test_ltv_controlled test_ltv_readiness test_ltv_history_competition -j1
