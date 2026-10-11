#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1 CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1
cd /home/he/output/ltv_integration_cleanup_real_20261005/baseline
colcon --log-base /home/he/output/ltv_integration_cleanup_real_20261005/baseline/log build --base-paths /home/he/output/ltv_integration_cleanup_real_20261005/baseline_f3e055ab --packages-up-to ov_msckf --executor sequential --build-base /home/he/output/ltv_integration_cleanup_real_20261005/baseline/build --install-base /home/he/output/ltv_integration_cleanup_real_20261005/baseline/install --cmake-args -DENABLE_ROS=ON -DBUILD_LTV_PHASE0_TESTS=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DPYTHON_EXECUTABLE=/usr/bin/python3
