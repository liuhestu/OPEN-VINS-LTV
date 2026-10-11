#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/he/open_vins_ltv_ws/install/setup.bash
set -u
cmake -S ov_msckf -B /home/he/output/ltv_passive_hardening_v2/build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DBUILD_LTV_PHASE0_TESTS=ON
