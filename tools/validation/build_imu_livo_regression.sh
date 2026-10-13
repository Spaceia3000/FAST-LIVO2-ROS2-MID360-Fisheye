#!/usr/bin/env bash
set -eo pipefail
source /home/breidy/Documents/fast_livo2_baseline_ws/install/setup.bash
TASK_SOURCE=/home/breidy/Documents/ham_slam_ws/validation_sources/FAST-LIVO2-imu-fix
TASK_OVERLAY=/home/breidy/Documents/fast_livo2_worktrees/imu_validation_20261013/mapping_overlay
mkdir -p "$TASK_SOURCE/Log" "$TASK_OVERLAY"
export CMAKE_BUILD_PARALLEL_LEVEL=2
export MAKEFLAGS=-j2
colcon --log-base "$TASK_OVERLAY/log" build --base-paths "$TASK_SOURCE" --packages-select fast_livo --symlink-install --build-base "$TASK_OVERLAY/build" --install-base "$TASK_OVERLAY/install" --executor sequential --cmake-args -DBUILD_TESTING=OFF -Dvikit_common_DIR=/home/breidy/Documents/ham_slam_ws/install/vikit_common/share/vikit_common/cmake -Dvikit_ros_DIR=/home/breidy/Documents/ham_slam_ws/install/vikit_ros/share/vikit_ros/cmake > "$TASK_OVERLAY/build.log" 2>&1
