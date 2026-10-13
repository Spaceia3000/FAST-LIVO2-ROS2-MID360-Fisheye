#!/usr/bin/env bash
# Stage only. Does not activate, replace original build files, or launch FAST.
set -eo pipefail
if [ "$#" -ne 2 ]; then echo "Usage: bash stage_imu_release.sh <full-production-Git-SHA> <passing-regression.json>" >&2; exit 2; fi
TASK_SHA="$1"
TASK_REGRESSION="$2"
test -f "$TASK_REGRESSION"
TASK_WS=/home/breidy/Documents/fast_livo2_baseline_ws
TASK_SOURCE="$TASK_WS/src/FAST-LIVO2-ROS2-MID360-Fisheye"
TASK_RELEASE="$TASK_WS/releases/imu_origin_$TASK_SHA"
test "$(git -C "$TASK_SOURCE" rev-parse HEAD)" = "$TASK_SHA"
git -C "$TASK_SOURCE" diff --quiet
git -C "$TASK_SOURCE" diff --cached --quiet
test ! -e "$TASK_RELEASE"
mkdir -p "$TASK_RELEASE"
source "$TASK_WS/install/setup.bash"
export CMAKE_BUILD_PARALLEL_LEVEL=2
export MAKEFLAGS=-j2
colcon --log-base "$TASK_RELEASE/log" build --base-paths "$TASK_SOURCE" --packages-select fast_livo --symlink-install --build-base "$TASK_RELEASE/build" --install-base "$TASK_RELEASE/install" --executor sequential --cmake-args -DBUILD_TESTING=OFF -Dvikit_common_DIR=/home/breidy/Documents/ham_slam_ws/install/vikit_common/share/vikit_common/cmake -Dvikit_ros_DIR=/home/breidy/Documents/ham_slam_ws/install/vikit_ros/share/vikit_ros/cmake > "$TASK_RELEASE/build.log" 2>&1
test "$(git -C "$TASK_SOURCE" rev-parse HEAD)" = "$TASK_SHA"
git -C "$TASK_SOURCE" diff --quiet
git -C "$TASK_SOURCE" diff --cached --quiet
python3 "$TASK_SOURCE/tools/validation/activate_imu_release.py" --stage "$TASK_RELEASE" --expected-source "$TASK_SHA" --runtime-evidence "$TASK_REGRESSION"
