#!/usr/bin/env bash
set -eo pipefail
source /home/breidy/Documents/fast_livo2_baseline_ws/install/setup.bash
TASK_ROOT=/home/breidy/Documents/fast_livo2_worktrees/imu_init_time_origin_20261013
TASK_RUN=/home/breidy/Documents/fast_livo2_worktrees/imu_validation_20261013
TASK_EVIDENCE="$TASK_ROOT/docs/evidence/imu_time_origin_20261013"
mkdir -p "$TASK_RUN"
for ARM in before after; do
  if [ "$ARM" = before ]; then TASK_SOURCE=/home/breidy/Documents/fast_livo2_worktrees/imu_init_time_origin_before_20261013; else TASK_SOURCE="$TASK_ROOT"; fi
  mkdir -p "$TASK_RUN/$ARM" "$TASK_SOURCE/Log"
  cmake -S "$TASK_SOURCE" -B "$TASK_RUN/$ARM" -Dvikit_common_DIR=/home/breidy/Documents/ham_slam_ws/install/vikit_common/share/vikit_common/cmake -Dvikit_ros_DIR=/home/breidy/Documents/ham_slam_ws/install/vikit_ros/share/vikit_ros/cmake -DBUILD_TESTING=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON > "$TASK_EVIDENCE/$ARM.configure.log" 2>&1
  cmake --build "$TASK_RUN/$ARM" --target test_imu_initialization_time_origin --parallel 2 > "$TASK_EVIDENCE/$ARM.build.log" 2>&1
  set +e
  "$TASK_RUN/$ARM/test_imu_initialization_time_origin" --gtest_output="xml:$TASK_EVIDENCE/$ARM.junit.xml" > "$TASK_EVIDENCE/$ARM.test.log" 2>&1
  TASK_RC=$?
  set -e
  echo "$TASK_RC" > "$TASK_EVIDENCE/$ARM.returncode.txt"
  echo "$ARM test exit=$TASK_RC"
  if [ "$ARM" = before ] && [ "$TASK_RC" -ne 1 ]; then echo 'Unexpected baseline exit'; exit 2; fi
  if [ "$ARM" = after ] && [ "$TASK_RC" -ne 0 ]; then echo 'Patched test failed'; exit 3; fi
done
