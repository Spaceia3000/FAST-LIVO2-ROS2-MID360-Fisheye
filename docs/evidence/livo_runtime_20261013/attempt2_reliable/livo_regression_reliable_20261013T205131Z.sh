#!/usr/bin/env bash
set -eo pipefail
source /home/breidy/Documents/fast_livo2_baseline_ws/install/setup.bash
source /home/breidy/Documents/fast_livo2_rtabmap_lc_ws/mvp1_runs/metric_parity_20261013/project_overlay/install/local_setup.bash
source /home/breidy/Documents/fast_livo2_worktrees/imu_validation_20261013/mapping_overlay/install/local_setup.bash
export ROS_DOMAIN_ID=97
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>50</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
python3 -B /home/breidy/Documents/fast_livo2_worktrees/imu_init_time_origin_20261013/tools/validation/run_imu_livo_regression.py --project-source /home/breidy/Documents/fast_livo2_rtabmap_lc_ws/worktrees/metric_parity_20261013 --domain 97 --output /home/breidy/Documents/fast_livo2_worktrees/imu_validation_20261013/livo_regression_reliable_20261013T205131Z 
