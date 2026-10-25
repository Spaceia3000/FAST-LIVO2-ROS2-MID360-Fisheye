# Cohemos LIVO camera-gap — targeted validation (2026-10-20)

## Provenance and scope
- Baseline (untouched): `Cohemos @ b948df3` in `~/Documents/fast_livo2_baseline_ws/src/FAST-LIVO2-ROS2-MID360-Fisheye`.
- Experiment: `experiment/cohemos-livo-camera-gap-20261020` in `~/Documents/fast_livo2_worktrees/cohemos_camera_gap_20261020`.
- Input (read-only): `/media/breidy/Robotics/DATA_SET_SPARK/FAST_LIVO2_SLAM_SPARK_SYNCED_20261019/rosbag2_1970_01_01-01_04_33`. Bag length 292.95 sensor seconds; 2,915 LiDAR / 58,307 Livox IMU / 12,065 Sony images.
- Tested segment: start offset 40 s, rate 0.5x, clock 40 Hz; limited replay; `--read-ahead-queue-size 1000`, three sensor topics, no RViz; ROS 2 Humble / Cyclone DDS. Experiment versions were recorded with distinct logs; neither version was promoted.
- Historical source baseline at 0.2x (different run, not directly numerically comparable): `~/Documents/robotics_research_archive/cohemo_ingest_ab_20261019`: camera absent, peak IMU queue 421, LiDAR queue 19, estimated sensor lag max 1.869 s; max gap Sony 0.973 s.

## Root cause and targeted change
- Original `LIVMapper::sync_packages` returned false whenever `img_buffer.empty()`, even with LiDAR+IMU data available; this blocked LIO and grew input buffers.
- Experimental `runtime.livo_camera_wait_sensor_s: 0.30` enables fallback at a sensor-time watermark, using a LiDAR partition endpoint only if corrected IMU time covers it. Default is `0` to preserve original behavior if not opted in.
- Image-targeted LIO and fallback LIO are distinguished: preserve the paired image at exactly the LIO timestamp for the next VIO call; otherwise discard only truly stale images. Residual point partitions and frame provenance are carried forward; no changes to calibration, camera intrinsics, image admission, IMU offsets, or point filter.
- Avoid `wall_stamp_age_s` for 1970 bags: it uses current system epoch and is not a meaningful end-to-end delay; compare LiDAR header stamp with estimator stamp (sensor-time lag).

## A/B result on same segment (1 s wall telemetry samples)
| Metric | A: initial camera fallback (buggy pairing) | B: corrected LIO-VIO pairing |
|---|---:|---:|
| valid samples | 63 | 62 |
| timestamp range of LiDAR sample | 313.676–344.576 | 313.576–344.376 |
| sensor-time lag P50 / P95 / max (s) | 0.047 / 0.300 / 0.687 | 0.108 / 0.300 / 0.400 |
| peak queued IMU / LiDAR / camera messages | 126 / 7 / 1 | 111 / 3 / 1 |
| admitted camera frames | 117 | 113 |
| stale images rejected | 60 (51.3%) | 2 (1.8%) |
| fallback LIO cuts | 45 | 67 |
| fatal/ERROR lines in captured mapping log | 0 | 0 |

The higher median lag and number of fallback cuts in B are trade-offs requiring further inspection. These results demonstrate repair of false camera rejection and reduced maximum queue/lag *on this segment*, not absolute trajectory accuracy or long-run/Jetson realtime guarantees.

VIO `Raw feature num` log events: A 54; B 126 (whole respective short replays, which differ by a few sensor seconds). Version B `Log/mat_out.txt`: 318 pose records; 192 unique time stamps; zero non-finite records and zero time regressions; maximum positive update gap 0.4912 s. No observed `imu loop back`, `lidar loop back`, or `Reject non-monotonic` errors.

## Directed validation
- C++ policy: 4 GoogleTests PASS.
- `colcon test-result --verbose`: 5 tests, 0 errors, 0 failures, 0 skipped.
- Isolated build of `fast_livo` successful; v2 results were obtained from `/tmp/cohemos_camera_gap_build_copy` with installed vikit_common/vikit_ros and Livox overlay.
- Final source cleanup: return IMU_Processing.cpp to upstream Cohemos unchanged; policy is enforced by the synchronizer before deque consumption. Rebuild and 5 test results are recorded in `/tmp/cohemos_gap_final_build.log` and `/tmp/cohemos_gap_final_test.log`.
- Run A logs: `/tmp/cohemos_gap_replay_mapping.log`, `/tmp/cohemos_gap_replay_player_1000.log`.
- Run B logs: `/tmp/cohemos_gap_v2_mapping.log`, `/tmp/cohemos_gap_v2_player.log`.
- All logs above remain on laptop; they are temporary machine paths (preserve before cleaning /tmp). No merge/push performed.

## Operator usage (after building this branch in a proper ROS 2 workspace)
```bash
source /opt/ros/humble/setup.bash
source ~/Documents/sensors_ws/install/setup.bash
source <experimental_workspace>/install/setup.bash
ros2 launch fast_livo mapping_ugv.launch.py mode:=cohemo_livo.yaml use_rviz:=false use_sim_time:=true
```
For testing with the same bag in a second terminal:
```bash
source /opt/ros/humble/setup.bash
ros2 bag play /media/breidy/Robotics/DATA_SET_SPARK/FAST_LIVO2_SLAM_SPARK_SYNCED_20261019/rosbag2_1970_01_01-01_04_33 -r 0.5 --clock 40 --start-offset 40 --read-ahead-queue-size 1000 --disable-keyboard-controls --topics /mid360/livox/lidar /mid360/livox/imu /cam0/image_raw
```
Do not equate `--read-ahead-queue-size` with DDS QoS depth or internal frontend queues. The production deployment needs a separate long-run Jetson+UGV moving test, visual registration correctness and online latency measurement before promotion.
