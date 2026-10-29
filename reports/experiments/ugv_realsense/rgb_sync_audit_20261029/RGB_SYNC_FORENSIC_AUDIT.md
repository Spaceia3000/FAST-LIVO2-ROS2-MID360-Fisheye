# FAST-LIVO2 / RealSense D435i / MID-360 — RGB synchronization forensic audit
Date: 2026-10-29. Base: Spaceia3000/FAST-LIVO2-ROS2-MID360-Fisheye
`ugv_lsi_camera_gap@36b97b1488e8dddd334a6988fc8a5dec50d05be6`.
This analysis is read-only with respect to UGV code, calibration, main and integration.

## Evidence hierarchy
- **Direct live Jetson telemetry from 2026-10-28:** 8-second ROS sample:
  RealSense `/camera/camera/color/image_raw`: 226 messages (640x480);
  `/cloud_registered`: 119, of which 60 empty and 59 nonempty, max 3774 points;
  `/cloud_registered_metric`: 41 messages, none empty, 23774–46697 points;
  `/pgo_rgb_map_cloud`: zero publishers at that time.
  These are limited samples, not hardware frame-loss or calibration proofs.
- **Direct code inspection on Lenovo** (specific functions/lines, not every
  source file in the repository): `src/LIVMapper.cpp`,
  `src/vio.cpp`, `include/image_admission.h`,
  `include/livo_camera_fallback.h`, `config/ugv_realsense_livo.yaml`
  and `config/camera_realsense.yaml`.
- **Offline C++ reproducer on Lenovo:** `image_admission_cadence_probe.cpp`
  includes the actual project header and tests synthetic 30-Hz inputs.
  `image_admission_cadence_probe_result.txt` records 54/240 admitted
  for nominal 7.5 Hz (6.75 Hz) and 64/240 for nominal 10 Hz (8 Hz).
  This is a deterministic numerical edge case; it does not establish observed
  Jetson rates in either configuration.

## Root causes and confidence
1. **Confirmed: empty RGB PointCloud2 publication, not lost camera images.**
   `LIVMapper.cpp:610-745`: LIO stage invokes `publish_frame_world`.
   `LIVMapper.cpp:1661-1717`: in LIVO mode only the VIO branch fills
   `laserCloudmsg`, but the publish call always executes, including LIO.
   A zero-point cloud can overwrite an RViz PointCloud2 display using
   `Decay Time: 0`. No hard evidence of loss of camera frames follows.
2. **Confirmed: intentional frame admission cap.**
   `img_cbk` at `LIVMapper.cpp:1255-1299` calls
   `imageAdmissionDecision` before BGR copy. The B3 profile
   `runtime.image_admission_hz=7.5` deliberately accepts a subset of the
   30-Hz input stream. Rejection reasons DUPLICATE/OUT_OF_ORDER/TOO_SOON
   are currently combined in `camera_skipped_count`; this is insufficient
   evidence for classifying actual device, DDS or callback drops.
3. **Confirmed: additional scheduler-based image rejection is possible.**
   `sync_packages` at `LIVMapper.cpp:1383-1607`:
   chronological LIO cut, paired VIO if time agrees within 10 us,
   stale-image removal if `capture_time <= last_sensor_cut_time + epsilon`,
   and fallback LIO if image stream absent for 0.30 s of sensor time.
   A delayed *arrival* (not necessarily missing acquisition) can thus
   become stale and be removed. Measure this with counters before judging.
4. **Confirmed: limited image field of view and projection filter.**
   `publish_frame_world` rejects world points with camera depth <0 and
   projections outside the image margin. With `fx=607.838`, `fy=607.252`
   at 640x480 the pinhole model covers approximately 55.5° horizontal,
   43.1° vertical. The 360° MID-360 necessarily has many points outside
   the camera FOV. A sparse RGB point cloud does not alone imply bad
   LiDAR-camera synchronization.
5. **Plausible, not established:** camera and LiDAR clock-origin/offset,
   actual image exposure timestamp semantics, camera intrinsics/distortion
   mismatch with `/camera/camera/color/camera_info`, extrinsic orientation,
   motion compensation, OS scheduling, and RealSense USB/DDS losses.
   Verify each independently.
6. **Confirmed: callback execution may be serialized with mapping work.**
   `run()` at `LIVMapper.cpp:833-870` spins callbacks (budget 5 ms)
   on the same thread as `stateEstimationAndMapping`. Prolonged visual
   optimization may delay image callback intake. Changing to multi-threaded
   callbacks WITHOUT synchronizing every shared deque access in
   `sync_packages` is unsafe and not recommended as an immediate fix.

## Key clock and projection conventions
- Raw camera time adjusted in frontend:
  `t_image_corrected = header.stamp + img_time_offset`, currently offset 0.
- Internal VIO capture time:
  `t_capture = t_image_corrected + exposure_time_init`, currently 0.
- IMU handling in frontend:
  `t_imu_corrected = header.stamp - imu_time_offset`; configured
  `imu_time_offset=-0.017737` means about +17.737 ms on the IMU header.
  Do not change its sign or reset offsets without recalibration.
- LiDAR point time in data cut:
  `t_point = lidar_header + curvature_ms/1000`.
  The header convention must be start-of-scan if this relation is to hold.
- Correct projection:
  `p_c = T_c_w(t_image) p_w(t_point)`,
  `u=fx*x_c/z_c+cx`, `v=fy*y_c/z_c+cy`;
  requires matching clock conventions and calibrated extrinsics.
  Timing error during motion roughly induces image error
  `delta_u ~= fx * omega * delta_t` for a rotational component:
  at 30 degrees/s, 10 ms yields ~3.18 pixels with this camera fx.
  This is a first-order example, not a fixed accuracy threshold.

## Remediation architecture (no changes deployed)
### Gate A — Correct the ROS cloud publication contract in a feature branch
- Preserve LO/LIO behaviors, odometry, world point transforms and PCD-saving.
- In LIVO do not publish uninitialized `PointCloud2` from LIO updates;
  publish RGB on VIO updates only and count zero-projection VIO events
  separately. Preserve sensor-stamp contract and avoid new copying.
- Tests: LO, LIO, LIVO; no empty RGB on LIO; no regression in metric cloud,
  cloud_registered fields/QoS/stamps and SimpleLoopClosure consumers.
- Do NOT use an early return from `publish_frame_world` that also skips
  downstream PCD-save/buffer reset logic.

### Gate B — Disaggregate camera acquisition/admission/scheduling metrics
Counters: driver image arrival, callback received, ACCEPT, DUPLICATE,
TOO_SOON, OUT_OF_ORDER, deque stale, image-to-VIO consumed, fallback LIO,
VIO update succeeded, VIO skipped due to missing LiDAR points.
Histogram: interarrival dt(header), header-to-host age, arrival-to-process
latency, camera/LiDAR/IMU queues, oldest queue age, VIO process latency,
image-to-IMU offset residuals. Keep diagnostic logs/results on **Lenovo**.
The current `[ingest]` logger is enabled by B3, but its
`camera_skipped_count` does not disaggregate reasons.

### Gate C — Correct cadence and time association conservatively
- Add a scheduler regression test over nanosecond timestamp sequences
  and several epochs (zero, UNIX) with small positive/negative jitter.
  Review strict floating double minimum-interval boundary before changing
  `image_admission_hz`. Suggested dedicated monotonic scheduling/epsilon
  is a design candidate, not yet implemented or validated.
- First validate camera `header.stamp` origin, driver frame metadata,
  LiDAR MID-360 timestamp sync state, IMU corrected stamps, and actual
  image exposure convention. Compare `camera_info` with
  `camera_realsense.yaml`; run fixed-target LiDAR-to-image overlay.
- Only after that consider software temporal offset estimation, hard sync
  where hardware supports it, or re-calibration (FAST-Calib, Kalibr).
  Do not retimestamp using `now()` or adjust by eye.

### Gate D — Resource gate and physical UGV
Run 7.5 Hz/0.15 m B3 versus reference 10 Hz/0.10 m on the same
route. Collect camera acceptance/stale counters, projection percentages,
VIO photometric residuals, timing p50/p95, CPU/RAM and estimator queues.
Acceptance requires repeatable geometric correctness and 60+ minutes
of movement without unbounded backlog. Only user acceptance allows
future promotion to `integration/v1-humble`.

## External sources reviewed (official/primary)
- HKU-MARS FAST-LIVO2 code/paper:
  https://github.com/hku-mars/FAST-LIVO2
  https://doi.org/10.1109/TRO.2024.3502198
- HKU-MARS timestamp semantics / Mid360 community issues:
  https://github.com/hku-mars/FAST-LIVO2/issues/156
  https://github.com/hku-mars/FAST-LIVO2/issues/120
  https://github.com/hku-mars/FAST-LIVO2/issues/458
- RealSense ROS2 wrapper, metadata, stream diagnostics:
  https://github.com/realsenseai/realsense-ros
  https://github.com/realsenseai/realsense-ros/issues/2963
- Livox MID-360 timestamps (0=boot time, 1=PTP):
  https://github.com/Livox-SDK/livox_wiki_en/blob/master/source/tutorials/new_product/mid360/livox_eth_protocol_mid360.md
- ETH Zurich Kalibr temporal/spatial calibration:
  https://github.com/ethz-asl/kalibr/wiki/Camera-IMU-calibration
- OpenVINS time-offset estimation and calibration:
  https://docs.openvins.com/gs-calibration.html
- HKU-MARS FAST-Calib:
  https://github.com/hku-mars/FAST-Calib
- Oxford Robotics LiDAR-visual-inertial:
  https://robots.ox.ac.uk/~mfallon/publications/2021RAL_wisth.pdf
- KAIST online LiDAR-camera calibration:
  https://sgvr.kaist.ac.kr/wp-content/uploads/2024/09/CalibFormerNet_Moon.pdf

## Limitations and next actions
Jetson remote was offline at this audit, so no new real-time sensor traces,
cross-device timestamp comparison, user UGV motion or calibration data.
No calibration, runtime profile, ROS 2 or estimator code was changed.
All analysis work and test files live on Lenovo and this isolated GitHub
analysis branch. `integration`, `main`, UGV workspaces are untouched.
