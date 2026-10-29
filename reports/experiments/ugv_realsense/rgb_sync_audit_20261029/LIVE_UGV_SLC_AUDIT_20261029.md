# UGV real-time audit — FAST-LIVO2 + SimpleLoopClosure, 2026-10-29

**Method:** Remote read-only inspection of Jetson AGX Orin and Lenovo; 9-second
read-only ROS 2 subscriber sample, subsequent graph and ROS log inspection.
No UGV node was stopped/restarted; no runtime YAML/calibration/code was edited;
no separate datasets, reports, or bags were created on Jetson.

**Runtime sources on Jetson NVMe**
- FAST-LIVO2: `ws_fast_livo2/src/FAST-LIVO2-ROS2-MID360-Fisheye`,
  `ugv_lsi_camera_gap@15a5ad8648aa7889fcfc4fddcc4f9446c164f690`.
- SimpleLoopClosure: `ws_simple_loop_closure/src/simple_loop_closure`,
  `ugv_lsi@c272c289c5a5e2867d35cfc12f8cf247f36dd131`.
- FAST-LIVO2 uses `config/ugv_realsense_livo.yaml`, LIVO B3,
  7.5 Hz visual admission, 0.15 m surface filter.
- The user's working-tree RViz differences in `cohemo_livo.rviz`
  and `ugv_livo.rviz` were preserved without editing.

## Operational evidence: positive
- Live processes: Livox MID-360 driver, RealSense D435i node,
  FAST-LIVO2 laserMapping, mapping_ugv_rviz, and simple_loop_closure.
- /cloud_registered_metric was published. In a 9-second
  live sampling window 58 cloud messages and 58
  /aft_mapped_to_init odometry messages had **58/58 identical nanosecond
  header timestamps**. Thus the ExactTime data contract was met for
  the observed window; this does not certify packet delivery indefinitely.
- Typical LiDAR metric snapshots contained approximately 24k–41k points
  (one observed 26,719). Fields x/y/z/intensity/normal/curvature,
  **no rgb**. This is a primary source of the gray 3D geometry
  in the user's screenshot.
- RGB corrected global cloud /pgo_rgb_map_cloud is genuinely published
  with x/y/z/rgb. One directly received message contained 15,597 points.
- The raw RGB cloud /cloud_registered can emit a zero-point message
  in LIVO during LiDAR-only frontend updates (observed in a direct sample,
  consistent with the prior 2026-10-28 source-code diagnosis). Do not
  confuse that message with a lost USB/camera image.
- FAST ingest log samples showed 7–8 accepted camera images per second,
  19–24 admission-skipped images/s, camera queue ~0–1, LiDAR queue ~0–1,
  IMU queue ~29–46, cumulative camera stale count 3 and fallback-LIO
  cuts 3. Snapshot ages typically ~0.05–0.21 s; 15-minute
  running process alone is not proof of 60-minute acceptance.
- Jetson memory snapshot: 61 GiB total, ~55 GiB available; process CPU
  estimates FAST-LIVO2 ~145% and RViz ~95% (Linux percentage of one CPU).

## Loop-closure live proof: 0 -> 7 accepted as motion resumed
- Initial marker snapshot: 24 original and optimized keyframes, 0
  closure edges, 0 m correction. During that window the UGV moved
  ~2.8 mm in ~8 seconds, below the keyframe spacing threshold of 0.5 m.
  Stationarity explains the absence of new keyframes in that sample.
- Subsequent live snapshot: **102 original/optimized keyframes**, 14
  marker LINE_LIST points = **7 accepted loop edges**; maximum
  original-to-optimized keyframe position displacement **0.00257 m**
  (2.57 mm). Do not confuse this with absolute pose accuracy.
- **Independently confirmed by ROS backend log:**
  `Loop Detected:88 -> 0`
  `Loop Detected:90 -> 4`
  `Loop Detected:92 -> 6`
  `Loop Detected:94 -> 2`
  `Loop Detected:96 -> 1`
  `Loop Detected:98 -> 0`
  `Loop Detected:100 -> 0`.
- In SimpleLoopClosure source
  `simple_loop_closure_node.cpp::processAvailableKeyFrames`,
  edges are pushed into `loop_edges_` **only after** the registration
  gate and successful GTSAM iSAM2 update. Thus 7 edges are more than
  visualization; they verify the active backend accepted constraints.
- Current configuration: keyframe distance 0.5 m or angle 0.3 rad;
  candidate needs >30 s temporal separation AND >30 m
  trajectory-distance separation, local radius <=15 m,
  and converged ICP with fitness score <0.3.
  These thresholds are the current operational constraints,
  not validated optimal values.

## Source of gray cloud in screenshots
- RViz `mapping/UGV metric cloud` is enabled and subscribes
  to `/cloud_registered_metric` with `Color Transformer: Intensity`.
  It draws a point cloud using laser intensity rather than camera RGB.
- The user also enabled `Legacy display diagnostic` on
  `/cloud_registered` and `mapping/PointCloud2` on
  `/cloud_effected`, allowing overlapping displays. The latter's
  FAST publisher is declared, but the active `pub_effect_point_en`
  parameter is false, so do not infer it is actively sending points.
- `GLOBAL - Simple Loop Closure (GTSAM)/Mapa global RGB corregido`
  subscribes to `/pgo_rgb_map_cloud`, contains RGB fields and was
  observed to publish. The environment can itself be gray/white, so
  color perception is not an objective loop-closure success metric.
- **Operator test without code changes:** uncheck `UGV metric cloud`
  and all `Legacy` cloud displays; leave only
  `Mapa global RGB corregido (GTSAM)` enabled. To compare raw
  geometry, enable them individually. Do not save over the user's
  modified RViz without explicit operator choice.

## Why the corrected RGB global map can look sparse
The SimpleLoopClosure `rgb_global` cache is bounded:
frame_admission_hz=5, frame_voxel_leaf_m=0.15,
map_voxel_leaf_m=0.20, max_cached_frames=1200,
max_points_per_frame=2500, max_output_points=350000,
association_max_delta_s=0.75. A backend log sample at approximately
18:38 showed `frames_cached=1200 frames_matched=229
frames_unmatched=971 global_points=15597
keyframes_optimized=102` and many frames skipped by its
display-oriented admission policy. When the robot remained stationary,
keyframes did not increase but RGB frames continued to enter the
rolling cache, so valid historical associations aged out; the map
became sparse or stopped updating even though FAST-LIVO2 geometry
continued normally. This indicates a **visualization cache-design
limitation**, not proof of SLAM estimator failure.
Do not increase cache bounds blindly: memory, CPU, and association
quality must be measured first.

## Limitations and remaining experimental gates
- No external ground truth, ATE/RPE, calibrated RGB reprojection
  residuals, loop fitness history, true revisit labeling, or
  before/after map alignment metric was gathered during this session.
- The 7 accepted ICP/GTSAM constraints show the backend is operational,
  but do not certify that all constraints are geometrically correct.
  Correction of 2.57 mm is small and must be interpreted with
  trajectory geometry and independent reference.
- No 60-minute moving-UGV test was completed by the assistant;
  no conclusion of 60-minute stability is warranted.
- Next: controlled revisit of a known landmark, independent
  spatial/temporal metrics, accepted/rejected loop counts,
  drift-before/after and RGB projection fidelity; optional isolated
  design improvement to keep keyframe-associated RGB frames in cache.
- **Do not merge** to `integration/v1-humble` without operator
  validation and approval. No code/calibration changes performed.

## Minimal operator checks (Jetson)
```bash
ros2 node list
ros2 topic info /cloud_registered_metric
ros2 topic info /pgo_rgb_map_cloud
ros2 topic echo --once /simple_loop_closure/loop_edges --field points
```
Loop marker is LINE_LIST, thus every two Point elements identify
one accepted edge. For this session 14 points = 7 accepted constraints.
