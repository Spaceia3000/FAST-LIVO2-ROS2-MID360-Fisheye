# FAST-LIVO2 / UGV — sparse RViz cloud: runtime diagnosis (2026-10-28)

**Target:** Jetson AGX Orin, ROS 2 Humble, running `ugv_realsense_livo.yaml` on FAST-LIVO2
`ugv_lsi_camera_gap@15a5ad8`. Functional code unchanged.

## Observation versus interpretation

- User's RViz screenshots show sparse gray point cloud, stable viewpoint and active RealSense image.
- Live `ros2 param get /laserMapping`: `common.img_en=1`,
  `common.lidar_en=1`, `imu.imu_en=true`,
  `preprocess.filter_size_surf=0.15`.
- Read-only ROS2 subscription sampled eight seconds, recording **message headers and counts only**:
  - `/cloud_registered`: **119** messages, **60 empty**, 59 nonempty; max **3774 points**.
  - `/cloud_registered_metric`: **41** messages, **0 empty**, min **23774**,
    median **26682**, max **46697 points**.
  - `/camera/camera/color/image_raw`: **226** images, all **640x480**.
  - `/Laser_map`: **no messages during observation**, although one FAST-LIVO2 publisher exists.
  - `/cloud_visual_sub_map_before`: no messages observed.
  - `/pgo_rgb_map_cloud`: **zero publishers**; backend
    `simple_loop_closure_node` was not running at this time.
- The 8-second observation is a finite snapshot; does not establish hour-long
  queue, estimator, tracking or mapping stability.

## Verified implementation causes

1. In `src/LIVMapper.cpp`, the LIO stage passes a current-world-frame cloud
   (selected via `dense_map_en ? feats_undistort : feats_down_body`) into
   `pcl_w_wait_pub`; `/cloud_registered_metric` publishes that LiDAR-frame cloud.
   `dense_map_en: true` **does not mean globally accumulated RViz history**.
2. `publish_frame_world` emits RGB data when `slam_mode_ == LIVO` AND
   `lio_vio_flg == VIO`. When LIO processing calls the same function in LIVO
   mode, the outgoing `PointCloud2` is empty. The 60/119 empty messages
   are consistent with this branch behavior. The RGB output is also restricted
   to points whose camera projections fall within the visible camera frame.
3. `rviz_cfg/ugv_livo.rviz`: `/cloud_registered` and
   `/cloud_registered_metric` both use `Decay Time: 0`. This shows the
   latest message, not an accumulated map. Because roughly half of RGB
   messages are empty, the displayed RGB cloud may blink/appear sparse.
4. The global corrected RGB display is enabled but
   `/pgo_rgb_map_cloud` has **no publisher** without a live
   SimpleLoopClosure process; no loop-corrected map can appear.
5. Although `/Laser_map` has a declared publisher in `LIVMapper.cpp`,
   there is no publish call for `pubLaserCloudMap` in the inspected version;
   no map message was received. A topic name alone is not proof of publication.
6. SimpleLoopClosure's `vis_map_voxel_leaf_size=0.4`,
   `rgb_global.frame_voxel_leaf_m=0.15`,
   `rgb_global.map_voxel_leaf_m=0.20`, plus caching bounds,
   further affect display density WHEN that backend is actually running.

## Immediate no-code experiment (operator-side RViz only)

1. In RViz, under `mapping`, keep **UGV metric cloud** on
   `/cloud_registered_metric`; temporarily disable overlapping
   `Legacy display diagnostic` on `/cloud_registered` and redundant overlays.
2. For **UGV metric cloud**, change `Decay Time` from `0` to **3 seconds**
   (increase to **5 seconds** only if Jetson frame rate and RAM remain healthy).
   Use `Style=Points`, `Size (Pixels)=2` or `3`.
   This accumulates **only on RViz**; it does NOT produce a persistent SLAM map.
3. Optionally enable `/cloud_registered` in isolation, with
   `Decay Time=3` and `Color Transformer=RGB8` to visualize the projection
   coverage rather than mistaking a single empty LIO message for camera failure.
4. For global RGB map and trajectory corrections, start backend separately:
   `ros2 launch simple_loop_closure simple_loop_closure.launch.py mode:=livo`.
   Verify publisher of `/pgo_rgb_map_cloud`, matching time stamps for
   `/cloud_registered_metric` and `/aft_mapped_to_init`, and accepted loops.
   This is NOT a claim that loop closure will be accepted.
5. Avoid changing `filter_size_surf`, camera extrinsics, IMU calibration,
   memory constraints or code until these visualization and backend gates pass.

## Next engineering decision

If visual accumulation works but a persistent global map is still required:
develop and validate a **separate bounded accumulation/export pipeline**
or a minimal publisher for the existing map structure, on an isolated feature
branch first. Avoid converting a per-frame stream into an unbounded copy on
the Jetson. Require point counts, memory, CPU, latency and map accuracy
measurements before considering a merge. Promotion to
`integration/v1-humble` remains **blocked pending physical operator acceptance**.

**Evidence location (Lenovo):**
`~/Documents/fast_livo2_validation_reports/2026-10-28_ugv_b3_acceptance/`.
No diagnostic log or report is written to the Jetson for this gate.
