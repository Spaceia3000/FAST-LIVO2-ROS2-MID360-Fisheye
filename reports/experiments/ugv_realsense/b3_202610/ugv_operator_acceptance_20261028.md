# UGV RealSense B3 — operator acceptance gate

**Date:** 2026-10-28. **Stage:** software candidate ready; hardware acceptance PENDING.
**Authoritative code:** FAST-LIVO2 `ugv_lsi_camera_gap` at `15a5ad8648aa7889fcfc4fddcc4f9446c164f690`;
SimpleLoopClosure `ugv_lsi` at `c272c28`;
Vikit independent `fix/jetson-humble-ament-vikit` at `d218bd3`.
**Do not merge into `integration/v1-humble` or `main` before UGV acceptance.**

## Storage boundary

The **Lenovo laptop** holds all reports, stdout/stderr logs, metrics, hashes, plots, screenshots and recordings:
`~/Documents/fast_livo2_validation_reports/2026-10-28_ugv_b3_acceptance/`.
The Jetson holds only required ROS 2 workspaces, their build/install/log directories and normal runtime data.
Do not generate extra bags, benchmarks, backups, screenshots or experiment reports on Jetson.
Never rewrite calibration YAML, existing UGV reference profiles or user-edited RViz files during acceptance.

## Preflight / exact version gate — ON JETSON

Ensure the required drivers and camera are already started *once*, not duplicated. Verify
`ros2 topic list` includes `/mid360/livox/lidar`, `/mid360/livox/imu`,
and `/camera/camera/color/image_raw` (camera needed for LIVO).
`ros2 pkg prefix fast_livo` must resolve to the active Jetson workspace.
Before running, verify the Jetson workspace's FAST source is on the candidate branch and
HEAD equals the SHA above; abort if not. Validate the `simple_loop_closure` package is
installed before enabling the loop backend. The exact source script paths are device-dependent;
confirm `/opt/ros/humble/setup.bash` and the installed Livox, Vikit, FAST and SLC overlays first.

## Autonomous FAST-LIVO2 tests — separate ROS terminals

After sourcing the installed ROS 2 Humble and pinned sensor/Vikit/FAST underlays
in each Jetson terminal:

```bash
# LO: LiDAR only
ros2 launch fast_livo mapping_ugv.launch.py mode:=ugv_lo.yaml use_sim_time:=false use_rviz:=false
# LIO: LiDAR + IMU
ros2 launch fast_livo mapping_ugv.launch.py mode:=ugv_lio.yaml use_sim_time:=false use_rviz:=false
# LIVO reference: LiDAR + IMU + RealSense, admission 10 Hz, filter 0.10 m
ros2 launch fast_livo mapping_ugv.launch.py mode:=ugv_livo.yaml use_sim_time:=false use_rviz:=false
# LIVO B3 candidate: admission 7.5 Hz, filter 0.15 m
ros2 launch fast_livo mapping_ugv.launch.py mode:=ugv_realsense_livo.yaml use_sim_time:=false use_rviz:=false
```

These are **four alternative runs**: launch one at a time, never simultaneously.
If the UGV has an active graphical display, `use_rviz:=true` is optional; on a
headless Jetson, keep it false and visualize topics from a ROS-connected computer.
The launcher selects `rviz_cfg/ugv_lio.rviz` for LIO and
`rviz_cfg/ugv_livo.rviz` for LIVO.

## SimpleLoopClosure backend — additional terminal, FAST already running

After sourcing the SimpleLoopClosure workspace, run one backend matching FAST:

```bash
# When FAST is in LIO mode
ros2 launch simple_loop_closure simple_loop_closure.launch.py mode:=lio

# When FAST is in LIVO or B3 mode
ros2 launch simple_loop_closure simple_loop_closure.launch.py mode:=livo
```

The backend consumes `/aft_mapped_to_init` and
`/cloud_registered_metric`; both must be published with matching stamps and
compatible frame IDs. Its corrected RGB map topic is `/pgo_rgb_map_cloud`.
**The current SimpleLoopClosure launch contract supports LIO and LIVO, NOT LO**.
Do not claim LO+loop-closure validation without implementing and testing it.

## Acceptance plan / measurements (all evidence on Lenovo)

1. Confirm loaded profile and branch, camera/LiDAR/IMU message rates, monotonic stamps,
   TF frame chain and time base; check image calibration and the intended camera topic.
2. Run short static and controlled motion trials in LO, LIO, LIVO reference, and B3;
   inspect trajectory continuity, point-cloud geometry, RGB alignment and runtime warnings.
3. Compare LIVO reference versus B3 on the same repeatable route and conditions.
   For each trial log time, CPU/GPU/RAM, message frequencies, estimator queues/backlog,
   sensor age/dropouts, map memory, tracking failures and processing latency p50/p95.
4. Run B3 for **at least 60 minutes with sustained movement**. Do not mark PASS for
   51 minutes, for stationary operation, or if unbounded backlog occurs near the end.
   Validate numerical stability, no estimator loss, no runaway queues or memory growth.
5. In LIO and LIVO, activate SimpleLoopClosure separately; test a genuine revisit,
   accepted/rejected loop constraints, corrected geometry and RViz before/after.
   A visual jump by itself is insufficient evidence: inspect backend statistics and pose error.
6. Explicitly verify no camera subscription/wait in LO/LIO, reference RealSense
   configuration and the user's RViz files remain unchanged.

**Gate status:** PENDING physical UGV validation. Track each run with revision SHA,
profile name, initial/final time, operator, dataset/route, pass criteria and failures.
After user confirms PASS, open a reviewed PR into `integration/v1-humble`.
Do not automatically merge based on compile/test success alone.

## Software evidence

See `arm64_software_validation_20261028.md` (Jetson historical gate: ARM64 Release
build PASS; C++ 13/13 PASS; Python 10/10 PASS; finite-point test under -ffast-math
PASS). Those software gates do **not** establish physical B3 stability.
