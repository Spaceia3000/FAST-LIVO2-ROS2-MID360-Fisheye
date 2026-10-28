# ARM64 software validation — 28 October 2026

## Exact revision and execution environment

- FAST-LIVO2 feature commit: db724894ab0f0802699431488fdb51bb37bf4501.
- Parent: ugv_lsi_camera_gap at 6c26bc413676278dfcd6c1272daf5047596267e5.
- Jetson AGX Orin, Ubuntu 22.04, ROS 2 Humble, Release build.
- Build underlay: /opt/ros/humble, ws_livox_driver/local_setup.bash, ws_vikit/local_setup.bash.
- Validation build location: ws_fast_livo2/build_b3_validation_20261028.
- Validation install: ws_fast_livo2/install_b3_validation_20261028.
- Build log: ws_fast_livo2/log_b3_clean_20261028.
- C++ test log: ws_fast_livo2/log_b3_test_20261028.
- Build output: PASS, one nonfatal CMP0074 PCL_ROOT developer warning.
- GoogleTest via colcon: 13 cases, 0 errors, 0 failures, 0 skipped
  (4 LIVO camera fallback; 7 visual map growth; 2 ament test framework checks).
- Python: 10 tests PASS (A/B generator, profile contracts, VIO memory).
- Historical B3 generator output exactly matches ugv_realsense_livo.yaml.
- LO/LIO/LIVO/Cohemo mode contracts checked with actual installed launch module.
- LIO contains no runtime image admission / camera wait / spin budget block.
- Original ugv_lio.yaml, ugv_livo.yaml and camera_realsense.yaml not modified.
- Runtime dynamic library: libvikit_common.so resolves to ws_vikit/install/vikit_common/lib.
- Executable SHA256: 9481abf8fc5c98da83bdb157f063efd71c948ff71bb686926355d38d36ba70fd.
- Fallback GTest XML SHA256: 798d6544c71417db3225b6ca544a37957e18a5235739c07c9bc9009cfbbc73f8.
- Visual map GTest XML SHA256: fcc14d8d81c3a31c2fdeeb029d38942c65b5c2e54df9ed0c9495ece1c7c065ff.

## Discovered regression and resolution

The historical visual map guard used std::isfinite in voxelCoordinate.
ARM Release enables -ffast-math, and the restored historical non-finite test
failed specifically for NaN. The minimal fix at db72489 examines IEEE-754
float exponent bits using memcpy. The test now passes with the actual Release
compiler flags. This is a targeted numeric guard, not a change to the
estimator's geometric or inertial formulation.

## Not proven by these tests

No physical UGV movement or sensor-stream runtime was executed in this gate.
No claim of one-hour real-time stability, absolute accuracy, RGB reprojection
quality or loop closure success is made. The user must run and approve the
physical UGV validation before any promotion to integration/v1-humble.
