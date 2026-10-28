# UGV_LSI RealSense B3 candidate — 2026-10-28

This profile is intentionally an independent full ROS 2 YAML rather than
replacing the operational ugv_livo.yaml. It changes only three scalars
relative to that baseline: 0.15 m filtering, 7.5 Hz image admission, and
ingest diagnostics enabled. No extrinsic, intrinsic, IMU or LiDAR time-offset
changes are permitted.

The scripts/s1_prepare_yaml_ab.py generator from the historical
c40cf24 A/B tool is retained and adapted to current names. Its generated
B3 file must match config/ugv_realsense_livo.yaml exactly. The manifest and
checksums live at reports/experiments/ugv_realsense/b3_202610/.

Run in live UGV only after build and tests; user controls all motion.
mode:=ugv_realsense_livo.yaml camera_params_file:=camera_realsense.yaml
selects the candidate. mode:=ugv_livo.yaml selects the reference.
No change is approved to integration/v1-humble or main.
