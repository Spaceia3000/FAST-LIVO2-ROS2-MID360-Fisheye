# RealSense D435i — B3 reference

The 14–15 October 2026 B3 profile used visual admission 7.5 Hz,
LiDAR filter_size_surf=0.15 m and ingest telemetry enabled.

Historical operational observation: approximately 51 minutes with low estimator
backlog while moving and using RViz, followed by increasing backlog. This is
partial validation only, not a one-hour PASS. Complete original Jetson B3
logs were not located during the 28 October audit.

The present candidate preserves the later LIO camera-gap fallback (0.30 s),
IMU propagation-origin fix and independently installed Vikit. Those later
changes have not been validated jointly with B3 during a moving UGV trial.

Required operator acceptance: 60+ minutes moving, stable backlog/queues,
valid geometry, bounded memory and no lost localization. Until then, the
existing ugv_livo.yaml remains the operational reference.
