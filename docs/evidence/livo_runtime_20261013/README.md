# UGV10 full LIVO runtime attempts (2026-10-13)

Attempts1 and2 FAILED exact raw-input integrity. Attempt3 PASSED all unchanged runtime gates with buffered reliable transport. All attempts are retained; the first two are not release acceptance.

| Attempt | LiDAR /1899 | IMU /37984 | Camera /4502 | Odom | Dense | Type2 links |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 1, BEST_EFFORT probe | 959 | 36922 | 96 | 2970 | 2992 | 32 |
| 2, RELIABLE KEEP_LAST100 probe | 1899 | 37980 | 4471 | 2970 | 2970 | 27 |

Original player raw publishers offer RELIABLE KEEP_LAST10. Camera storage does not show large bursts: minimum interval15.988ms, median33.350ms, maximum50.021ms and at most4 records/100ms. Native frontend Get-image logging is incomplete too (attempt1:4476; attempt2:4443 of4502); logs alone cannot distinguish DDS loss from logging loss. Exact count gates remain unchanged.

Attempt2 offline snapshot comparison matches133/133 graph anchors at exact DB Node timestamps to the saved dense trajectory, max translation0.0000197485m and rotation0.0000377982deg. This is internal consistency only: subscribed snapshots have no terminal-service epoch guarantee, and no ground-truth accuracy is claimed. Runtime FAIL is unchanged.

Raw logs/CDR/odometry CSV are losslessly gzip compressed with mtime0. JSON includes original host artifact paths, sizes and hashes; SQLite databases remain on host with SHA256. Both attempts cleaned their owned process groups, preserving user FAST process116827.

Next transport-only validation uses player raw RELIABLE KEEP_ALL VOLATILE with wait-for-all-acked10000ms and raw/odometry audit readers KEEP_ALL. Rate0.5, FAST/RTAB source and algorithm settings, exact gates, memory32GiB and disk5GiB guards remain. Reader process RSS is included in the next guard. This change tests queue/transport pressure; it does not establish a cause for prior losses.

## Attempt3: PASS with buffered transport

Executed21:04:06UTC, completed after397.416s. Source runtime commitb8075486fa8fef16254a30b1671df9cf146cce32; same compiled mapping/IMU artifacts as previous attempts. Only validation transport/capture changed. Script and profile hashes, actual offered QoS and loaded-library maps/hashes are recorded in JSON.

- Exact raw receipt:1899LiDAR,37984IMU,4502cameras. Camera input timestamp sequence matches the original bag CSV at nanosecond precision; native FAST Get-image log contains all4502 in exact order at its printed six-decimal precision. No missing/extra camera entries. `check_camera_receipt.py` reproduces the comparison.
- Odometry2994, metric clouds2994 and final dense poses2994. Final dense timestamps equal the entire observed odometry sequence, including order. No nonfinite, duplicate or backward odometry. All29,940,192 metric XYZ points finite.
-2448nonempty XYZ+RGB clouds,1797nonempty images, multiple observed colors. Native source remains LIVO, with expected map/camera_init frames.
-133optimized graph poses and31unique type2 closures. Graph-only GetMap2 JSON/CDR captured successfully.133/133anchors match dense by exact timestamp; maximum translation0.0000228944m and rotation0.000033548deg. These measure internal graph/dense consistency, not ground-truth accuracy or equivalence to HA.
- Full sensor bag replay reaches480.37s(IMU),480.24s(LiDAR),440.54s(camera). Camera ends39.70s before LiDAR by dataset construction; LIVO odometry reaches440.51s.
- Peak total owned/probe RSS6,493,417,472bytes under32GiB. Four owned PGIDs independently verified empty after cleanup; all process exit codes0. User FAST PID116827 remained untouched.

Native mapping loads isolated IMU SHA256 `01e2fe9ffa5f63ecb33b519e0afb353a387e936f62144a59c62ab530f639f1c4` and existing baseline VIKIT SHA256 `050e237845f213b422f77ed1d2b9aa626e27f8e989ef881bffb5afad05fdd568`. No production activation is represented by this evidence.

Reproduction command/environment is saved in the attempt3 shell script. Output paths must be fresh. The player QoS file SHA256 is `06bbad1f1133fd51459c0be5191b6bdfb1bf07b6265c780ee126d1518f60be73`; this QoS is scoped to playback, not a global middleware modification. Recovered receipt establishes this configuration passed, not a unique proof of the underlying default-transport failure mechanism.
