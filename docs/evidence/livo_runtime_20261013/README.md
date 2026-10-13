# UGV10 full LIVO runtime attempts (2026-10-13)

Both attempts FAILED exact raw-input integrity. These are preserved failures, not release acceptance.

| Attempt | LiDAR /1899 | IMU /37984 | Camera /4502 | Odom | Dense | Type2 links |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 1, BEST_EFFORT probe | 959 | 36922 | 96 | 2970 | 2992 | 32 |
| 2, RELIABLE KEEP_LAST100 probe | 1899 | 37980 | 4471 | 2970 | 2970 | 27 |

Original player raw publishers offer RELIABLE KEEP_LAST10. Camera storage does not show large bursts: minimum interval15.988ms, median33.350ms, maximum50.021ms and at most4 records/100ms. Native frontend Get-image logging is incomplete too (attempt1:4476; attempt2:4443 of4502); logs alone cannot distinguish DDS loss from logging loss. Exact count gates remain unchanged.

Attempt2 offline snapshot comparison matches133/133 graph anchors at exact DB Node timestamps to the saved dense trajectory, max translation0.0000197485m and rotation0.0000377982deg. This is internal consistency only: subscribed snapshots have no terminal-service epoch guarantee, and no ground-truth accuracy is claimed. Runtime FAIL is unchanged.

Raw logs/CDR/odometry CSV are losslessly gzip compressed with mtime0. JSON includes original host artifact paths, sizes and hashes; SQLite databases remain on host with SHA256. Both attempts cleaned their owned process groups, preserving user FAST process116827.

Next transport-only validation uses player raw RELIABLE KEEP_ALL VOLATILE with wait-for-all-acked10000ms and raw/odometry audit readers KEEP_ALL. Rate0.5, FAST/RTAB source and algorithm settings, exact gates, memory32GiB and disk5GiB guards remain. Reader process RSS is included in the next guard. This change tests queue/transport pressure; it does not establish a cause for prior losses.
