# Native IMU propagation-origin regression — 2026-10-13

## Result and scope

Executed on the Legion, Ubuntu 22.04 / ROS 2 Humble, with native C++ `ImuProcess::Process2` and `UndistortPcl`. Both arms compiled the real `imu_proc` library and the same regression test with Release options. Maximum build concurrency was 2. No FAST mapping process, rosbag, production install, calibration, threshold, or user session was modified.

| Arm | Git revision | Tests | Exit |
| --- | --- | --- | --- |
| Before: unified baseline plus test only | `092010e` (parent `cf22c794`) | 0 passed, 2 failed | 1 (expected) |
| After: initialization fix | `eed007f` | 2 passed, 0 failed | 0 |

The first fixture placement-constructs `ImuProcess` in storage filled with byte `0x43`; this leaves a large finite pattern in the previously uninitialized double. At a nonzero epoch of 1000 seconds, constant angular velocity 1 rad/s over 0.1 seconds must produce yaw 0.1 rad. Before gives 0 rad; after satisfies the 1e-6 rad tolerance and finite pose/velocity/covariance checks.

The second fixture propagates, calls `Reset`, initializes at the earlier epoch 100 seconds, and propagates 0.05 seconds. Before retains a stale origin and gives 0 rad; after satisfies expected 0.05 rad with the same tolerance and finite-state checks. Both tests also check the reported LIO update endpoint.

This proves a concrete initialization/reset defect and its narrow correction. It does not prove that this defect caused a particular observed loop closure, provide ATE/RPE, validate an entire LIVO dataset, or compare backend accuracy. The synthetic fixture deliberately isolates constant-rate propagation; it is not a model of the platform's stationary initialization procedure.

## Why this correction

`last_prop_end_time` is read at the start of `UndistortPcl` and controls integration clipping. The user-defined constructor previously left it indeterminate; the successful initialization path returned without assigning it. Reset could retain a timestamp from a previous run. The fix gives it an explicit invalid initial/reset value and assigns the actual selected LIO/VIO measurement endpoint after each successful IMU initialization batch. It does not use arbitrary epoch zero or change normal propagation equations.

## Reproduce

The exact executed script is `run_native_test.sh`. It uses these isolated source worktrees and build directories, with explicit installed VIKIT CMake package locations. The hardcoded VIKIT linker paths on `laser_mapping` are not involved because only `imu_proc` and the test target are built.

If the before worktree is absent, create it from the retained test-only commit:

```bash
git -C /home/breidy/Documents/fast_livo2_worktrees/imu_init_time_origin_20261013 worktree add --detach /home/breidy/Documents/fast_livo2_worktrees/imu_init_time_origin_before_20261013 092010e
bash /home/breidy/Documents/fast_livo2_worktrees/imu_init_time_origin_20261013/docs/evidence/imu_time_origin_20261013/run_native_test.sh
```

The script writes evidence at its recorded paths; preserve this checked-in evidence before rerunning. The run shown here completed with `before test exit=1` and `after test exit=0`.

## Evidence and setup issues

- Configure/build logs, raw test stdout, JUnit XML, and explicit return codes are retained for both arms.
- `environment.json` pins source revisions, compiler, kernel, CMake and scope.
- `artifact_hashes.json` records executable, real IMU shared-library, and compile-command hashes. Binaries remain in isolated host build directories, not Git.
- `before.ldd.txt` and `after.ldd.txt` were captured after sourcing the same baseline setup as the test script; all dependencies resolve, and each test loads its own isolated IMU library. Files ending `.ldd.unsourced.txt` retain an initial diagnostic made without sourcing ROS; its missing ROS libraries are not a test failure.
- `campaign.setup_failure.log` preserves the first setup attempt, which stopped before CMake because this VIKIT install has no `local_setup.bash`. The successful script uses explicit `vikit_common_DIR`/`vikit_ros_DIR` instead; no installed package was edited.
- `SHA256SUMS` hashes all evidence files except itself.

The branch is separate from `integration/v1-humble`. No merge or production deployment is included in this validation.
