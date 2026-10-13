# Atomic FAST executable activation safety

Prepared for activation only after isolated regression PASS and review/merge of the FAST branch. No production build, activation or restart was performed by this evidence step.

`tools/validation/stage_imu_release.sh <full-Git-SHA>` builds the unified production source into a fresh `fast_livo2_baseline_ws/releases/imu_origin_<SHA>` directory with two jobs and an isolated staging install. It refuses an existing release and a changed source tree. The old build directory is never compiled or overwritten. ROOT_DIR remains the production source.

The stage verifier records source SHA, executable/library hashes, ELF runtime path, and dynamic bindings. All six native FAST implementation libraries must resolve inside the new release build. The executable is the build artifact, preserving its verified RUNPATH; it is not copied through a stripping install step.

`activate_imu_release.py --activate /absolute/release/release_manifest.json` rechecks source/artifact hashes, writes rollback metadata, and atomically replaces only the existing installed executable symlink. Existing processes keep their old mapped binary/library inodes. The installed ROS package prefix, launch names, and parameters do not change. It never starts FAST or truncates log files.

`activate_imu_release.py --rollback /absolute/release/activation.json` restores the prior symlink atomically, and refuses to overwrite a link subsequently changed by the user.

The 3 tests in `test_activate_imu_release.py` executed in temporary directories and passed. They verify preservation of the old open inode/content through activation and rollback, refusal of a changed artifact, and refusal to overwrite a user-retargeted link. This is filesystem safety validation, not a production activation or a SLAM runtime test. Raw output is `tests.log`.

After activation, stop the prior user FAST session with Ctrl-C before launching a new native FAST session: both builds otherwise use the same source-root log filenames. The separately validated isolated runtime build uses its own ROOT_DIR; do not describe the production release as byte-identical to that isolated artifact or as having undergone an additional replay.
