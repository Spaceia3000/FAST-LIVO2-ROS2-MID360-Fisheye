# HA-S0-B-FAST-RECONCILE-20260925-01

## Task contract

task_id: HA-S0-B-FAST-RECONCILE-20260925-01

### Writable repository
Spaceia3000/FAST-LIVO2-ROS2-MID360-Fisheye

Base:
integration/v1-humble @ a1f1a0b19fd8bd8df7c0a67badca45a0097433e0

Task branch:
feat/s0-fast-reconcile-20260925-01

### Read-only evidence
Local baseline workspace:
~/Documents/fast_livo2_baseline_ws/src/FAST-LIVO2-ROS2-MID360-Fisheye

Historical validated/local refs:
- feat/ugv-livo-rviz @ 096ba281a2dd18ddeff0276011c0e7f7e6035d2c
- dev/fast-livo2-scientific-validation at the same commit
- fix/system-validation-metric-cloud-contract @ 17144a6614718369a38fca65fdb4f9e7fa84c9f8
- HA integration/v1-humble is reference-only.

Do not use lsi-uc3m; it belongs to another worktree and history.

## S0-A findings to preserve

The local RViz/scientific branch diverges from canonical. Do NOT merge it wholesale.

Promotable behavior to inspect and port:
1. scientific validation harness / frozen decision-policy / fixtures and tests from the validated branch history;
2. repetition selection preserving schedule ordering;
3. process CPU-sampling state;
4. lifecycle ownership/shutdown repair at 096ba281 as one coherent change in LIVMapper.h, LIVMapper.cpp and main.cpp;
5. the three UGV LIVO assets currently untracked in the original workspace, byte-for-byte:
   - config/ugv_v1_livo.yaml
   - launch/mapping_ugv_livo.launch.py
   - rviz_cfg/fast_livo2_ugv_livo.rviz
6. inspect the separate metric-cloud validation candidate. Port it only if it is strictly a validation-contract improvement and does not alter estimator/runtime mathematics.

Rejected/guarded:
- do not port lsi-uc3m;
- do not port calibration or estimator tuning;
- do not replace canonical build flags with the local f429324 build-flag variant;
- do not treat the current three-file dirty recorder patch as sufficient if the fuller metric-cloud contract is needed.

## Frozen UGV hashes
config/ugv_v1_livo.yaml
204e244e9776388e03b07219f2dcf2fe4da394f419c77a93fcb94c2cf7bdbb10

launch/mapping_ugv_livo.launch.py
cdfbd3f5e047e1c1e3d8701214b336b1f4b963787f9a6ce2cca5b7abe778d63e

rviz_cfg/fast_livo2_ugv_livo.rviz
c6cc3c04e83ac50699c97272ce49b4ff392df413a23db7beb2a250f7a555ff9f

## Scientific invariants
- no change to ESIKF equations;
- no change to LIO/LIVO registration mathematics;
- no threshold tuning;
- no calibration changes;
- preserve /aft_mapped_to_init semantics;
- preserve /cloud_registered_metric exact sensor-time contract;
- /cloud_registered and /rgb_img remain visualization products in S0;
- preserve canonical x86 portability/build flags.

## Required validation
- inspect affected functions line-by-line before editing;
- git diff --check;
- build fast_livo in isolated build root;
- fast_livo tests;
- system-validation tests;
- UGV YAML/RViz parse and launch --show-args;
- preserve exact UGV asset hashes;
- original local workspace byte/status invariance.

No merge to integration in this job. GitHub Actions owns commit/push after tests.