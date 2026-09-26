# FAST-LIVO2 scientific system-validation harness

This directory is intentionally limited to FAST-LIVO2. It neither imports nor
runs HA_MSLAM. It uses existing local bags only: no dataset download, conversion,
repair, or metadata rewrite is performed.

## Scientific contract

The implementation exposes exactly the three modes selected by
`LIVMapper::slam_mode_`:

| Variant | `common.img_en` | `common.lidar_en` | `imu.imu_en` |
|---|---:|---:|---:|
| LO | 0 | 1 | false |
| LIO | 0 | 1 | true |
| LIVO | 1 | 1 | true |

Canonical standalone launch (one FAST node and optional RViz, no HA dependency):

```bash
ros2 launch fast_livo s1_fast_golden.launch.py mode:=lo
ros2 launch fast_livo s1_fast_golden.launch.py mode:=lio
ros2 launch fast_livo s1_fast_golden.launch.py mode:=livo
# Official dataset files, in order, followed by authoritative mode overrides:
ros2 launch fast_livo s1_fast_golden.launch.py mode:=livo \
  params_file:=/absolute/path/config/avia.yaml \
  camera_params_file:=/absolute/path/config/camera_pinhole.yaml
# Native compressed input; no image_transport republisher:
ros2 launch fast_livo s1_fast_golden.launch.py mode:=livo \
  input_mode:=compressed_direct image_topic:=/camera/image/compressed
```

`use_sim_time` and `use_rviz` default to true. Source the built FAST workspace
and pinned Livox sensor underlay; the launch validates its typesupport without
starting the driver. Raw input forces `common.enable_image_processing=false`.
Custom parameter files remain intact and mode overrides are applied last.

The existing scientific runner starts `fastlivo_mapping` directly with ordered
base/camera files and typed overrides. The manifest `launch_file` is profile
provenance, not a second execution path.

GOLDEN scientific outputs are `/aft_mapped_to_init`,
`/cloud_registered_metric`, `/lidar_measurement_information`, and the `/tf`
edge `camera_init -> aft_mapped`. Information inherits the odometry header and
body frame. The metric-cloud gate requires unique, strictly increasing cloud
stamps that match odometry stamps and frames exactly; extra odometry is allowed.
Every cell records `/tf` and requires a bijection with odometry by exact header
stamp, with no duplicate FAST-edge stamps and bit-exact float64 translation and
quaternion components. `pose_tf_contract` records this check; missing or failed
TF evidence invalidates the cell. No nearest-neighbor association is used.

The golden RViz configuration uses `camera_init`. `/cloud_registered` is a
legacy/display diagnostic; `/path` and `/rgb_img` are also display outputs,
not scientific timing truth. RGB may be empty in LO/LIO. Their existing timestamp
semantics are unchanged.

Calibration datasets are forbidden. The runner sets and verifies
`localizability.calibration.enabled:=false`. Consequently,
`/lidar_localizability_calibration` is not part of the baseline recording and
this experiment makes no claim based on that disabled telemetry. No
localizability or accuracy threshold is invented by this harness.

## Data manifest, exclusion rules and preflight

The manifest declares two distinct roots, `official` and `ugv`; every bag is
a relative path confined to exactly one root. Input and output trees may not
contain one another. Every dataset must declare `purpose: validation` and a
non-calibration data role. Calibration-labelled paths and roles are rejected
both by JSON Schema and by runtime guards.

The YAML document is validated inside the runner with
`jsonschema.Draft202012Validator` before path access or process launch. The
example contains five official sequences and `ugv_custom_fastlivo2_10`.
The three canonical UGV profiles are `config/ugv_v1_{lo,lio,livo}.yaml`
at the repository root, with byte-exact audited content. Their source paths
and SHA-256 values are recorded in
`profiles/ugv/provenance.yaml`. Before every UGV cell starts a ROS process,
the runner recomputes and fail-closed compares all three canonical hashes, then
verifies that the selected mode uses the profile recorded for that mode. The
result is retained as `profile_hash_verification` in the resolved and run
manifests. No HA_MSLAM executable, launch file, node, package or runtime
workspace is required.

Before runtime nodes are launched, the runner requires `metadata.yaml`, runs
read-only `ros2 bag info`, then executes `analyze_input.py` exactly once per
dataset into `_input_analysis/<dataset>/`. Counts, timestamp integrity,
coverage and synchronization evidence are propagated to every cell manifest.
Any failure invalidates the dataset before playback. The harness never rewrites
a bag; any approved repair must operate on a disclosed, checksummed copy.

Ordering is pre-specified by `schedule_seed`. For the three-repeat anchor, the
order is a Latin square: LO/LIO/LIVO, LIO/LIVO/LO, LIVO/LO/LIO. Single-repeat
datasets use a deterministic SHA-256-derived permutation from the recorded
seed and dataset ID. Dry-run prints the complete schedule and every command.

## Environment

Source the ROS 2 installation and the workspace containing this exact checkout:

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
source ~/Documents/fast_livo2_baseline_ws/install/setup.bash
python3 -m pip install -r test/system_validation/requirements.txt
export FAST_LIVO2_OFFICIAL_DATA_ROOT=~/Documents/datasets/fast_livo2/converted/original
export FAST_LIVO2_UGV_DATA_ROOT=~/Documents/datasets/ugv_fast_livo2
```

Do not point these variables at calibration-only bags. The example records no
dense cloud by default. `record.cloud_registered: true` preserves the historical
node-clock-stamped cloud when explicitly required. For an S2-ready capture use
`record.metric_cloud: true`: the runner enables and records
`/cloud_registered_metric`, requires its recorder subscription, and fails
postflight unless every nonempty metric cloud has one exactly matching odometry
header stamp and frame. Extra odometry-only updates are valid in LIVO.

Preview all commands and perform the same bag preflight without playback:

```bash
python3 test/system_validation/run_validation.py \
  --manifest test/system_validation/datasets.example.yaml \
  --output-root /absolute/path/to/results --dry-run
```

Run one selected cell:

```bash
python3 test/system_validation/run_validation.py \
  --manifest test/system_validation/datasets.example.yaml \
  --output-root /absolute/path/to/results \
  --dataset CBD_Building_03 --variant LO --repetition 1
```

`--repetition` is repeatable, accepts positive integers only, and filters the
pre-specified schedule without reordering the remaining cells. A selection with
no matching cell fails before runtime execution.

Playback uses `--clock --disable-keyboard-controls` and the manifest's
`playback_rate`. CBD_Building_03 uses the pre-specified three-row Latin square;
the other sequences use one seeded deterministic order. Increase repetitions
only through an amended, recorded experimental plan.

The output root is protected by a non-blocking exclusive lock. Recorder,
`/laserMapping`, required graph outputs, and recorder subscriptions to odometry
and information must become observable before playback. The effective parameter
dump is checked against the mode, sensor topics, simulated time and disabled
calibration contract. Each subprocess owns a process group; SIGINT/SIGTERM
handlers stop complete groups. Drain uses a 5 s quiet window with a conservative
120 s timeout, not a fixed sleep. Postflight bag-info, successful analysis,
non-empty `odometry.csv`, and `metrics.json` are mandatory for a VALID cell.
A post-playback shutdown crash or SIGINT-to-SIGTERM/SIGKILL escalation is
`INVALID_SHUTDOWN`, but postflight analysis is still attempted so evidence is
retained. Every stop records signals, timeout/escalation history, reason and
final code. Run-directory timestamps include UTC microseconds to avoid collision.

## Campaign aggregation and decision policy

Run the campaign aggregator **once, after the complete runner campaign has
finished** and the root `validation_summary.json` plus every available cell
`run_manifest.json`, `analysis/metrics.json` and dataset
`input_integrity.json` have been written. It is not invoked from, and must not
be rerun inside, each individual cell:

```bash
python3 test/system_validation/aggregate_campaign.py \
  --validation-summary /absolute/path/to/results/validation_summary.json \
  --policy test/system_validation/decision_policy.yaml \
  --output /absolute/path/to/results/campaign_analysis
```

The pre-specified policy keeps structural decisions separate from descriptive
measurements. The aggregation directory contains `campaign_summary.json`,
`campaign_summary.yaml`, `campaign_cells.csv`, `campaign_axes.csv`,
`campaign_repeatability.csv` and `campaign_overview.png`. The seven reported
axes are DATA_INTEGRITY, FUNCTIONAL, NUMERICAL, TEMPORAL, SHUTDOWN, REALTIME
and ACCURACY. Missing ground truth produces `ACCURACY=NOT_ESTABLISHED`;
realtime remains descriptive unless an explicit instrumented criterion exists.
The campaign verdict is restricted to the recorded datasets, modes,
repetitions, commit, parameter files and machine.

## Outputs

Each run directory contains:

- the output rosbag (odometry, LiDAR measurement information and ROS events);
- node, recorder, player, and analyzer logs;
- `resolved_manifest.yaml` and effective parameters dumped from
  `/laserMapping`;
- `run_manifest.json` with the seed/schedule, exact argv arrays, input/output
  bag-info, environment, OS/CPU/binary/package-prefix evidence, full Git status
  and diffs, tracked/untracked source SHA-256, config/input SHA-256, exits,
  crashes, termination events, elapsed time, initial/final free disk, topics,
  CPU/RSS samples and cell VALID/INVALID state;
- completed `input_analysis`, per-topic `input_counts` and `input_timing`
  evidence produced once per dataset before any playback;
- `sha256_manifest.json` covering generated artifacts;
- analysis CSV, JSON, YAML and PNG files. The analyzer consumes the provisional
  run manifest so `metrics.json` can include resource and real-time-factor data;
- Python and installed numpy/PyYAML/matplotlib/psutil/jsonschema versions
  (`N/A` when unavailable), plus ROS package prefix/executable provenance;
- root `validation_summary.json` with every cell and global VALID/INVALID state.

Dense clouds are excluded by default. The analysis requires
`/aft_mapped_to_init` and reports message counts, timestamp range, nonpositive
and maximum gaps, mean rate, finite fractions, frames, path length, maximum
step, and start/end displacement. The latter is a **drift proxy only for a
closed or near-closed trajectory**, not absolute drift.

LiDAR measurement-information is summarized using finite eigenvalues and
condition numbers where present. No localizability-calibration result is claimed
while its publisher is disabled. Plots cover trajectory, timing/displacement,
eigenvalues, residual and conditioning. JSON converts NaN/Inf to `null`; CSV
preserves raw diagnostic values.

## Optional ground truth

Without GT, the output status is `MEASURED_NO_GT`; no ATE/RPE PASS claim is
allowed. With a manifest `ground_truth.topic`, samples are timestamp-matched,
translation is rigidly aligned without scale, and translation ATE/RPE plus
rotational RPE are reported. This is descriptive output, not a threshold-based
verdict.

Analyze an existing output bag separately:

```bash
python3 test/system_validation/analyze_results.py \
  --bag /absolute/path/to/output_bag \
  --output /absolute/path/to/analysis
```

The analyzer also accepts synthetic/exported CSV inputs for offline review.
The directed tests use synthetic fixtures and compile the current publication
function against installed ROS message headers. A C++ compiler and ROS headers
are required for the native bit-exact regression (10,000 poses):

```bash
python3 -m pytest -q test/system_validation/tests
```

## Current limitations and disclosure

No bags have been executed by this change. Dataset paths in the example must be
bound to existing laptop data. ROS 2 dynamic message support and the built
`fast_livo` interfaces are required to read an output bag. CPU percent is a
sampled process-tree quantity, not machine-wide utilization. Topic rate is
derived from output message timestamps. Input per-topic counts and timing are
measured by a full read-only bag traversal, which adds substantial preflight
time. The canonical UGV profiles retain `img_time_offset: 0.0` and
`exposure_time_init: 0.0`; this is recorded provenance, not evidence that the
physical camera timing offset is zero. The 5 s/120 s drain policy is conservative
and may lengthen failed runs, but bounds the wait. Free disk is recorded before
and after every cell; no hard disk threshold is imposed in this revision.

The repository has a license/metadata mismatch that predates this harness. It
is not corrected here because licensing changes are outside the validation
scope. No source dataset has been corrected or rewritten. If any data repair is
later approved, both final PDFs must identify the original, the copy, the exact
operation, and before/after checksums.
