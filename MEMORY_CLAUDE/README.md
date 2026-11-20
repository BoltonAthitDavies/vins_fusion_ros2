# YOLO dynamic-object filtering in VINS-Fusion — engineering record

Working notes kept for the project report. Companion file:
`orbslam3_ros2/MEMORY_CLAUDE/README.md`, which carries the shared
background — the RY-SLAM paper, the model provenance, the detector node design,
the mask-coverage measurements and the environment/install notes. **Read that one
first**; this file only records what is specific to VINS.

Date: 2026-09-03. Last updated: 2026-09-17. This package is a git submodule on branch `wil`, so these
changes live in a separate repository from `orbslam3_ros2`.


## 1. What made VINS different from ORB-SLAM3

Two things, and the second is the important one for the report.

**(a) No fork to patch.** VINS is vendored in-tree under `vins/`, so there was no
equivalent of the `thirdparty/ORB_SLAM3` second-commit dance. Everything is one
package and one `colcon build`.

**(b) Features PERSIST across frames.** This is the substantive difference.
ORB-SLAM3 re-extracts ORB features every frame, so masking *extraction* is
sufficient — a dynamic keypoint simply never gets created again. VINS propagates
features by **KLT optical flow** (`calcOpticalFlowPyrLK`) and only detects new
ones to top up the count. So masking detection alone is **not enough**: a feature
that latched onto a moving box before it was detected would ride that box for
hundreds of frames.

Filtering in VINS therefore has to do two jobs:

1. cull already-tracked points that sit on (or drift onto) a dynamic region;
2. stop `goodFeaturesToTrack` seeding new points there.


## 2. The integration point — one substitution does both jobs

VINS already had a `cv::Mat mask` in `FeatureTracker::setMask()`, used to enforce
`min_feature_distance` spacing, and it already hands that same Mat to
`goodFeaturesToTrack` as its mask argument. Upstream's own loop already treats
"mask is 0 here" as "do not keep this point":

```cpp
for (auto &it : cnt_pts_id) {
    if (mask.at<uchar>(it.second.first) == 255) {
      cur_pts.push_back(...); ids.push_back(...); track_cnt.push_back(...);
      cv::circle(mask, it.second.first, options->min_feature_distance, 0, -1);
    }
    // else: silently dropped
}
```

So **seeding `mask` from the dynamic mask instead of a blank all-255 canvas does
both halves at once** — tracked points on dynamic regions fail the `== 255` test
and are dropped from `cur_pts`/`ids`/`track_cnt`, and the same Mat then prevents
new detections there. That is the whole mechanism; the rest is plumbing.

Why this is safe: dropping points here is upstream's *established* pattern — it
already discards any point falling inside a previously drawn min-distance circle —
and `prev_pts` is rebuilt wholesale from `cur_pts` at the end of `trackImage`
(`prev_pts = cur_pts`, line ~400), so removing more points breaks no index
correspondence.

```
dynamic_detector_node.py  (lives in orbslam3_ros2, GPU)
    /cam0/image_raw ──► YOLO(weight/best.pt) ──► /orbslam3/dynamic_dets
vins_estimator.cpp
    stereoCallback ──► maskFor(t, size)   [nearest-stamp lookup, non-blocking]
                   ──► ImageData::mask
Estimator::inputImage ──► FeatureTracker::trackImage(t, img0, img1, dynamic_mask)
                      ──► setMask()  ← the substitution above
                      ──► goodFeaturesToTrack(cur_img, ..., mask)
```


## 3. One detector, both estimators

The detector is a standalone node publishing a generic
`vision_msgs/Detection2DArray`, so **ORB-SLAM3 and VINS can run off the same
detector simultaneously** — no duplicate GPU cost, and byte-identical masks for
both, which is exactly what a fair head-to-head comparison wants.

Deliberately **no launch file was added to this package**: doing so would couple a
git submodule to a workspace sibling (`orbslam3_ros2`) across a repo boundary, and
this package's own README already documents running VINS via `ros2 run`. The
detector is started separately.


## 4. Changes (262 insertions, 9 files)

| file | change |
|---|---|
| `vins/src/featureTracker/feature_tracker.cpp` | `setMask()` seeds from the dynamic mask (with size/type validation); `trackImage()` latches it; counts `dynamic_dropped` |
| `vins/include/vins/featureTracker/feature_tracker.h` | `dynamic_mask` member, `dynamic_dropped` counter, new `trackImage` parameter |
| `vins/include/vins/common/sensor_data_type.h` | `ImageData::mask`, cloned/moved in all four copy/move paths |
| `vins/src/estimator/estimator.cpp` | `inputImage` forwards `image.mask` |
| `vins/include/vins/estimator/estimator.h` | `dynamicDroppedPoints()` accessor (`featureTracker` is private) |
| `src/vins_estimator.cpp`, `include/vins_fusion_ros2/vins_estimator.h` | `filter` param, detection subscription + ring buffer, `maskFor()`, periodic report |
| `CMakeLists.txt`, `package.xml` | `vision_msgs` |

Node parameters (same names and defaults as the ORB-SLAM3 node, deliberately, so
one set of numbers describes both systems): `filter` (false),
`dynamic_dets_topic`, `mask_dilate_px` (8), `det_max_age` (0.15 s),
`max_mask_fraction` (0.8 — see the ORB-SLAM3 record for how that was measured).

The mask is applied to **image0 (left) only**, matching ORB-SLAM3.


## 5. Verification results

Bag: `dataset/dynamic_dataset`, replayed at 0.7× with two `image_transport
republish` nodes converting compressed → raw (VINS subscribes to raw
`sensor_msgs/Image`, the bag carries only `/camN/image_raw/compressed`).

```
filter:=true
  dynamic filter: 890 frames masked, 2 with no recent detection,
                  2461 tracked points dropped onto dynamic regions,
                  0 masks discarded for exceeding max_mask_fraction

filter:=false
  "filter: off (stock VINS feature tracking)"
  no subscription created, zero filter reports
```

**2461 tracked points dropped** is the number that proves point (b) from §1 is
actually being handled — those are KLT-propagated features culled because they
landed on a dynamic region, which the ORB-SLAM3 approach would never have needed
to count.

Two incidental findings:

* The Ceres `Terminating: Initial residual and Jacobian evaluation failed` spam
  appears **671 times in the baseline run too**. It is pre-existing VINS behaviour
  on this bag, **not** caused by the filtering. Do not attribute it in the report.
* Filter statistics are reported on a **periodic wall timer**, not in the
  destructor. This node's executor does not reliably return from `spin()` on
  SIGINT, so a shutdown-only report is never seen on a bag replay — the destructor
  log was written first and never fired once.


## 6. ATE numbers — measured, but NOT valid

For the record so they are not re-derived and mistakenly trusted:

```
VINS filter=OFF : ATE RMSE 6.431 m  (670 poses,  13 s ground-truth overlap)
VINS filter=ON  : ATE RMSE 4.880 m  (1568 poses, 13 s ground-truth overlap)
```

**These are not a controlled comparison and must not go in the report as a
result.** The two runs produced 670 vs 1568 poses over the *same* 13 s overlap
window, and both ran with the detector, two republishers and VINS contending for
one machine at 0.7× replay. The pose-count asymmetry alone invalidates the
comparison.

For a real result: run each configuration alone, at a replay rate slow enough that
nothing drops, and score with `script/vio_metrics.py` against
`output/output_vins/simulation/dynamic_dataset/ground_truth.csv`.

The 6.4 m baseline is consistent with the known finding that the dynamic warehouse
run diverges.

**The semantic-prior caveat applies identically here** — see §7 and §8 of the
ORB-SLAM3 record. The filter masks by CLASS, with no test of whether an object is
actually moving, and nothing in either pipeline is 3D-aware.

Updated 2026-09-11 after measuring the detector against Gazebo ground truth
(`script/yolo_eval.py`): over-masking is a much smaller problem than first thought —
the mask covers ~11.4% of the frame at 8 px dilation, of which ~8.4% is genuinely
moving, so a perfect motion gate would recover only ~3 points of frame area. The
dominant limitation is instead the detector's **recall on moving props, 0.599**:
both systems are currently masking barely 60% of the objects the method depends on
finding. Full numbers and the two ground-truth traps are in the ORB-SLAM3 record §8.


## 7. How to run

```bash
# republish compressed -> raw (VINS needs raw Image)
ros2 run image_transport republish compressed raw --ros-args \
  -r in/compressed:=/cam0/image_raw/compressed -r out:=/cam0/image_raw \
  -p use_sim_time:=true
# ...and the same for cam1

# detector (from orbslam3_ros2 — shared by both estimators)
ros2 run orbslam3_ros2 dynamic_detector_node.py --ros-args \
  -p image_transport:=compressed -p use_sim_time:=true

# VINS with filtering
ros2 run vins_fusion_ros2 vins_fusion_ros2_node --ros-args \
  -p config_file:=$PWD/vins_fusion_ros2/config/wil_sim/stereo_imu.yaml \
  -p use_sim_time:=true -p filter:=true -p output_path:=<out>

ros2 bag play dataset/dynamic_dataset --clock -r 0.7
```

Build from the **workspace root** (`colcon build --packages-select
vins_fusion_ros2 --symlink-install`); running it from inside the package creates
stray `build|install|log` dirs. Use `--parallel-workers 2` — a full-parallel build
was killed by the OOM killer on this 15 GB box.


## 8. Structured evaluation logging (implemented 2026-09-16)

Passing the ROS parameter `output_path` now creates that directory and all missing
parents automatically. It overrides the YAML output location, writes the trajectory
to `<output_path>/vio.csv`, and uses `<output_path>/pose_graph/` as the default pose
graph location. `pose_graph_save_path` can override the latter independently.

Each completed raw run can contain:

| file | evidence recorded |
|---|---|
| `run_metadata.csv` | pipeline, dataset/world paths, command notes, topics, frames, estimator settings and replay-rate label |
| `frontend.csv` | KLT tracking time, mono/stereo feature counts, enqueue/backlog state and dynamic points removed |
| `backend.csv` | initialization and marginalization state, stage timings, feature statistics, Ceres problem/solver statistics, bias norms and speed |
| `estimator_state.csv` | per-frame position, orientation, velocity, IMU biases and time delay |
| `performance.csv` | end-to-end processing, input/use counts, backlog, resets, CPU time and resident memory |
| `events.csv` | startup, initialization, reset reason and graceful shutdown events |
| `run_summary.csv` | run totals, initialization time, keyframes, resets, solver calls and outliers |
| `filter_summary.csv` | whether filtering ran and its aggregate detection/mask/point-removal counts |
| `yolo_mask.csv` | per-frame boxes, mask coverage, match age and rejected masks; created only when `filter:=true` |
| `vio.csv` | estimated trajectory in the common comparison format |

This is sufficient to analyse the VINS **frontend and sliding-window backend** as
well as resource pressure and failure/reset behaviour. It does not provide
loop-closure statistics because this repository contains no active `loop_fusion`
component; this implementation is VIO rather than a complete loop-closing SLAM
system. Do not present an absent loop-closure log as a zero loop-closure result.

`replay_rate` and `world_path` in `run_metadata.csv` are provenance supplied by the
launch command. They do not control rosbag playback or load a navigation map:
`replay_rate` must match the actual `ros2 bag play --rate`, while `world_path`
identifies the Gazebo world used to record the dataset. Stop the node gracefully
where possible so summaries and buffered output are finalized.

### Dataset-path provenance correction (2026-09-17)

Dataset identity is determined by the containing dataset folder name, **not by old
metadata**. The canonical storage locations for the relogged experiment are:

* local: `dataset_dynamic_nofloortexture_01_000` and
  `dataset_static_nofloortexture_001` under
  `/home/ambushee/wil_project/dataset/`;
* removable drive: every other `dataset_*_nofloortexture_*` used by this experiment
  under `/media/ambushee/32E4AAB1E4AA772F/dataset/`.

On 2026-09-17, all 66 `run_metadata.csv` files under `output/output_*` were audited
against this rule. Fifteen incorrect `dataset_path` values were corrected (12 that
incorrectly pointed local datasets to the drive and 3 that pointed
`dataset_dynamic_nofloortexture_02_000` to a nonexistent local directory). All 66
resolved paths existed after correction; trajectory and measurement files were not
changed.

## 9. Precomputed detections — IMPLEMENTED, THEN REMOVED (2026-10-12 to 10-16)

> **REMOVED 2026-10-16**, see ORB record §12 for the reason: Experiment 3 is a
> deployment evaluation at replay rate 1.0, so the live detector is part of the
> system under test. Diff preserved at
> `MEMORY_CLAUDE/precompute_and_fixes_2026-10-16.patch`.
>
> KEPT: `filter_status` in `filter_summary.csv`, the 20 s watchdog that ERRORs
> when nothing publishes the detection topic, and the
> `dynamic_points_dropped` / `mindistance_points_dropped` split. Those exist
> because of the 2026-09-16 failure recorded below, which the live path can
> still reproduce.

### Original section (historical)


Same change as ORB-SLAM3 §12 (read that first: the lag mechanism, the 33 % vs
91 % mask availability on two identical runs, and the stale-republisher trap are
all documented there). VINS gained the identical parameter:

- `dets_csv` (string, default empty). With `filter=true` and a file from
  `script/yolo_precompute.py`, no detection subscription is created and
  `maskFor()` looks each frame up by its exact header stamp (nearest within
  `det_max_age` as a fallback, reported as a nonzero age). `loadDetsCsv()`,
  `stampNs()` and the file branch mirror the ORB node line for line.
- `filter_summary.csv` gains `detection_match_rate` and `detection_source`
  (`none|live|file`); the 10 s wall-timer report now also warns when the match
  rate drops below 98 %.

Verification, `realistic_003`, `--rate 0.3`, 120 s wall, one fresh republisher
pair, no stale processes:

```
yolo_mask.csv        676 frames, matched 676, det_age_ms 0.0 on every row
filter_summary.csv   detection_match_rate 1.0000, detection_source file,
                     dynamic_points_dropped 3360, masks_rejected 0
Ceres "Uninitialized" lines   0
```

The first attempt at this check printed 1207 Ceres `Uninitialized` errors and
every stamp three times. That was the harness, not VINS: two stale republishers
from an earlier session plus the test's own pair delivered each frame twice,
and `kill -9` on the `ros2 run` wrapper left the node binary alive to consume
two further bag replays. VINS fed duplicate stamps (dt = 0) is what produces
the Ceres spam. Kill by binary name (`pkill -f vins_fusion_ros2_nod[e]`,
`pkill -f lib/image_transport/republis[h]`), never the wrapper.

**What is and is not proven for VINS.** Mask determinism per frame is proven
(file lookup, age 0, no detector). Frame-set determinism is NOT: VINS subscribes
to raw `sensor_msgs/Image`, so its frames still arrive through a republisher
pair over best-effort DDS — the same path ORB-SLAM3 §12.3 showed to lose frames
nondeterministically. 676 frames in a 36 s window (~1080 published) is partly
`image_skip` and partly that path. A two-run frame-set diff like ORB's has not
been done for VINS; do it before quoting a VINS ON/OFF comparison, and if it
fails, the fix is a reliable-QoS subscription or decoding compressed in the
node as ORB does.

## 10. Motion-gated masking — `filter_mode:=motion` (2026-10-12/14)

Same policy, parameters, logs and calibration script as ORB-SLAM3 §13 — read
that first. Only the census differs, because VINS has no per-frame
pose-optimisation inlier flag:

- `ImageData::vote_boxes` travels with the frame; `Estimator::inputImage`
  pushes it in lockstep with `featureBuffer`, and `processMeasurements` runs
  `censusBoxes()` right after `processImage()` on the backend thread — the only
  moment the sliding-window feature list and this frame's observations are
  both current.
- A feature is an **inlier** when the window holds a SOLVED, positive depth for
  it (`FeaturePerId::isSolved()`). Features on a mover keep being purged by
  `outliersRejection` (reprojection error > 3 px over the window) and re-enter
  as unsolved young tracks, so their box's ratio falls below the background's.
  This is the VINS analogue of ORB's keyframe-anchored map point.
- Finished censuses queue in `censusOut_`; the node drains them with
  `takeBoxCensus()` at the start of `maskFor()` (`applyPendingVotes()`), so the
  verdict lags by the backend queue depth, typically one frame.

Not used, deliberately: `predictPtsInNextFrame()` / `setPrediction()` would give
a direct predicted-vs-tracked residual per feature, but this fork never calls
it, and enabling it changes KLT initialisation (`OPTFLOW_USE_INITIAL_FLOW`, one
pyramid level) for the baseline too — it would contaminate the ON/OFF A/B.

Verification is recorded below once the first motion run on `realistic_003`
completes.

### Verification — `realistic_003`, 2026-10-14 (`motion_20261014_01`)

`dets_runs1.csv` (4-class), rate 0.3, `motion_min_points 15`, re-vote 15.
1720 frames, match rate 1.0, 0 Ceres errors, 1518 tracked points culled.

```
yolo_mask.csv     292 of 1720 frames painted; box-frames moving 522, static 1916, unknown 5621
truth / verdict   unknown   static   moving
moving                556        2      181
static               4106     1865      321
none                  927       68       31
census, voted rows (n_kp >= 15): moving n=15, static n=1319
  box/bg inlier ratio   moving: p25 0.00 median 0.00 p90 0.48
                        static: p10 0.79 median 1.15 p90 1.98
sweep (recall / false alarm):
  min_pts  8: ratio 0.25 0.63/0.027   0.50 0.75/0.041   0.75 0.89/0.098
  min_pts 15: ratio 0.25 0.73/0.005   0.50 0.87/0.011   0.75 1.00/0.076
```

The census is strongly discriminative: a truly-moving box's solved-depth ratio
is essentially zero (the mover's features never keep a consistent depth), a
static box's sits at the background rate. At the run's parameters the per-vote
recall is 0.87 for 1.1 % false alarm.

The limiting factor is **coverage, not discrimination**: VINS tracks ~150–200
features per frame, so only 1346 of 8057 box-frames reached 15 keypoints and
70 % of box-frames stayed `unknown` (never scored, no chain to inherit from).
Lowering `motion_min_points` to 8 quadruples the moving sample (15→73) at
recall 0.75 / false alarm 4.1 %. The 321 static box-frames masked are mostly
inherited from the few false votes and bounded by the re-vote probation.
Only 15 truly-moving voted rows is a small sample; `realistic_002` (pallet
jacks travelling 19 m) is the bag to confirm on.

### Run 2 — `motion_20261014_04`, + `motion_min_box_age 30` (current defaults)

```
yolo_mask.csv     219 of 1719 frames painted; box-frames moving 394, static 1724, unknown 5931
truth / verdict   unknown   static   moving
moving                569        7      163     <- 22 % of moving box-frames masked
static               4373     1697      218     <-  3.5 % of static box-frames masked
census, voted rows (n_kp >= 15, box_age >= 30): moving n=16, static n=1295
  box/bg inlier ratio   moving: median 0.00 p75 0.92     static: p10 0.82 median 1.15
sweep: min_pts 15 ratio 0.50 -> recall 0.62 / false 0.8 %;  min_pts 8 -> 0.57 / 1.9 %
```

The gate is clean — under 1 % false alarm per vote, 3.5 % of static
box-frames masked overall — and starved: 74 % of box-frames are never judged
because VINS keeps ~150–200 features per frame and a box rarely holds 15 of
them for a second. Only 16 truly-moving voted rows, so the recall figure is a
small-sample estimate. Levers if resumed: `motion_min_points 8` (coverage ×3
at 1.9 % false alarm), `max_cnt` in the VINS config (more features per frame),
and `realistic_002` for a larger moving sample.

## 11. The motion census does not work, and why (2026-10-15) — NEGATIVE RESULT

Everything in §10 is implemented and runs. On `dataset_dynamic_nofloortexture_00_000`
it does not detect motion. Four diagnostics, in order, each ruling out the
explanation the previous one suggested. **Read this before attempting a fifth.**

Ground truth throughout is `script/motion_votes_eval.py`, which projects the
props into cam0 and labels each voted box moving/static from `pose/info` speed.
On this bag **64 % of box-frames sit on genuinely moving objects**, so the
scene is not the problem.

### 11.1 The headline

| census | signal | result |
|---|---|---|
| solved depth (§10) | feature holds a solved, positive depth in the window | moving and static indistinguishable |
| reprojection residual | median `ave_err × FOCAL_LENGTH` per box vs background | **inverted** — movers score LOWER |
| residual availability | fraction of a box's features that have a residual at all | recall 0.56 at 41 % false alarm |

### 11.2 Solved depth: measures depth observability, not motion

At the shipped defaults, against truth:

```
truth / verdict   unknown   static   moving
moving               1558      180      216     <- catches 11 % of truly-moving box-frames
static                605       54       56
voted rows:  moving n=84  median ratio 1.66     static n=26  median 1.93
             recall 0.250   false alarm 0.231   (chance)
```

Starvation was suspected first — the median box held **3** tracked features of
the 150 VINS keeps. Two config experiments settled it:

- `max_cnt 150 → 250`: **no effect**. `feature_count` went 150 → 147; the scene
  cannot supply more corners ≥30 px apart. Per-box distribution identical
  (median 3, p75 6, p90 16), `processing_ms` 55.7 → 57.2. The ceiling is
  `min_dist`, not `max_cnt`.
- `min_dist 30 → 15` (with `max_cnt 250`): features per box 3 → **12**, coverage
  19.6 % → 55.8 %. And the signal *vanished*: **94.7 %** of features inside
  moving objects held solved depths, box/background ratio min **0.96**, median
  2.03, and **zero** of 83 voted rows fell below the threshold.

More data made it worse, which is the signature of a quantity that was never
measuring the right thing. A moving point triangulates perfectly well — it
converges to the WRONG depth, and `isSolved()` cannot tell the difference.

### 11.3 Reprojection residual: confounded by survivorship

`outliersRejection()` already computes each feature's average reprojection
residual against the optimised window, uses it as a >3 px cutoff, and discards
it. Capturing it before the purge looked like the obvious fix. Measured:

```
box/bg residual   moving n=31  median 0.57      static n=37  median 1.22
sweep (moving if ABOVE):  1.5 -> recall 0.226 / false 0.270
background median residual 1.915 px;  n_resid median 1
```

Movers score **lower**. `outliersRejection` purges >3 px features every frame,
so a mover's bad features are gone within a frame or two and only the ones that
happened to fit remain to be measured. Inverting the rule does not rescue it:
31 moving samples, and the mechanism is a selection artifact, not a property of
motion. The same effect caps coverage — a residual needs `used_num >= 4` and a
mover's features rarely survive that long.

### 11.4 Residual availability: also no

If movers lose features before they reach four observations, the *fraction* of a
box's features that have a residual should itself be the signal. It is not:
`n_resid/n_kp` medians 0.14 (moving) vs 0.27 (static), sweep gives recall 0.56
at **41 %** false alarm.

### 11.5 The common cause

All three read the BACKEND: the sliding window after optimisation and outlier
rejection. VINS is *designed* to be robust to moving points — it purges anything
over 3 px every frame and `featureManager` keeps only survivors. **By the time
the window has an opinion about a feature, the features that disagreed with it
are gone.** Asking the filter that removes movers to report where the movers are
cannot work, whichever statistic is used.

### 11.6 A trap in comparing runs

The residual run's solved-depth block scored recall 0.738 (0.930 at
`min_points 15`) against 0.250 for the earlier run at the *same* `max_cnt` and
`min_dist`. **That is not an improvement.** The two runs used different decision
rules, so they masked different boxes, so the surviving feature population
differed, so the census differed. A census computed downstream of its own
masking is not comparable across policies. Two earlier figures were quoted in
this record before ground truth saw them and did not survive more data
(`realistic_003` "recall 0.62 / 0.8 %" rested on 16 samples); do not quote a
threshold or a rate until `motion_votes_eval.py` has scored it.

### 11.7 If it is resumed: use the frontend

The signal must be computed BEFORE rejection. Three things already exist and are
unused:

- **`FeatureTracker::rejectWithF()`** — written, and commented out at
  `vins/src/featureTracker/feature_tracker.cpp:297`. RANSAC fundamental matrix
  between consecutive frames; the outliers it marks are points violating
  epipolar geometry, i.e. points that moved. Pre-rejection, every tracked point,
  every frame. Cheapest honest attempt remaining.
- **`Estimator::predictPtsInNextFrame()` / `FeatureTracker::setPrediction()`** —
  implemented at `vins/src/estimator/estimator.cpp:1807`, never called.
  Predicted-vs-tracked displacement per feature. Must be computed WITHOUT
  calling `setPrediction`, which changes KLT initialisation
  (`OPTFLOW_USE_INITIAL_FLOW`, one pyramid level) and would contaminate the
  baseline.
- **`ptsVelocity()`** — per-point image velocity, computed every frame and
  already carried in the feature message as `vx, vy`. Ego-motion is a smooth
  field; a mover is a coherent local anomaly in it. No new computation.

ORB-SLAM3's gate is also weak (record §13: recall 0.43 at 5 % false alarm), so
this is not a VINS-specific verdict on motion gating as an idea.

### 11.8 Status

The §10 implementation is reverted to the solved-depth census and remains in the
tree, off by default (`filter_mode: class`). Experiment 3 runs on `class` mode,
which is the RY-SLAM method the thesis tests; motion gating belongs in Chapter 5
as attempted-and-failed, with the measurements above as its evidence.

Kept from the same day and NOT part of this rewrite: `dets_csv` auto-discovery,
`filter_status`, the detector watchdog, and the
`dynamic_points_dropped`/`mindistance_points_dropped` split.


## 12. One-command launch — `launch/wil_sim_vins_filter.launch.py` (2026-10-16)

A filter run took four terminals in the right order, and forgetting the detector
is SILENT -- the node subscribes, nothing publishes, every frame records
matched=0. That is the 2026-09-16 failure (§9) and it recurred on 2026-10-15/16
once the precompute shortcut was removed.

**Cross-package reference, deliberate.** `dynamic_detector_node.py` lives in the
workspace sibling `orbslam3_ros2`, and this package is a submodule. That is why
it shipped no WiL launch file for so long, and why this one first lived in
`script/` and was launched by absolute path. Moved into `launch/` on 2026-10-16
at the user's request, so it is reachable as `ros2 launch vins_fusion_ros2 ...`
exactly like the ORB one. `orbslam3_ros2` is referenced BY PACKAGE NAME,
resolved through the ament index at launch time, and is deliberately NOT
declared in package.xml: `colcon build` of this submodule alone still succeeds,
and only `filter:=true` needs the sibling present. With `filter:=false` there is
no cross-package reference at all. If this submodule is ever reused outside this
workspace, this is the one file to delete.

```bash
ros2 launch vins_fusion_ros2 wil_sim_vins_filter.launch.py \
  output_path:=.../output_vins_yolo/simulation/<bag>/<run-id> \
  experiment_id:=<run-id> dataset_path:=<bag> world_path:=<world>
# bag in one other terminal:  ros2 bag play <bag> --rate 1.0
```

It starts the detector (only when `filter:=true`) and the estimator — so both
are up before playback rather than the detector joining halfway through. Since
§13 both read the compressed topics directly, so no republish helpers are
started; `republish:=true` restores them for a raw-only source. `filter:=false` gives the baseline
arm with no detector process at all, which is the correct control for a
deployment comparison. Free-text parameters are pinned with
`ParameterValue(..., value_type=str)`; without that a `run_notes` containing a
colon-space is YAML-parsed into a dict and aborts the launch.

`replay_rate` remains metadata only — nothing enforces it, and a wrong value
survived twelve September manifests. Verify it afterwards from the IMU arrival
rate in `performance.csv` against the bag's nominal rate.

## 13. In-process compressed images — no republish helpers (2026-10-16)

VINS subscribed only to raw `sensor_msgs/Image`, but every bag in this project
carries **only** `/camN/image_raw/compressed`. The gap was bridged by two
`image_transport republish` processes, which cost three documented failures:

- **loss**: raw 1280x720 rgb8 is 2.7 MB per message over best-effort DDS. Two
  identical ORB runs received 919 and 813 frames of the same window with
  `queue_dropped = 0` -- 394 stamps present in only one run;
- **duplication**: two republisher pairs on one topic deliver every frame twice.
  VINS fed duplicate stamps (dt = 0) emits Ceres "Uninitialized" errors; one run
  logged 1 207 of them and every census row three times;
- **staleness**: a pair left running from an earlier session silently feeds
  later runs. Two from 13:34 were still alive at 16:25 on 2026-10-12 and had
  fed every ORB run in between.

`image_transport` parameter, `raw` (default) or `compressed`. With `compressed`
the node subscribes to `<image0_topic>/compressed` through a second
`ApproximateTime` sync over `CompressedImage` and decodes with `cv::imdecode`
straight to **grayscale** -- not through image_transport's plugin, which on the
real rig's `"yuv422; jpeg compressed mono8"` format decodes the mono8 payload
but stamps the output yuv422, so cv_bridge rejects every frame. orbslam3_ros2
reads the same blobs the same way (its record §12.3).

Both stereo callbacks now share `submitStereo()`; a pair where either image
fails to decode is dropped whole, since a half-decoded pair is worse than a
missing one. `decode_failures_` counts them.

`launch/wil_sim_vins_filter.launch.py` defaults to `image_transport:=compressed`
and starts **no** republishers. `republish:=true` restores them for a raw-only
source. The configured `image0_topic`/`image1_topic` stay suffix-free in the
YAML; `/compressed` is appended by the node, so one config serves both
transports.

## 2026-11-03 — real-rig IMU noise ×10 (acc_n 1.0, gyr_n 0.1) in config/wil and wil_311026_selfcal stereo_imu.yaml

Mirrors the ORB-SLAM3 real-rig setting found the same day on dataset_realv6_000: with the copied densities (0.1 / 0.01) ORB-SLAM3's inertial fusion ran away in most runs (1.1–9.6 m) while stereo-only tracked at 0.63 m; with ×10 three consecutive ORB runs scored 0.47–0.49 m. Walks unchanged, previous lines kept as comments. NOT changed here: g_norm is still 9.2642 (the August IMU); the current IMU reads 9.88 m/s² at rest on realv5/realv6 and VINS diverged on realv6 with its accelerometer bias frozen at the 0.6 m/s² mismatch — fix g_norm before re-running VINS on real bags. RTAB-Map has no IMU noise parameters.

> **2026-11-03:** every bag was rescued from the flash drive to `/media/ambushee/New Volume/wil_rescue/` (rsync, `rescue.log` there); paths to `/media/ambushee/32E4AAB1E4AA772F/` in this record mean that root now.

## 15. Real-rig divergence ROOT-CAUSED: fisheye features beyond ~77 deg of field angle (2026-11-15)

Every real-rig VINS run to date diverged (realv6 217 resets; realv7_000/001/002 to 5-37 km).
The pasted-in analysis blamed the stale `g_norm` plus 40-45 % frame delivery. Both were
real but neither was the cause: with `g_norm` corrected the runs still diverged, and a
**stereo-only run (imu:0) diverged too** (`realv7_001/stereo_only`, position 1e8 m), so the
inertial side was exonerated. Chain of evidence, all on `dataset_realv7_001`:

1. **Static phase is fine with the extrinsics fixed.** With `estimate_extrinsic:1` the
   estimator drifted while the robot stood still (GT 0.00 m/s, parallax 0.1-0.3 px): both
   camera blocks are freed once the OLDEST window frame exceeds 0.2 m/s, which a false
   velocity reached within a second. `estimate_extrinsic:0` held the pose at 0.00 for
   the whole 4.5 s static phase. Extrinsics are now logged per frame (`extrinsics.csv`).
2. **Failure is at motion onset and is a backend purge, not lost tracking.** At 7.27 s
   (first keyframe after the robot rolls) 98 of 151 features are rejected in ONE frame,
   146 at 7.53 s, each right after the solve pushed the accel bias to 2-3 m/s^2. Offline,
   the node's exact KLT recipe keeps 96-100 % of corners at 30/15/7.5 Hz through the
   same frames; no blur (Laplacian var 540-650); frames arrived at the full 33 ms.
3. **Stereo-only diverges identically at 7.5 s** with `last_average_parallax` = 6740
   (normalised units x460) at that keyframe. That number is only possible for features
   with |x/z| of order 10: field angle > 84 deg.
4. **The lens.** `cam0_fisheye.yaml` (mu 607.5, k2..k5): r(60/75/77/80/85/90 deg) =
   610/742/758/782/819/853 px from the principal point (958, 540). The lens circle on the
   frames reaches ~900 px (mean grey 114 to 900 px, 21 beyond), i.e. ~100 deg, and the
   frame's left/right edges sit at 960 px. The stock detector (150 corners, min_dist 30)
   puts 11-35 % of corners beyond 77 deg, 4-21 % beyond 85 deg, 1-11 % AT/BEYOND 90 deg,
   and 1-15 % on the bright-to-black lens rim. VINS maps every lifted ray to the
   normalised plane as (x/z, y/z) (`feature_tracker.cpp` undistortedPts, ~L509/L579):
   |x/z| -> inf at 90 deg and flips sign beyond. ORB-SLAM3's KannalaBrandt8 never divides
   by z, which is why ORB tracks this rig at 0.17-0.63 m and VINS never did. Upstream
   VINS handles wide lenses with a static `fisheye_mask`; this port's member of that name
   was never loaded.

**Fix: node parameter `fov_mask_deg` (default 0 = off).** Builds a CV_8UC1 mask once from
the cam0 camera model (`CameraFactory` -> `spaceToPlane` of the cone, so exact for KB,
mu != mv) and ANDs it into `ImageData::mask`, i.e. through the same `setMask()` seeding
the YOLO filter uses: tracked points that drift past the limit are culled and no new
corners are seeded there. Launch arg of the same name in both WiL launch files;
recorded in `run_metadata.csv`. At 70 deg, 65 % of the frame is kept.

**Result, realv7_001, rate 1.0, scored with eval_real_gt.py (wheel offset -3.766 s,
r 0.98, lever arm 0, so "provisional"):**

| run | config | resets | ATE RMSE | median | RPE 1 s | path est/GT |
|---|---|---|---|---|---|---|
| stereo_only | stereo.yaml, no mask | 0 | diverged (1e8 m) | | | |
| stereo_only_fov70 | stereo.yaml, 70 deg | 0 | 0.846 m | 0.772 | 0.096 | 1.14 |
| imuz180_fov70 | stereo_imu + R=diag(-1,-1,1), extr/td FREE, 70 deg | 0 | 1.036 m | 0.859 | 0.097 | 0.75 |
| stock_fov70 | stereo_imu (R=diag(1,-1,-1)), extr/td FREE, 70 deg | 0 | 1.307 m | 1.136 | 0.149 | 0.67 |
| extr0_td0_fov70 | stereo_imu, extr+td FIXED, 70 deg | 0 | 0.739 m | 0.724 | 0.091 | 1.13 |
| imuz180_extr0_td0_fov70 | both of the above | 0 | 0.733 m | 0.712 | 0.090 | 1.13 |
| skip1_fov70 | + `image_skip` 1 | 0 | 0.713 m | 0.700 | 0.090 | 1.13 |
| accn01_fov70 | + `acc_n` 0.1 / `gyr_n` 0.01 | 0 | 0.686 m | 0.677 | 0.090 | 1.13 |
| **settled** | **config/wil/stereo_imu.yaml as edited 17 Nov** | 0 | **0.678 m** | 0.676 | 0.090 | 1.13 |

The mask alone turns every arm from divergent to completed.

**`estimate_extrinsic:1` (and `estimate_td:1`) is the second real defect, not the IMU.**
The first three rows were all run with both free, and on that evidence (16 Nov) this
record wrongly concluded "the inertial arms are worse than stereo-only, so the IMU has a
remaining defect". Holding the calibration fixed reverses it: 1.31/1.04 -> 0.739/0.733 m,
path ratio 0.67/0.75 -> 1.13, and stereo-inertial now BEATS stereo-only (0.846 m). The
free-extrinsic runs' under-scaling was the optimiser eating the baseline, which point 1
above had already shown it does within a second of a start.

**`config/wil/stereo_imu.yaml` now CARRIES the settled values** (17 Nov): `estimate_extrinsic 0`,
`estimate_td 0`, `body_T_cam0` R = `diag(-1,-1,1)` with cam1 re-derived (stereo relative
bit-identical, baseline 0.091710 m), `acc_n 0.1` / `gyr_n 0.01`, `image_skip 2`. The file
itself was then run through `wil_vins_filter.launch.py` with `fov_mask_deg:=70` and scores
**0.678 m** (run dir `settled`), against ORB-SLAM3 LC-GPU 0.450 m on the same bag. Its
header opens with a block stating that `fov_mask_deg:=70` must be passed, since that is a
node parameter and omitting it silently voids the run.

**`acc_n`/`gyr_n`: the x10 inflation of 3 Nov is reverted.** It was a workaround for
ORB-SLAM3's runaway, whose cause was different; 1.0/0.1 gives 0.733 m, 0.1/0.01 gives
0.686 m. Random walks never A/B'd.

**The mask angle now lives in the CONFIG FILE, not the launch command (17 Nov).** It was
first a ROS parameter only, and the same afternoon a real-rig run launched without the
argument diverged exactly as the pre-fix runs did -- a value that must never be forgotten
does not belong in an optional argument. `fov_mask_deg` is now a key read by `VINSOptions`
(absent = 0, so the pinhole sim configs are untouched); `config/wil/stereo_imu.yaml` and
`config/wil/stereo.yaml` carry 70.0. The ROS parameter still overrides the file and warns
when it disagrees, and the node logs a line at startup when it is running unmasked. BOTH
WiL launch files default the argument to **-1** = "take it from the config"; their earlier
default of 0.0 silently overrode the file and is what cost that run, so do not reinstate it.
Verified by replaying the user's command verbatim with no mask argument: 0.708 m ATE
(`rate1_003`), against 0.678 m for the same config run with the argument given explicitly
(`settled`) -- a 0.030 m spread over two identical-input runs, which is this estimator's own
repeatability and the floor below which no config comparison here means anything.

**Stereo calibration generation: the 3 Nov one is right, and it is not close.** Only
`camera_parameters_101126/` (10 Nov) still exists on disk beside it; run under the settled
settings it gives **72.8 m** ATE and a path 60x ground truth (`cal1110`), against 0.678 m for
the 3 Nov calibration in the live file. Worth noting WHY this was never caught: with
`estimate_extrinsic:1` the optimiser absorbed calibration error, so the four generations were
indistinguishable and untested. Fixing the extrinsics is what makes the calibration choice
matter -- a wrong one is now fatal rather than invisible, so re-score any future calibration
on a bag before adopting it.

**`image_skip`: stays 2, and 1 is a trap.** At 1 the ATE looked better (0.713 vs 0.733 m)
but that is inside VINS's own 3-7 % spread, while the backend queue wait went 7.6 -> 18.5 ms
mean and 207 -> 511 ms max and **15 % FEWER pairs reached the node at all** (971 vs 1147):
the image callback falls behind and frames are lost before the tracker sees them,
nondeterministically. The key's comment used to claim "every frame -> deterministic" while
the value was 2 (copied from `config/wil_sim/`, where it really is 1); both measurements are
now in the comment. Revisit on faster hardware.

70 deg was the first value tried, not a sweep; one bag; lever arm still zero, so every
number above is "provisional" in the scorer's sense.

### 15.0 The whole real campaign, re-run with the settled config (17 Nov)

User-run, `config/wil/stereo_imu.yaml` as it now stands (mask from the file, no launch
argument), rate 1.0, scored by `script/eval_real_gt.py` against the tracker; lever arm still
zero so all VINS rows are "provisional". ORB-SLAM3 column is its loop-closing GPU arm
(`*_lc_gpu_p15`) on the same bags, for scale.

| bag | VINS run | resets | VINS ATE | median | RPE 1 s | path | ORB-SLAM3 |
|---|---|---|---|---|---|---|---|
| realv6_000 | rate1_002 | 0 | 0.786 m | 0.518 | 0.048 | 0.82 | 0.478 m |
| realv7_000 | rate1_002 | 0 | 0.464 m | 0.434 | 0.064 | 1.10 | 0.268 m |
| realv7_001 | settled / rate1_003 | 0 | 0.678 / 0.708 m | 0.676 | 0.090 | 1.13 | 0.450 m |
| realv7_002 | rate1_002 | 0 | 0.374 m | 0.334 | 0.055 | 1.11 | 0.172 m |

**Every real bag now tracks.** realv6 went from 217 resets and no usable trajectory to zero
resets and 0.786 m. VINS is consistently 1.7-2.2x ORB-SLAM3's error on this rig and runs
8-13 % long on path, but it is in the same class and the failures are gone.

`realv6_000` is reported by the scorer as **"failed (ground truth invalid)"**, and that is the
REFERENCE, not the run: the tracker export has a 2.44 s gap, which breaks the scorer's own
`gap_ok` rule (no gap > 1.0 s). The estimate itself is healthy -- 0 resets, RPE 0.048 m, 247 of
248 samples matched. Quote the number with that caveat, or fix the export; do not report
realv6 as a VINS failure.

### 15.0.1 Candidate sweep: nothing beats the settled file yet

**The repeatability floor, measured properly.** THREE runs of the unchanged file on the same
bag with identical input (`settled`, `rate1_003`, `rate1_002`) gave 0.676 / 0.678 / 0.708 m:
mean 0.687, sd 0.018, full range 0.032 m. So the bar a candidate has to clear is
**mean - 2sd = 0.652 m**, and anything between 0.65 and 0.72 m is this estimator talking to
itself. Measure this before any sweep on a new bag; it is three runs and it decides what the
sweep can possibly conclude. (Independently consistent with the project's own note that VINS
repeats to 3-7 %: 0.018/0.687 = 2.6 %.) Scored so far:

| candidate | change | ATE | RPE 1 s | verdict |
|---|---|---|---|---|
| **reference** | `config/wil/stereo_imu.yaml` | **0.676 / 0.678 / 0.708 m** | 0.090 | three runs, identical input: mean 0.687, sd 0.018, range 0.032 m |
| c1_fov60 | `fov_mask_deg` 70 -> 60 | 0.701 m | 0.089 | inside the floor, no change |
| c2_fov77 | `fov_mask_deg` 70 -> 77 | 0.680 m | 0.090 | inside the floor, no change |
| c5_features | `min_dist` 15, `max_cnt` 250 | 0.704 m | 0.090 | inside the floor AND 6x the queue wait (43 vs 7 ms mean) -- reject on cost |
| c6_parallax | `keyframe_parallax` 10 -> 5 | 0.713 m | 0.090 | inside the floor, no change |
| c3_td01 | `td` 0.0 -> +0.1 s | 0.767 m | 0.101 | **clearly WORSE** -- see below |
| c4_leverarm | cam0 t -> (0, +0.26, +0.065) m | **0.649 m** | **0.081** | best of any run on this bag, but see below |

**The mask angle is flat between 60 and 77 deg** (0.701 / 0.678-0.708 / 0.680 m). It is not a
tuning knob: either the bad features are excluded or they are not, and anywhere in that range
excludes them. Stop sweeping it. 70 stays, and the choice is now evidence-based rather than a
first guess.

**More features is a pure cost.** `min_dist` 15 with `max_cnt` 250 bought nothing (0.704 m) and
sextupled the mean backend queue wait, which is the mechanism that loses frames on this laptop.

**`td` +0.1 s is clearly worse (0.767 m, RPE 0.090 -> 0.101), and that is informative.** It is
what a correctly-sized offset applied BACKWARDS looks like. The measurement says image motion
lags the gyro by +70..+130 ms, so the frame stamped t shows the world at t-0.1; VINS applies
`currentTimestamp = frame_stamp + td`, so the correction needs td NEGATIVE. c3 tested the sign
I guessed, not the sign the derivation gives -- `stereo_imu_c7_tdm01.yaml` (td -0.1) tests that.
If BOTH signs are worse than 0.0, VINS's td is not the quantity cross-correlation measured and
this line is dead.

**`c4_leverarm` is the only candidate that clears the bar, and only just.** 0.649 m against a
threshold of 0.652 m -- it beats three reference runs by 2 sd with a sample of one, which is
suggestive and nothing more. The stronger signal is the RPE: 0.081 m against 0.090 for every
other run on this bag, and RPE is far less sensitive to the global fit than ATE is. Two runs settle it: a repeat of c4 under a second name, and
`stereo_imu_c8_leverarm_backward.yaml`, which reverses ONLY the forward component to
(0, +0.26, -0.065). If the arm is real, c8 must be clearly worse than c4; if c8 matches c4, the
arm is not what moved the number. The +0.26 m vertical component is not in question -- it is the
large one and its sign is fixed by the accelerometer reading +9.87 on y at rest.

### 15.1 The IMU rotation is also wrong, by 180 deg about the vertical — but VINS cannot see it

Measured on all four real bags from the wheel feedback and the accelerometer, straight
segments only (steering 0): the first start-up reads +0.24/+0.52/+0.49/+0.62 m/s^2 on
IMU **z** (realv6, v7_000, v7_002, v7_001) and the final stops read -0.31/-0.49/-0.52,
while optical-flow expansion shows the camera travelling FORWARD (optical +z). So IMU +z
= forward; `body_T_cam0 = diag(1,-1,-1)` maps forward to body -z. The correct rotation
is `diag(-1,-1,1)`: the RealSense is turned 180 deg about its OPTICAL axis, not about x.
Gravity stays on +y and the yaw-rate sign is unchanged under both, which is exactly why
the realv5 gyro check (ORB record) called the x/z flip "undecidable". The cross-check
against ORB's body-frame velocity on realv7_001 (7-10 s) gives (-0.15, 0, -0.37) vs IMU
strapdown (+0.18, 0, +0.29): opposite in x and z, same in y. ORB-SLAM3's `IMU.T_b_c1`
carries the same wrong rotation. Config variant `config/wil/stereo_imu_imuz180.yaml`
(cam1 re-derived by left-multiplying R_y(180); baseline and cam0_T_cam1 unchanged).

**It changes no VINS number, and the "1.307 -> 1.036 m" first written here was not the
rotation.** `diag(1,-1,-1)^T @ diag(-1,-1,1) = diag(-1,1,-1)`, i.e. exactly 180 deg about
body **y** — and y is the gravity axis on this rig. Rotation about gravity is the
unobservable yaw gauge VINS fixes arbitrarily in `initFirstIMUPose` (`g2R` then yaw
zeroed), and with the extrinsics fixed and the lever arm zero nothing else couples to it.
Measured with both held fixed: 0.739 vs 0.733 m ATE (0.006 m, noise) and the scorer's
yaw alignment differs by 179.8 deg (-80.8 vs +99.0), which is the gauge rotation showing
up where it should. The earlier 1.307/1.036 gap was two different SEEDS for a free
`estimate_extrinsic`, converging differently. So: fix the rotation because it is the
physical truth and because it WILL matter once the lever arm is non-zero — but never
quote an accuracy gain for it. **ORB-SLAM3 is the opposite case: its `IMU.T_b_c1` already
carries a non-zero translation (0.26 m below, 0.065 m backward), so there the 180 deg
error does move the lever arm and must be fixed with its own A/B.**

### 15.2 Two estimator defects fixed the same day (both bit on realv6/realv7)

- `resetState()` cleared `imuBuffer`. Frames are processed up to ~3 s after their IMU
  arrived (JPEG decode + KLT ~70 ms per pair against a 30 Hz stream, reliable depth-100
  queue), so after a reset every frame found only IMU newer than itself, got one
  interpolated sample with a clamped dt, re-initialised on garbage and tripped the
  IMU-gap check again: the 217-reset (realv6) and 125-reset (realv7_001) loops. Fixed:
  the buffer is kept across resets.
- The online camera-IMU offset `td` lived in `options->time_delay`, was never restored
  on reset (realv6 jumped +0.018 -> -2.51 s in one solve and stayed there), and had no
  bound. Fixed: `configured_time_delay_` restored on reset; |td| > 0.2 s forces a reset
  logged as `td_runaway`; new `td_runaway_resets` row in run_summary.csv.
- Measured camera-IMU latency from the bags (image flow vs gyro cross-correlation):
  +70..+130 ms in every window, constant. Header stamps lead receive time by 13 ms
  (cameras) and 0 ms (IMU); no clock problem.

### 15.3 Things ruled out today, so they are not re-tried

g_norm (9.8858 parses, IMU reads 9.867-9.877 at rest on all bags); stereo baseline collapse
(baseline logged constant at 0.09171 with extrinsics fixed); camera-side swap (cam1 LEFT,
d = -6 px, matches both configs); a multi-second camera lag (70-130 ms measured); KLT
failure at motion onset (96-100 % survival offline); motion blur; the IMU orientation
field (parsed, never used); IMU bursts at the node (imu_recv tracks the bag).

### 15.4 Open

Done 17 Nov: the settled values are in `config/wil/stereo_imu.yaml` and the six diagnostic
variants were deleted (their measurements are the table above; the configs themselves are
reconstructible from it, and keeping a superseded config beside its replacement is the
trap this project has already been bitten by).

Still open, in order:
1. **`fov_mask_deg` has one data point (70).** Sweep 60/70/77/80 on realv7_001; the
   normalised coordinate passes |x/z| = 4 at 76 deg, so the useful range is narrow. Note the
   repeatability floor measured on 17 Nov: two runs of the SAME config and bag gave 0.678 and
   0.708 m, so only differences well above 0.03 m are real.
2. **The marker lever arm is zero**, so every number is "provisional" and `estimate_extrinsic`
   cannot responsibly go back to 1. Tape-measure it, or reuse ORB's figures with the z sign
   taken from the corrected rotation.
3. **A fixed `td` of ~0.1 s.** The real camera-IMU offset measured from the bags is
   +70..+130 ms, constant; the file ships `td: 0.0`, i.e. wrong by about a frame.
4. **Re-run realv6, realv7_000 and realv7_002** with the settled config; only realv7_001 has
   been run this way.
5. **ORB-SLAM3's `IMU.T_b_c1`** needs the same 180 deg rotation fix and its own A/B -- its
   lever arm is NON-zero (0.26 m below, 0.065 m back), so there the error is real.
6. **`wil_311026_selfcal/`** is still a realv6 stopgap. `config/wil/stereo_imu_101126.yaml`
   is the 10 Nov calibration, now SCORED AND REJECTED (72.8 m) -- delete it or mark it so.
7. `rate1_001` under realv6 is VOID (two nodes wrote it at once).

**Report impact:** tables 4.16 and 4.18 and the real-rig mechanism paragraph attribute
the VINS failures to `g_norm` and 40-45 % frame delivery. Both are disproved; the
mechanism is the fisheye normalisation plus the free extrinsic. Chapter 4's real-robot
slot needs rewriting once the campaign is re-run.

**Report impact:** tables 4.16 and 4.18 and the real-rig mechanism paragraph attribute
the VINS failures to `g_norm` and 40-45 % frame delivery. Both are disproved; the
mechanism is the fisheye normalisation plus the free extrinsic. Chapter 4's real-robot
slot needs rewriting once the campaign is re-run.
