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
