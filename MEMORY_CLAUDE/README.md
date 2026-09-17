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
