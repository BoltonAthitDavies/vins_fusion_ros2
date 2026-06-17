# Autonomous Driving in CARLA — VINS-Fusion SLAM + MPC + Traffic-Light Detection

A ROS 2 (Humble) stack that drives a CARLA ego vehicle autonomously by closing the loop over three
components:

1. **SLAM / state estimation** — a ROS 2 port of **VINS-Fusion** (stereo + IMU + GPS variants, with
   `global_fusion` for GPS), validated against CARLA ground truth.
2. **Control** — a **sampling Model-Predictive Controller (MPC)** that follows a reference trajectory
   using the estimated state.
3. **Perception** — a **traffic-light detector** (`src/tf_detect`) that turns the front camera into a
   stop / slow / go action.

> **Status.** SLAM is fully evaluated (online vs offline vs live, 4 variants × 5 runs × 3 scenes).
> The MPC and traffic-light detector are implemented and run live; their standalone experiments and
> the full closed-loop integration are **not yet evaluated** and are marked *"to be added"* below.

---

## Table of contents

1. [Problem statement](#1-problem-statement)
2. [Environment setup](#2-environment-setup)
3. [Ego vehicle](#3-ego-vehicle-kinematics-state-input-sensors)
4. [SLAM (VINS-Fusion): setup, experiment, results, analysis](#4-slam-vins-fusion)
5. [MPC: setup & method](#5-mpc-model-predictive-control)
6. [Traffic-light detection](#6-traffic-light-detection)
7. [Integration (SLAM + MPC + traffic light)](#7-integration)
8. [Conclusion & discussion](#8-conclusion--discussion)
9. [Commands](#9-commands)

---

## 1. Problem statement

**Goal: drive an ego vehicle in CARLA using SLAM-estimated state for MPC trajectory following, with
traffic-light-aware stopping.**

The vehicle must not rely on CARLA ground truth at run time. Instead it estimates its own pose from
on-board sensors (stereo cameras + IMU + GPS) via VINS-Fusion, feeds that estimate to an MPC that
tracks a reference path, and gates throttle/brake on a camera-based traffic-light state.

```
                       ┌─────────────────────────────────────────────┐
   CARLA (sync mode)   │  stereo 20 Hz · IMU 200 Hz · GNSS 10 Hz      │
   Tesla Model 3       └───────────────┬──────────────────┬──────────┘
                                       │                  │
                       ┌───────────────▼─────┐   ┌────────▼───────────┐
                       │  VINS-Fusion + GPS  │   │ traffic-light       │
                       │  (state estimation) │   │ detector (tf_detect)│
                       └───────────────┬─────┘   └────────┬───────────┘
                            state              stop/slow/go
                       {x, y, yaw, speed}             │
                                       │              │
                              ┌────────▼──────────────▼────────┐
                              │  sampling MPC (bicycle model)   │
                              └────────────────┬────────────────┘
                                  VehicleControl{throttle, steer, brake}
                                               │
                                       ┌───────▼────────┐
                                       │  CARLA ego     │
                                       └────────────────┘
```

---

## 2. Environment setup

| Component | Detail |
|-----------|--------|
| OS / middleware | Ubuntu, **ROS 2 Humble** |
| Simulator | **CARLA 0.9.x**, **synchronous mode**, fixed Δt = 0.005 s (**200 Hz** world tick) |
| Map | `Town10HD` (primary), `Town01` |
| Ego | `vehicle.tesla.model3` |
| Optimizer | **Ceres 2.2** (Manifold API) |
| Vision | **OpenCV 4.10 + CUDA** built in `~/local` (GPU feature tracking) |
| 3D mapping | apt **`rtabmap_ros`** (run under a clean `LD_LIBRARY_PATH` to avoid an ABI clash with `~/local`) |
| Packages | `vins_fusion_ros2`, `global_fusion`, `tf_detect` |

CUDA OpenCV must be on the library path at both link and run time:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
export LD_LIBRARY_PATH=/home/fibo3/local/lib:$LD_LIBRARY_PATH   # CUDA OpenCV 4.10
colcon build --packages-select vins_fusion_ros2 global_fusion --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash   # re-source after every build
```

---

## 3. Ego vehicle (kinematics, state, input, sensors)

### Sensors

| Sensor | Spec | Rate |
|--------|------|------|
| Stereo RGB cameras | 960 × 720, FOV 90°, `fx=fy=480`, `cx=480`, `cy=360`, **baseline 0.5 m** | **20 Hz** |
| IMU | 6-axis, mounted at CoG height (z = 1.5 m) | **200 Hz** |
| GNSS | WGS-84 lat/lon/alt (noise-free in sim) | **10 Hz** |
| World tick | synchronous, fixed Δt = 0.005 s | **200 Hz** |

Cameras are mounted at x = 1.5 m, z = 1.5 m; left at y = −0.25 m, right at y = +0.25 m (CARLA frame).

### Frame convention (CARLA → ROS/ENU)

CARLA is **left-handed (+y = right, yaw clockwise)**; ROS/ENU is right-handed (+y = left, yaw CCW).
The bridge (`carla_world.cpp`, `getEgoState`/`getEgoOdom`) converts by **negating y, yaw, vy, and ωz**:

```
x_ros =  x_carla
y_ros = -y_carla
yaw_ros = -yaw_carla        vy_ros = -vy_carla      ωz_ros = -ωz_carla
```

> A left/right **camera swap** (a consequence of this +y = right convention) was the root cause of
> early live divergence; fixing it is what made live VINS match the offline result (see §4).

### Kinematic model & control

The MPC rolls out a **kinematic bicycle model** (wheelbase **L = 2.875 m**, dt = 0.10 s):

```
x   += v · cos(ψ) · dt
y   += v · sin(ψ) · dt
ψ   += (v / L) · tan(δ) · dt
v   += a · dt
```

- **State:** `{x, y, yaw, speed}` (ROS/ENU frame).
- **Input:** `{steer δ, accel a}`, mapped to a CARLA `VehicleControl`:
  `throttle ≈ 0.10 + 0.20·a + 0.04·(speed error)` (clamped to ≤ 0.45), `brake = −a / max_decel`,
  `steer = −δ` (sign flip back into the CARLA frame). Max speed 8 m/s, max accel 2 m/s², max decel
  4 m/s².

---

## 4. SLAM (VINS-Fusion)

### 4.1 Setup

Four estimator variants, each run **5 times** per scene, across **three evaluation paths**:

| Variant | Sensors |
|---------|---------|
| `stereo` | stereo cameras only (VO) |
| `stereo+imu` | stereo + IMU (VIO) |
| `stereo+gps` | stereo VO fused with GPS via `global_fusion` |
| `stereo+imu+gps` | stereo+IMU fused with GPS |

| Path | How | Properties |
|------|-----|-----------|
| **online** | `ros2 bag play … --clock` | what you'd get live; timing-dependent |
| **offline** | `vins_bag_reader` (direct rosbag read) | **~10× faster, bit-deterministic** |
| **live** | native CARLA, real-time C++ node | the real target; runs all variants as observers |

Ground truth = CARLA `/carla/ego_vehicle/odometry`. Metric = **APE RMSE** (Umeyama-aligned) and
**5-run spread** (run-to-run determinism); `DIV` = diverged (> 10⁴ m).

### 4.2 Results

**`town01_normal`** — easy baseline (APE RMSE [m] / 5-run spread [m]):

| Variant | online | offline |
|---------|:------:|:------:|
| stereo | 20.6 / 0.000 | 20.6 / 0.000 |
| stereo+imu | 22.8 / 7.2 | 80.1 / 0.000 |
| stereo+gps | 10.8 / 17.1 | 1.3 / 0.000 |
| **stereo+imu+gps** | 1.5 / 2.6 | **0.5 / 0.000** |

**`town10_normal`** — normal-speed loop (APE RMSE [m]):

| Variant | online | offline | live |
|---------|:------:|:------:|:----:|
| stereo | 34.1 | 6.4 | **0.6** |
| stereo+imu | 1388 | 2182 | **DIV** |
| stereo+gps | 2.6 | 0.3 | **0.3** |
| stereo+imu+gps | 173.6 | 121.4 | **DIV** |

**`town10_alwaysrun`** — continuous, varied motion (APE RMSE [m] / spread [m]):

| Variant | online | offline |
|---------|:------:|:------:|
| stereo | DIV / — | 9.3 / 16.1 |
| stereo+imu | 920 / 3.3 | 27.2 / 165 |
| stereo+gps | DIV / — | 0.5 / 1.19 |
| **stereo+imu+gps** | 171.9 / 593 | **0.3 / 0.24** |

| Live tracking (town10_normal) | Divergence onset (town10_normal) |
|---|---|
| ![live track](figures/track_live_town10_normal.png) | ![live onset](figures/onset_live_town10_normal.png) |

| Offline track — town01 (all bounded) | Offline track — town10_alwaysrun (all four bounded) |
|---|---|
| ![direct town01](figures/track_direct_town01_normal.png) | ![direct alwaysrun](figures/track_direct_town10_alwaysrun.png) |

### 4.3 Analysis — key findings

1. **GPS is the single biggest accuracy win.** On every normal-speed scene the GPS variants are the
   most accurate (sub-metre offline/live), and they stay bounded where pure VO/VIO diverges.
2. **Pure stereo is the most *repeatable*** (bit-identical across 5 runs offline) but **never the most
   accurate**, and it can diverge on low-excitation motion.
3. **The IMU paths diverge on smooth, normal driving** — and this is a **fundamental visual-inertial
   observability degeneracy**, not a tuning bug. CARLA's smooth, planar, low-rotation motion leaves
   the IMU-coupled states (velocity, accel/gyro bias, gravity) unobservable. This is corroborated
   cross-dataset: VINS-Fusion's stereo+IMU **works** on EuRoC (rich drone rotation) and KITTI, but
   **diverges** on CARLA stereo+IMU while CARLA stereo-only stays bounded:

   | Live: stereo bounded vs IMU km-explosion | EuRoC: IMU works on rich motion |
   |---|---|
   | ![carla stereo-only](figures/vio_carla_so_vs_gt.png) | ![euroc](figures/euroc_vio_vs_gt.png) |

   The **varied motion** of `town10_alwaysrun` restores IMU observability (stereo+imu+gps → 0.3 m),
   confirming the cause is the *motion profile*, not the pipeline.
4. **Live reproduces offline.** After fixing the left/right camera swap and raising the rates
   (20 Hz cameras, 200 Hz IMU, 200 Hz world tick), live stereo and stereo+gps are **sub-metre and
   deterministic** in real time (0.6 m and 0.3 m). Both IMU variants diverge on **all 5** live runs —
   which rules out a control/timing artifact and points squarely at the degeneracy above.
5. The estimator itself is deterministic; non-determinism only appears in `global_fusion` when it is
   fed a marginal/diverged estimate. Real-time performance: ~3.3 ms/frame on CPU (backlog 0).

   ![perf](figures/perf_ab.png)

---

## 5. MPC (Model-Predictive Control)

> *Experiment / result / analysis: **to be added.*** This section documents the method as implemented.

A **sampling (brute-force) MPC** in `mpc.hpp`. Each control cycle it evaluates a small grid of
candidate inputs and keeps the lowest-cost one:

- **Candidates:** 7 steering offsets × 6 acceleration levels = **42 bicycle-model rollouts** per cycle.
- **Horizon:** 100 steps × 0.10 s = 10 s (configurable via `horizon:=`).
- **Cost:** lookahead-weighted sum of position error + line-of-sight heading + terminal heading +
  speed-tracking error, plus steering / Δsteering / acceleration regularizers. Steering is
  rate-limited and low-pass filtered to prevent chatter.
- **Defaults:** `target_speed` 3.5 m/s (raise to ~5.5 for autopilot-recorded GT), wheelbase 2.875 m.

**Driving the MPC on a VINS state (bootstrap + handover).** Because a VINS estimate is not available
at t = 0 (and lives in a different frame from the recorded path), the MPC first drives on
`/carla/ego_vehicle/noise_odometry` (ground truth + bounded noise, available immediately) for
`bootstrap_secs`, then **SE(2)-stitches** the selected VINS odom into that bootstrap frame and hands
over. Two helpers make a VINS estimate usable as the MPC state:

- **Frame registration** — the recorded path is in CARLA `map` coordinates; `/vins_stereo_gps` is in
  GPS-ENU (origin at the first GPS fix). `smooth_odom.py --map-offset 100,10` adds the spawn offset
  and relabels the frame so the registered estimate can be driven exactly like ground truth.
- **Velocity fill** — VINS odometry carries no twist, so the MPC's speed input would be 0 (→ constant
  max throttle). `smooth_odom.py` fills the twist from a smoothed position derivative.

Drive modes (`drive:=`): `coverage` (Traffic-Manager loop route, ignores lights — for map coverage),
`autopilot` (plain Traffic Manager), `trajectory` (MPC follows `/carla/ego_vehicle/trajectory_cmd`).

---

## 6. Traffic-light detection

> *Experiment / result / analysis: **to be added.*** Code in [`tf_detect/`](tf_detect/).

A **hybrid 3-stage** detector that locates the relevant light geometrically, confirms it with a neural
net, then reads its color:

1. **Geometric projection (where to look).** Parse the OpenDRIVE map (`Town10HD.xodr`) plus
   `carla_light_boxes.csv` (exact lamp-head geometry dumped from CARLA actors), and project the 3D
   light into the image using the ego odometry + camera intrinsics/extrinsics → a 2D region of
   interest. A route-turn hint picks the relevant signal at junctions.
2. **YOLOv11-small confirmation (is a light there).** `yolo11s.pt` (COCO class 9 = traffic light)
   runs on the ROI to confirm and tighten the box.
3. **HSV color classification (what state).** The confirmed box is split into vertical thirds
   (top = red, middle = yellow, bottom = green); the active-pixel (high saturation/value) vertical
   position gives the state plus a confidence score.

**Topics.** Subscribes: `/carla/ego_vehicle/<cam>/image` (+ `camera_info`),
`/carla/ego_vehicle/odometry`, optional `/traffic_light/route_turn`. Publishes:
`/traffic_light/state` (`red`/`green`/`yellow`/`unknown`/`none`), `/traffic_light/action`
(`stop`/`slow`/`go`), `/traffic_light/status` (JSON detail), optional `/traffic_light/debug_image`.

Run live with `ros2 launch tf_detect traffic_light_state.launch.py`; analyse a recorded bag offline
with `scripts/validate_projection.py` (writes an annotated video + per-frame CSV).

---

## 7. Integration

> *To be added.*

The intended closed loop: **VINS state → MPC follows the recorded reference path → traffic-light
action gates throttle/brake.** All three components run live today (VINS state drives the MPC around a
full town10 lap; the traffic-light node publishes a stop/go action), but the combined evaluation —
following a path *and* stopping at lights, end to end — has not yet been measured.

---

## 8. Conclusion & discussion

- **SLAM is validated.** GPS-fused variants are sub-metre and deterministic offline; **live reproduces
  offline** in real time once the camera swap and sensor rates were fixed.
- **The IMU-on-smooth-motion failure is a genuine observability degeneracy**, not a bug — it persists
  across online/offline/live and is cured by varied motion, and matches VINS-Fusion behaviour on
  EuRoC/KITTI. The practical recommendation is **stereo+GPS** for normal CARLA driving.
- **MPC and traffic-light detection are implemented and run live**, but lack standalone evaluation.

**Next steps:** (1) close and measure the full SLAM + MPC + traffic-light loop; (2) quantify MPC
path-tracking error against the reference; (3) report traffic-light detection precision/recall and
state-transition latency.

---

## 9. Commands

> Verified against [`launch/carla_native_multi.launch.py`](launch/carla_native_multi.launch.py) and
> the repo-root scripts. Source `src/Athit VSLAM` is the original cheat-sheet; one path was corrected
> here (`carla_stereo_config.yaml` → `carla_stereo.yaml`).

### Build

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
export LD_LIBRARY_PATH=/home/fibo3/local/lib:$LD_LIBRARY_PATH
colcon build --packages-select vins_fusion_ros2 global_fusion --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### Method 1 — online (rosbag playback)

Run each estimator in its own terminal, then play a bag:

```bash
# A: stereo
ros2 run vins_fusion_ros2 vins_fusion_ros2_node --ros-args -r __node:=vins_stereo \
  -p use_sim_time:=true -p config_file:=$PWD/src/config/carla/carla_stereo.yaml \
  -r odometry:=/vins_stereo/odometry -p output_path:=$PWD/output_rosbag/stereo2

# B: stereo+imu
ros2 run vins_fusion_ros2 vins_fusion_ros2_node --ros-args -r __node:=vins_stereo_imu \
  -p use_sim_time:=true -p config_file:=$PWD/src/config/carla/carla_stereo_imu.yaml \
  -r odometry:=/vins_stereo_imu/odometry -p output_path:=$PWD/output_rosbag/stereo_imu2

# C: stereo+gps (global_fusion)
ros2 run global_fusion global_fusion_node --ros-args -r __node:=gfus_stereo \
  -p use_sim_time:=true \
  -r /vins_estimator/odometry:=/vins_stereo/odometry -r /gps:=/carla/ego_vehicle/gnss \
  -r global_odometry:=/vins_stereo_gps/odometry \
  -p output_csv:=$PWD/output_rosbag/stereo_gps2/vio.csv

# D: stereo+imu+gps (global_fusion)
ros2 run global_fusion global_fusion_node --ros-args -r __node:=gfus_stereo_imu \
  -p use_sim_time:=true \
  -r /vins_estimator/odometry:=/vins_stereo_imu/odometry -r /gps:=/carla/ego_vehicle/gnss \
  -r global_odometry:=/vins_stereo_imu_gps/odometry \
  -p output_csv:=$PWD/output_rosbag/stereo_imu_gps2/vio.csv

# F: play the bag
ros2 bag play rosbag2_2026_06_07-11_29_12_town01_drivenormal --clock -r 1.0
```

### Method 2 — offline (direct read, fast & deterministic)

```bash
BR=install/vins_fusion_ros2/lib/vins_fusion_ros2/vins_bag_reader
BAG=rosbag2_2026_06_07-11_29_12_town01_drivenormal

# stereo (no extra args) / stereo+gps (append: gps /carla/ego_vehicle/gnss)
$BR src/config/carla/carla_stereo.yaml     $BAG ./output_direct/out_stereo1.csv
$BR src/config/carla/carla_stereo.yaml     $BAG ./output_direct/out_stereogps1.csv    gps /carla/ego_vehicle/gnss
$BR src/config/carla/carla_stereo_imu.yaml $BAG ./output_direct/out_stereoimu1.csv
$BR src/config/carla/carla_stereo_imu.yaml $BAG ./output_direct/out_stereoimugps1.csv gps /carla/ego_vehicle/gnss
```

### Method 3 — live (native CARLA C++)

```bash
# A: start CARLA
/home/fibo3/carla-package/CarlaUE4.sh -RenderOffScreen -quality-level=Low

# B: drive. coverage (map sweep):
ros2 launch vins_fusion_ros2 carla_native_multi.launch.py \
  town:=Town10HD spawn:=100.0,-10.0,1.0,0.0,0.0,90.0 drive:=coverage variants:=stereo_gps

#    trajectory (MPC) on stereo VINS:
ros2 launch vins_fusion_ros2 carla_native_multi.launch.py \
  town:=Town10HD spawn:=100.0,-10.0,1.0,0.0,0.0,90.0 drive:=trajectory \
  mpc_state:=/vins_stereo_vel/odometry bootstrap_secs:=8.0 target_speed:=5.5 horizon:=15 variants:=stereo

#    trajectory (MPC) on stereo+gps:
ros2 launch vins_fusion_ros2 carla_native_multi.launch.py \
  town:=Town10HD spawn:=100.0,-10.0,1.0,0.0,0.0,90.0 drive:=trajectory \
  mpc_state:=/vins_stereo_gps_map/odometry bootstrap_secs:=8.0 target_speed:=5.5 horizon:=15 variants:=stereo_gps

# C: live plot
python3 plot_result_rtab_cpp.py

# D: publish the reference path. GT state:
python3 play_gt_path.py --npz gt/rtab_log_gt_town10.npz --state /carla/ego_vehicle/odometry --lookahead 20.0
#    VINS/registered state (map frame, no anchor needed):
python3 play_gt_path.py --npz gt/rtab_log_gt_town10.npz \
  --state /carla/ego_vehicle/noise_odometry --position-mode --lookahead 6.0

# E: log every odom variant
python3 mpc_debug_log.py --out run5

# F: register + velocity-fill the VINS odom used as mpc_state
python3 smooth_odom.py --in /vins_stereo/odometry --out /vins_stereo_vel/odometry --max-track-speed 1000   # stereo
python3 smooth_odom.py --map-offset 100,10 --out /vins_stereo_gps_map/odometry                              # stereo+gps
```

### RTAB-Map 3D map

```bash
# build the live map (use_rtab is on by default; add rtab_viz:=true for the live GUI)
ros2 launch vins_fusion_ros2 carla_native_multi.launch.py ... rtab_viz:=true

# view the saved map (open the db the run actually wrote to — default below)
source /opt/ros/humble/setup.bash
rtabmap-databaseViewer ~/rtab_carla_live.db          # Edit → View 3D Map…
```

### Traffic-light detection

```bash
ros2 launch tf_detect traffic_light_state.launch.py
# offline validation over a recorded bag:
python3 src/tf_detect/scripts/validate_projection.py --bag <rosbag>.db3 --map Town10HD.xodr
```
