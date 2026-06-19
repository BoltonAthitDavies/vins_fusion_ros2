# Traffic-light detection — experiment plan & data collection

Two experiments, reported separately:

- **Exp. A — Map→camera ROI projection** (geometry only, *no YOLO*): how well the projected
  light-head ROI lands on the image, **ground-truth pose vs live VINS pose**.
- **Exp. B — YOLO + state classification**: presence detection works, but reading
  red/yellow/green does not — quantify *why* (the YOLO box does not frame the 3-lamp housing).

All per-frame numbers are emitted live on `/traffic_light/status` (JSON in `.data`); record it plus
the inputs (below) and analyse offline.

---

## Exp. A — ROI projection accuracy (GT pose vs VINS pose), no YOLO

**Question.** The light ROI is obtained by projecting the 3D light head (`carla_light_boxes.csv`,
`Town10HD.xodr`) into the image with the ego pose. How much does using the *estimated* pose
(`/vins_stereo_vel/odometry`, after `register_vins_to_map.py`) displace that ROI vs the *exact* CARLA
pose (`/carla/ego_vehicle/odometry`)?

**Method.** Run the node (`yolo_model:=disabled`, so stage 3 is skipped and only the projected box is
drawn) **twice on the same recorded bag**, once with `odom_topic:=/carla/ego_vehicle/odometry` and once
with `odom_topic:=/vins_stereo_vel/odometry` + the calibrated `odom_map_*` offsets. Log the projected
box center `proj_box=[u,v,w,h]` from `/traffic_light/status` each frame.

**Metrics** (per frame, then binned by `distance_m`):
| metric | from |
|--------|------|
| ROI center displacement Δu, Δv (px) = VINS-proj − GT-proj | `proj_box[0:2]` of the two runs |
| Δpixel = √(Δu²+Δv²), RMS per distance bin | derived |
| containment: does the VINS ROI still cover the GT light center? | box overlap |
| (optional) yaw/translation residual of the registration | `register_vins_to_map.py` report |

**Plots.** (1) Δpixel vs distance (scatter + per-bin RMS); (2) side-by-side overlay frames (GT box green,
VINS box cyan) at far / mid / near; (3) the SE(2) registration residual table.

**Expected result.** ROI placement is accurate near the light and degrades with distance as VINS drift
grows — a clean, isolated measurement of the map→camera projection under pose error.

---

## Exp. B — YOLO + state classification, and *why the state flickers*

**Question.** Stage-2 YOLO presence detection works, but stage-3 state (red/yellow/green) flickers.
Show numerically that the cause is **the YOLO box not framing the 3-lamp housing**, so the lit lamp's
position carries no reliable state signal, plus the amber-rendering and small apparent size.

**Ground-truth state.** The native controller publishes GT light states on
**`/carla/traffic_lights/gt_status`** (`std_msgs/String`, JSON — added to `carla_vins_multi_node`, no
`carla_msgs` dependency) at ~5 Hz:

```json
{"t": 1234.56, "lights": [{"actor_id": 473, "opendrive_id": "960", "state": "red"}, ...]}
```

Match a status frame's candidate to its GT entry by **`opendrive_id` == the node's `signal_id`** (both
are the OpenDRIVE signal id; `carla_light_boxes.csv` also carries `actor_id`/`opendrive_id` if you need
the actor). Pick the GT entry nearest in time (`t`). Fallback: hand-label per segment from the debug
video.

**Sub-results.**

- **B1 — Presence vs distance.** Fraction of frames YOLO confirms a light (`yolo_status==confirmed`)
  binned by `distance_m`. (Stage 1–2 sanity: localization works.)
- **B2 — Framing mismatch (key evidence).** Compare the geometric `proj_box` (frames the housing, from
  the map) with the YOLO `yolo_box`:
  - height ratio `yolo_h / proj_h`,
  - vertical-center offset `(yolo_cy − proj_cy) / proj_h`,
  - vertical IoU of the two boxes.
  A box that framed the housing would give ratio ≈ 1, offset ≈ 0, IoU high. Show it does **not**.
- **B3 — Position-signal degeneracy (key evidence).** Histogram of the lit-lamp centroid
  `diag.y_norm` **conditioned on ground-truth state**. A working signal puts red ≈ 0.17, yellow ≈ 0.5,
  green ≈ 0.83. Show instead that GT-red clusters at **≈ 0.45** and the three distributions **overlap**.
  Live evidence already seen on a GT-**red** approach: `y_norm ≈ 0.44–0.48` every frame, zone energy
  `[red, yellow, green]` with red ≈ yellow (e.g. `[8244, 8464, 5668]`).
- **B4 — Classification quality.** Confusion matrix of per-frame `raw_state` vs GT state; the
  **flicker rate** (state changes per second of `raw_state`); fraction `low_contrast` / `unknown`.
  Also report the *smoothed* `state` and the `caution`-merge effect (does it stop reliably?).
- **B5 — Resolution limit.** `bbox_height_px` (head apparent height) vs `distance_m`; note the head is
  ~17–45 px over the usable range, with the three lamps ~6–15 px apart.
- **B6 — Colour is not separable.** Scatter `diag.rg = (R−G)/(R+G)` vs GT state; show red and yellow
  both sit at rg ≈ 0 (amber, `lit_rgb ≈ [250,225,100]`), so hue cannot split them; green is separable.

**Conclusion to support.** Failure is *data-limited, not tuning-limited* (README §6.2): the YOLO box
framing (B2) makes position (B3) non-discriminative, amber rendering (B6) kills colour, and small size
(B5) prevents resolving the lamps — together they explain the flicker (B4).

---

## Rosbag recording — all useful topics

Record during a live run with the full stack up (CARLA + controller + `smooth_odom` + `play_gt_path` +
`traffic_light` node). One bag is enough for both experiments:

```bash
ros2 bag record -o tl_exp_$(date +%Y%m%d_%H%M%S) \
  /clock /tf /tf_static \
  /carla/ego_vehicle/cam_front_right/image /carla/ego_vehicle/cam_front_right/camera_info \
  /carla/ego_vehicle/cam_front_left/image  /carla/ego_vehicle/cam_front_left/camera_info \
  /carla/ego_vehicle/imu /carla/ego_vehicle/gnss \
  /carla/ego_vehicle/odometry /carla/ego_vehicle/noise_odometry \
  /carla/traffic_lights/gt_status \
  /vins_stereo/odometry /vins_stereo_vel/odometry \
  /carla/ego_vehicle/trajectory_cmd /traffic_light/route_turn \
  /traffic_light/state /traffic_light/action /traffic_light/status \
  /traffic_light/debug_image
```

Why each group:
- **clock/tf** — replay time base + frames.
- **stereo + imu + gnss + camera_info** — re-run VINS *and* the projection/YOLO offline (deterministic).
- **GT + noise odometry** — Exp. A reference pose and the MPC bootstrap state.
- **traffic_lights/gt_status** — GT light states (B4 confusion-matrix labels), from the controller.
- **vins odometry (raw + registered)** — the live pose actually used (Exp. A "VINS" arm).
- **trajectory_cmd / route_turn** — intersection-phase context.
- **traffic_light/{state,action,status}** — the live decisions + the full per-frame `diag`
  (`y_norm, rg, lit_rgb, lit_px, zone_energy, proj_box, yolo_box, bbox_height_px, distance_m, ...`),
  so most of Exp. B is computable straight from `/traffic_light/status` without re-running anything.
- **debug_image** — qualitative figures (drop it if disk-bound: it is the largest topic).

> **GT light state** is provided by the native controller on `/carla/traffic_lights/gt_status` (the
> `carla_ros_bridge` `/carla/traffic_lights/status` is not published in the native path). It reads every
> `*traffic_light*` actor's `GetState()` + `GetOpenDRIVEID()` each ~0.2 s. Without it, Exp. B still keeps
> B1/B2/B3(shape)/B5/B6 (the framing + degeneracy evidence need no GT); only the confusion matrix (B4)
> and per-state conditioning need the labels.
