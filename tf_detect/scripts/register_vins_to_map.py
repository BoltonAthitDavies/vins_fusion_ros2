#!/usr/bin/env python3
"""Measure the SE(2) transform that registers VINS `world` odom into CARLA `map`.

The traffic-light node projects 3D light boxes (OpenDRIVE/CARLA `map` frame) into
the camera using the ego pose. It therefore needs the ego pose in the `map`
frame. But the live MPC state `/vins_stereo_vel/odometry` is in the raw VINS
`world` frame (origin + yaw fixed at VINS init, NOT the map origin) — see
smooth_odom.py without --map-offset. The controller registers it internally via
an SE(2) bootstrap stitch but never publishes the registered pose.

`map_T_world` is a CONSTANT rigid frame relationship, so it can be measured by
pairing each VINS pose (world) with the ground-truth pose (map) at the same
timestamp:

    yaw_off          = wrap( yaw_map - yaw_world )
    (x_off, y_off)   = p_map.xy - Rz(yaw_off) @ p_world.xy
    z_off            = z_map - z_world

This tool accumulates pairs over a window (after a settle period so VINS has
stabilised), reports the robust mean offset AND the residual spread (low spread
=> trustworthy rigid registration; large/growing spread => VINS is drifting or a
frame assumption is wrong), then prints launch args ready to paste into the
traffic-light node:

    odom_map_x_offset / odom_map_y_offset / odom_map_z_offset / odom_map_yaw_offset_deg

This works for our rig because the IMU/body sits directly above ego (x=y=0), so
the body<->ego difference is purely vertical and is absorbed into z_off; x/y/yaw
are unaffected. Leave odom_child_to_ego_* at 0 when using these numbers.

The reference is GT (`/carla/ego_vehicle/odometry`) used ONCE to anchor — exactly
the bootstrap philosophy the MPC already uses (noise_odometry bootstrap). After
you freeze the numbers, runtime needs no GT. Optionally `--republish` streams the
registered odom on `out_topic` (frame `map`) so the node can consume it directly.
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry


def yaw_of(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def rot2(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]], dtype=float)


def circular_mean(angles: List[float]) -> float:
    s = sum(math.sin(a) for a in angles)
    c = sum(math.cos(a) for a in angles)
    return math.atan2(s, c)


def se2_from_pairs(
    pairs: List[Tuple[np.ndarray, float, np.ndarray, float]],
) -> dict:
    """pairs: list of (p_world[xyz], yaw_world, p_map[xyz], yaw_map).

    Returns the aggregated map_T_world (yaw_off, x_off, y_off, z_off) plus
    residual diagnostics. The transform maps a world pose into the map frame:
        p_map ~= Rz(yaw_off) @ p_world + [x_off, y_off, z_off]
    """
    yaw_offs = [wrap_pi(ym - yw) for (_, yw, _, ym) in pairs]
    yaw_off = circular_mean(yaw_offs)
    R = rot2(yaw_off)

    txy = [pm[:2] - R @ pw[:2] for (pw, _, pm, _) in pairs]
    tz = [float(pm[2] - pw[2]) for (pw, _, pm, _) in pairs]
    txy_arr = np.array(txy)
    x_off, y_off = np.median(txy_arr, axis=0)
    z_off = float(np.median(tz))

    # Residual: how well the single frozen transform maps every world pose onto
    # its GT map pose. Small + flat => rigid registration holds.
    t = np.array([x_off, y_off])
    res = [float(np.linalg.norm((R @ pw[:2] + t) - pm[:2])) for (pw, _, pm, _) in pairs]
    yaw_spread = math.degrees(
        max(wrap_pi(y - yaw_off) for y in yaw_offs)
        - min(wrap_pi(y - yaw_off) for y in yaw_offs)
    )
    return {
        "yaw_off_deg": math.degrees(yaw_off),
        "x_off": float(x_off),
        "y_off": float(y_off),
        "z_off": z_off,
        "n": len(pairs),
        "xy_std": float(np.std(np.linalg.norm(txy_arr - [x_off, y_off], axis=1))),
        "res_mean": float(np.mean(res)),
        "res_max": float(np.max(res)),
        "yaw_spread_deg": yaw_spread,
    }


class RegisterVinsToMap(Node):
    def __init__(self) -> None:
        super().__init__("register_vins_to_map")
        self.vins_topic = self.declare_parameter(
            "vins_topic", "/vins_stereo_vel/odometry"
        ).value
        self.ref_topic = self.declare_parameter(
            "ref_topic", "/carla/ego_vehicle/odometry"
        ).value
        self.settle_sec = float(self.declare_parameter("settle_sec", 10.0).value)
        self.window_sec = float(self.declare_parameter("window_sec", 8.0).value)
        self.max_dt = float(self.declare_parameter("max_pair_dt", 0.05).value)
        self.republish = bool(self.declare_parameter("republish", False).value)
        self.out_topic = self.declare_parameter(
            "out_topic", "/vins_stereo_vel_map/odometry"
        ).value

        self._ref_buf: List[Tuple[float, np.ndarray, float]] = []
        self._pairs: List[Tuple[np.ndarray, float, np.ndarray, float]] = []
        self._t0: Optional[float] = None
        self._frozen: Optional[dict] = None
        self._printed = False

        self.create_subscription(Odometry, self.ref_topic, self._on_ref, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.vins_topic, self._on_vins, qos_profile_sensor_data)
        self.out_pub = (
            self.create_publisher(Odometry, self.out_topic, 10) if self.republish else None
        )
        self.get_logger().info(
            f"registering vins={self.vins_topic} -> map (ref={self.ref_topic}); "
            f"settle {self.settle_sec}s then accumulate {self.window_sec}s"
        )

    @staticmethod
    def _stamp(msg: Odometry) -> float:
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    @staticmethod
    def _pose(msg: Odometry) -> Tuple[np.ndarray, float]:
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        return np.array([p.x, p.y, p.z], dtype=float), yaw_of(o.x, o.y, o.z, o.w)

    def _on_ref(self, msg: Odometry) -> None:
        p, y = self._pose(msg)
        self._ref_buf.append((self._stamp(msg), p, y))
        if len(self._ref_buf) > 400:
            self._ref_buf = self._ref_buf[-400:]

    def _on_vins(self, msg: Odometry) -> None:
        t = self._stamp(msg)
        p_w, y_w = self._pose(msg)

        if self._frozen is not None:
            if self.out_pub is not None:
                self._publish_registered(msg, p_w, y_w)
            return

        if self._t0 is None:
            self._t0 = t
        elapsed = t - self._t0
        if elapsed < self.settle_sec or not self._ref_buf:
            return

        # nearest ref by timestamp
        st, sp, sy = min(self._ref_buf, key=lambda r: abs(r[0] - t))
        if abs(st - t) > self.max_dt:
            return
        self._pairs.append((p_w, y_w, sp, sy))

        if elapsed >= self.settle_sec + self.window_sec and len(self._pairs) >= 10:
            self._frozen = se2_from_pairs(self._pairs)
            self._report(self._frozen)

    def _publish_registered(self, src: Odometry, p_w: np.ndarray, y_w: float) -> None:
        f = self._frozen
        R = rot2(math.radians(f["yaw_off_deg"]))
        xy = R @ p_w[:2] + np.array([f["x_off"], f["y_off"]])
        yaw = wrap_pi(y_w + math.radians(f["yaw_off_deg"]))
        out = Odometry()
        out.header = src.header
        out.header.frame_id = "map"
        out.child_frame_id = src.child_frame_id
        out.pose.pose.position.x = float(xy[0])
        out.pose.pose.position.y = float(xy[1])
        out.pose.pose.position.z = float(p_w[2] + f["z_off"])
        out.pose.pose.orientation.z = math.sin(0.5 * yaw)
        out.pose.pose.orientation.w = math.cos(0.5 * yaw)
        out.twist = src.twist
        self.out_pub.publish(out)

    def _report(self, f: dict) -> None:
        if self._printed:
            return
        self._printed = True
        ok = f["res_max"] < 2.0 and f["yaw_spread_deg"] < 5.0
        self.get_logger().info(
            "\n================ VINS world -> CARLA map registration ================\n"
            f" pairs={f['n']}  residual mean={f['res_mean']:.2f}m max={f['res_max']:.2f}m  "
            f"xy_std={f['xy_std']:.2f}m  yaw_spread={f['yaw_spread_deg']:.2f}deg\n"
            f" {'OK: rigid registration looks trustworthy.' if ok else 'WARN: high spread/residual — VINS may be drifting; re-run with a shorter window soon after bootstrap.'}\n"
            "---------------------------------------------------------------------\n"
            " paste into the traffic-light launch (keep odom_child_to_ego_* = 0):\n\n"
            f"   odom_map_x_offset:={f['x_off']:.3f} \\\n"
            f"   odom_map_y_offset:={f['y_off']:.3f} \\\n"
            f"   odom_map_z_offset:={f['z_off']:.3f} \\\n"
            f"   odom_map_yaw_offset_deg:={f['yaw_off_deg']:.3f}\n"
            "====================================================================="
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RegisterVinsToMap()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
