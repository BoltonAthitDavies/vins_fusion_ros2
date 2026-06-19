#!/usr/bin/env python3
"""Publish /traffic_light/route_turn (Right/Left/Straight) from the planner.

The traffic-light node needs to know which approach (turn) the ego will take so
it can pick the correct light box (Right->0, Left->1, Straight->2). Nothing in
the stack publishes that today, so this bridge derives it from the MPC target.

It compares the upcoming target heading (``/carla/ego_vehicle/trajectory_cmd``,
a Pose2D in the SAME ENU frame as the VINS odom — CCW positive) against the
current ego heading from the odom state topic:

    dyaw = wrap(target.theta - ego_yaw)
        dyaw > +angle_threshold  -> Left   (CCW)
        dyaw < -angle_threshold  -> Right  (CW)
        otherwise                -> Straight

Both inputs are ENU, so there is no Y-sign ambiguity. If your planner uses a
different handedness, set ``turn_sign:=-1.0`` to swap Left/Right.

NOTE: validate the Left/Right mapping once against the debug image before
trusting it on the vehicle — heading-delta turn inference is a heuristic and the
correct lookahead/threshold depend on how far ahead play_gt_path looks.
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class RouteTurnFromPath(Node):
    def __init__(self) -> None:
        super().__init__("route_turn_from_path")

        self.target_topic = self.declare_parameter(
            "target_topic", "/carla/ego_vehicle/trajectory_cmd"
        ).value
        self.odom_topic = self.declare_parameter(
            "odom_topic", "/vins_stereo_vel/odometry"
        ).value
        self.turn_topic = self.declare_parameter(
            "turn_topic", "/traffic_light/route_turn"
        ).value
        # Degrees of heading change that counts as a turn (hysteresis: must exceed
        # `angle_threshold` to enter a turn, drop below `release_threshold` to exit).
        self.angle_threshold = math.radians(
            float(self.declare_parameter("angle_threshold_deg", 18.0).value)
        )
        self.release_threshold = math.radians(
            float(self.declare_parameter("release_threshold_deg", 10.0).value)
        )
        self.turn_sign = float(self.declare_parameter("turn_sign", 1.0).value)
        self.publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 5.0).value)

        self.ego_yaw: Optional[float] = None
        self.target_theta: Optional[float] = None
        self.current_turn = "Straight"

        self.turn_pub = self.create_publisher(String, self.turn_topic, 10)
        self.create_subscription(
            Odometry, self.odom_topic, self._on_odom, qos_profile_sensor_data
        )
        self.create_subscription(Pose2D, self.target_topic, self._on_target, 10)
        self.create_timer(1.0 / max(0.5, self.publish_rate_hz), self._tick)

        self.get_logger().info(
            f"route_turn bridge ready: target={self.target_topic}, "
            f"odom={self.odom_topic} -> {self.turn_topic}"
        )

    def _on_odom(self, message: Odometry) -> None:
        q = message.pose.pose.orientation
        self.ego_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def _on_target(self, message: Pose2D) -> None:
        self.target_theta = float(message.theta)

    def _tick(self) -> None:
        if self.ego_yaw is None or self.target_theta is None:
            return
        dyaw = self.turn_sign * wrap_pi(self.target_theta - self.ego_yaw)

        enter = self.angle_threshold
        release = self.release_threshold
        if self.current_turn == "Straight":
            if dyaw > enter:
                self.current_turn = "Left"
            elif dyaw < -enter:
                self.current_turn = "Right"
        else:
            # Stay in the turn until the heading delta clearly relaxes.
            if abs(dyaw) < release:
                self.current_turn = "Straight"
            elif dyaw > enter:
                self.current_turn = "Left"
            elif dyaw < -enter:
                self.current_turn = "Right"

        self.turn_pub.publish(String(data=self.current_turn))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RouteTurnFromPath()
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
