#!/usr/bin/env python3
"""
mission_runner.py
-----------------
Sends a sequence of Nav2 goals using the FollowWaypoints action server.

Usage:
    ros2 run <your_package> mission_runner
    -- OR --
    python3 mission_runner.py   (if sourced into a ROS2 workspace)

Requirements:
    - Nav2 stack running (nav2_bringup)
    - A valid map loaded and localization active (e.g. AMCL)
    - The robot must have an initial pose set before running this node
"""

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def yaw_to_quaternion(yaw_rad: float) -> tuple[float, float, float, float]:
    """Convert a yaw angle (radians) to a quaternion (x, y, z, w).

    Assumes roll = pitch = 0, which is standard for a ground robot.
    """
    return (
        0.0,
        0.0,
        math.sin(yaw_rad / 2.0),
        math.cos(yaw_rad / 2.0),
    )


def make_pose(
    node: "MissionRunner",
    x: float,
    y: float,
    yaw_deg: float,
    frame_id: str = "map",
) -> PoseStamped:
    """Build a stamped pose from (x, y) metres and a yaw in **degrees**."""
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = node.get_clock().now().to_msg()

    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0

    qx, qy, qz, qw = yaw_to_quaternion(math.radians(yaw_deg))
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw

    return pose


# ---------------------------------------------------------------------------
# Mission definition
# ---------------------------------------------------------------------------
# Each waypoint is a dict with:
#   name    - human-readable label shown in the logs
#   x, y    - position in the MAP frame (metres)
#   yaw_deg - heading the robot should arrive at (degrees, 0 = +X axis)
#
# Edit this list to match your environment.
# ---------------------------------------------------------------------------

WAYPOINTS = [
    {"name": "Entrance",     "x":  -1.22706,  "y":  -1.00653,  "yaw_deg":   87.8},
    {"name": "Checkpoint A", "x":  -0.344933,  "y":  0.0226187,  "yaw_deg":  -17.35},
    {"name": "Checkpoint B", "x":  1.21575,  "y":  0.282732,  "yaw_deg": 90.0},
    {"name": "Checkpoint C", "x":  0.118748, "y":  0.972599, "yaw_deg": -128.04},
    {"name": "Checkpoint D", "x":  -0.084819, "y":  -0.084819, "yaw_deg": -137.290428},
    {"name": "Checkpoint D", "x":  -0.152675, "y":  -0.893434, "yaw_deg": -147.5286108},
    {"name": "End",     "x":  -1.22706,  "y":  -1.00653,  "yaw_deg":   87.8}
]

# Frame that the waypoints are expressed in.
# "map" is correct when AMCL / slam_toolbox is running.
# Use "odom" only for odometry-only tests with no localisation.
FRAME_ID = "map"


# ---------------------------------------------------------------------------
# ROS 2 node
# ---------------------------------------------------------------------------

class MissionRunner(Node):
    """Sends the WAYPOINTS list to Nav2 via the FollowWaypoints action server."""

    def __init__(self):
        super().__init__("mission_runner")

        self._client = ActionClient(self, FollowWaypoints, "follow_waypoints")
        self._goal_handle = None

        # Kick off the mission once the node is spinning.
        self.create_timer(0.5, self._start_mission)

    # ------------------------------------------------------------------
    # Mission launch
    # ------------------------------------------------------------------

    def _start_mission(self):
        """Called once after spin starts. Connects to Nav2 and sends goals."""

        # Only run once.
        self.destroy_timer(self._timers[0])  # Remove the one-shot timer.

        self.get_logger().info("=" * 55)
        self.get_logger().info("  Mission Runner — Nav2 FollowWaypoints")
        self.get_logger().info("=" * 55)

        # Print waypoint table.
        self.get_logger().info(f"  Loaded {len(WAYPOINTS)} waypoints:")
        for i, wp in enumerate(WAYPOINTS):
            self.get_logger().info(
                f"    [{i + 1}] {wp['name']:<18} "
                f"x={wp['x']:>6.2f}  y={wp['y']:>6.2f}  "
                f"yaw={wp['yaw_deg']:>7.1f}°"
            )
        self.get_logger().info("-" * 55)

        # Build PoseStamped list.
        poses = [
            make_pose(self, wp["x"], wp["y"], wp["yaw_deg"], FRAME_ID)
            for wp in WAYPOINTS
        ]

        # Wait for the action server to become available.
        self.get_logger().info("Waiting for Nav2 'follow_waypoints' server...")
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                "Nav2 action server not available after 10 s — is Nav2 running?"
            )
            rclpy.shutdown()
            return

        # Send the goal.
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses

        self.get_logger().info("Sending mission to Nav2 ...")
        send_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self._on_feedback,
        )
        send_future.add_done_callback(self._on_goal_response)

    # ------------------------------------------------------------------
    # Action callbacks
    # ------------------------------------------------------------------

    def _on_goal_response(self, future):
        self._goal_handle = future.result()

        if not self._goal_handle.accepted:
            self.get_logger().error("Goal REJECTED by Nav2. Aborting.")
            rclpy.shutdown()
            return

        self.get_logger().info("Goal ACCEPTED — robot is now navigating.")
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg):
        idx = feedback_msg.feedback.current_waypoint
        # Guard against an index that is out of range (Nav2 sends index 0-based).
        if idx < len(WAYPOINTS):
            name = WAYPOINTS[idx]["name"]
            self.get_logger().info(
                f"  → Navigating to waypoint [{idx + 1}/{len(WAYPOINTS)}]: {name}"
            )

    def _on_result(self, future):
        result = future.result().result
        missed = result.missed_waypoints  # List[int32] of skipped indices.

        self.get_logger().info("=" * 55)
        if missed:
            self.get_logger().warn(
                f"Mission finished with {len(missed)} missed waypoint(s): "
                + ", ".join(
                    f"[{i + 1}] {WAYPOINTS[i]['name']}"
                    for i in missed
                    if i < len(WAYPOINTS)
                )
            )
        else:
            self.get_logger().info("Mission COMPLETE — all waypoints reached!")
        self.get_logger().info("=" * 55)

        rclpy.shutdown()

    # ------------------------------------------------------------------
    # Graceful cancel on Ctrl-C
    # ------------------------------------------------------------------

    def destroy_node(self):
        if self._goal_handle is not None:
            self.get_logger().info("Cancelling active goal before shutdown...")
            self._goal_handle.cancel_goal_async()
        super().destroy_node()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = MissionRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user (Ctrl-C).")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()