#!/usr/bin/env python3
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints

def yaw_to_quaternion(yaw_rad: float) -> tuple[float, float, float, float]:
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

WAYPOINTS = [
    {"name": "Entrance",     "x":  1.24968,  "y":  1.24968,  "yaw_deg":   178.024},
    {"name": "Checkpoint A", "x":  0.375883,  "y":  -0.547583,  "yaw_deg":  -149.74},
    {"name": "Checkpoint B", "x":  -1.14621,  "y":  -1.26223,  "yaw_deg": 90.0}
]

FRAME_ID = "map"

class MissionRunner(Node):
    def __init__(self):
        super().__init__("mission_runner")

        self._client = ActionClient(self, FollowWaypoints, "follow_waypoints")
        self._goal_handle = None

        self.create_timer(0.5, self._start_mission)

    def _start_mission(self):
        self.destroy_timer(self._timers[0]) 

        self.get_logger().info("=" * 55)
        self.get_logger().info("  Mission Runner — Nav2 FollowWaypoints")
        self.get_logger().info("=" * 55)

        self.get_logger().info(f"  Loaded {len(WAYPOINTS)} waypoints:")
        for i, wp in enumerate(WAYPOINTS):
            self.get_logger().info(
                f"    [{i + 1}] {wp['name']:<18} "
                f"x={wp['x']:>6.2f}  y={wp['y']:>6.2f}  "
                f"yaw={wp['yaw_deg']:>7.1f}°"
            )
        self.get_logger().info("-" * 55)

        poses = [
            make_pose(self, wp["x"], wp["y"], wp["yaw_deg"], FRAME_ID)
            for wp in WAYPOINTS
        ]

        self.get_logger().info("Waiting for Nav2 'follow_waypoints' server...")
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                "Nav2 action server not available after 10 s — is Nav2 running?"
            )
            rclpy.shutdown()
            return

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses

        self.get_logger().info("Sending mission to Nav2 ...")
        send_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self._on_feedback,
        )
        send_future.add_done_callback(self._on_goal_response)

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
        if idx < len(WAYPOINTS):
            name = WAYPOINTS[idx]["name"]
            self.get_logger().info(
                f"  → Navigating to waypoint [{idx + 1}/{len(WAYPOINTS)}]: {name}"
            )

    def _on_result(self, future):
        result = future.result().result
        missed = result.missed_waypoints  

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

    def destroy_node(self):
        if self._goal_handle is not None:
            self.get_logger().info("Cancelling active goal before shutdown...")
            self._goal_handle.cancel_goal_async()
        super().destroy_node()


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