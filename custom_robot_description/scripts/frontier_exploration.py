#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose

from frontier_search import FrontierSearch, Frontier


class FrontierExploration(Node):
    def __init__(self):
        super().__init__('frontier_exploration')

        self.declare_parameter('min_frontier_size', 8)
        self.declare_parameter('max_frontiers_to_check', 8)
        self.declare_parameter('explore_rate_hz', 1.0)
        self.declare_parameter('fails_before_done', 10)

        self.MIN_FRONTIER_SIZE    = self.get_parameter('min_frontier_size').value
        self.MAX_FRONTIERS        = self.get_parameter('max_frontiers_to_check').value
        self.FAILS_BEFORE_DONE    = self.get_parameter('fails_before_done').value

        self.map:  OccupancyGrid | None = None
        self.pose: Point         | None = None
        self.current_goal_active        = False
        self.no_frontier_fails          = 0
        self.no_path_fails              = 0
        self.is_done                    = False

        self.create_subscription(OccupancyGrid, '/map',  self.map_callback,  10)
        self.create_subscription(Odometry,      '/odom', self.odom_callback, 10)

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Waiting for navigate_to_pose action server...')
        self._nav_client.wait_for_server()
        self.get_logger().info('Action server ready.')

        rate = self.get_parameter('explore_rate_hz').value
        self.timer = self.create_timer(1.0 / rate, self.explore_loop)


    def map_callback(self, msg: OccupancyGrid):
        self.map = msg

    def odom_callback(self, msg: Odometry):
        self.pose = msg.pose.pose.position


    def explore_loop(self):
        if self.is_done or self.map is None or self.pose is None:
            return

        if self.current_goal_active:
            return

        start = FrontierSearch.world_to_grid(self.map, self.pose)

        start = (
            max(0, min(start[0], self.map.info.width  - 1)),
            max(0, min(start[1], self.map.info.height - 1)),
        )

        frontiers = FrontierSearch.search(self.map, start)

        if not frontiers:
            self.get_logger().info('No frontiers found.')
            self.no_frontier_fails += 1
            self._check_if_done()
            return

        self.no_frontier_fails = 0

        top = sorted(frontiers, key=lambda f: f.size, reverse=True)[:self.MAX_FRONTIERS]
        best: Frontier = top[0]

        self.get_logger().info(
            f'Sending goal to frontier: size={best.size} '
            f'centroid=({best.centroid.x:.2f}, {best.centroid.y:.2f})'
        )
        self._send_goal(best.centroid)


    def _send_goal(self, position: Point):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position = position
        goal.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        send_future = self._nav_client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback,
        )
        send_future.add_done_callback(self._goal_response_callback)
        self.current_goal_active = True

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2.')
            self.current_goal_active = False
            self.no_path_fails += 1
            self._check_if_done()
            return

        self.get_logger().info('Goal accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        self.current_goal_active = False
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Reached frontier.')
            self.no_path_fails = 0
        else:
            self.get_logger().warn(f'Navigation failed with status {status}.')
            self.no_path_fails += 1
            self._check_if_done()

    def _feedback_callback(self, feedback):
        pass


    def _check_if_done(self):
        if (self.no_frontier_fails >= self.FAILS_BEFORE_DONE or
                self.no_path_fails   >= self.FAILS_BEFORE_DONE):
            self.get_logger().info('Exploration complete.')
            self.is_done = True
            self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExploration()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()