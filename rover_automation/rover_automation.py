#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
import math
import sys


class RoverNavigator(Node):
    def __init__(self, arena_width, arena_height, final_goal=None):
        super().__init__('rover_auto_navigator')

        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.arena_width = float(arena_width)
        self.arena_height = float(arena_height)
        self.final_goal = final_goal

        self.grid_resolution = 3.0  # meters between grid points
        self.grid_points = self.generate_exploration_points()

        self.get_logger().info(f"Generated {len(self.grid_points)} exploration points.")
        self.send_points_sequentially()

    def generate_exploration_points(self):
        points = []
        half_w = self.arena_width / 2.0
        half_h = self.arena_height / 2.0

        x_vals = self.frange(-half_w, half_w, self.grid_resolution)
        y_vals = self.frange(-half_h, half_h, self.grid_resolution)

        # BFS-like grid sweep (snake pattern)
        for i, x in enumerate(x_vals):
            if i % 2 == 0:
                for y in y_vals:
                    points.append((x, y))
            else:
                for y in reversed(y_vals):
                    points.append((x, y))

        return points

    def frange(self, start, stop, step):
        vals = []
        while start <= stop:
            vals.append(round(start, 2))
            start += step
        return vals

  
    def send_points_sequentially(self):
        for idx, (x, y) in enumerate(self.grid_points):
            self.get_logger().info(f"Navigating to grid point {idx+1}/{len(self.grid_points)}: ({x}, {y})")
            self.send_goal(x, y)

        if self.final_goal:
            fx, fy = self.final_goal
            self.get_logger().info(f"Final destination requested: ({fx}, {fy})")
            self.send_goal(fx, fy)

        self.get_logger().info("Exploration complete!")
        rclpy.shutdown()

  
    def send_goal(self, x, y):
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Nav2 NavigateToPose action server not available!")
            return

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.0  # no rotation needed

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(f"Sending goal: ({x}, {y})")

        future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected!")
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        self.get_logger().info(f"Arrived at ({x}, {y})")

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 3:
        print("Usage:")
        print("  rover_navigation.py <arena_width> <arena_height>  [optional_x optional_y]")
        return

    arena_width = float(sys.argv[1])
    arena_height = float(sys.argv[2])

    final_goal = None
    if len(sys.argv) == 5:
        final_goal = (float(sys.argv[3]), float(sys.argv[4]))

    node = RoverNavigator(arena_width, arena_height, final_goal)


if __name__ == '__main__':
    main()
