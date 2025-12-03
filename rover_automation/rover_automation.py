#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
import math
import sys
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration


class RoverNavigator(Node):
    def __init__(self, arena_width, arena_height, final_goal=None):
        super().__init__('rover_auto_navigator')

        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.costmap_msg = None
        self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.costmap_callback,
            10
        )
        self.arena_width = float(arena_width)
        self.arena_height = float(arena_height)
        self.final_goal = final_goal

        self.grid_resolution = 3.0  # meters between grid points
        self.grid_points = self.generate_exploration_points()
        filtered_points = self.filter_points_with_costmap(self.grid_points)
        self.optimized_points = self.optimize_route(filtered_points)

        self.get_logger().info(
            f"Generated {len(self.grid_points)} exploration points. Optimized route length: {self.path_length(self.optimized_points):.2f} m."
        )
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

  
    def optimize_route(self, points):
        unique_points = list(dict.fromkeys(points))
        if not unique_points:
            return []
        route = self.nearest_neighbor_route(unique_points)
        return self.two_opt(route)

    def nearest_neighbor_route(self, points, start=(0.0, 0.0)):
        remaining = points[:]
        route = []
        current = start
        while remaining:
            idx, point = min(
                enumerate(remaining),
                key=lambda item: self.distance(current, item[1])
            )
            route.append(point)
            current = point
            remaining.pop(idx)
        return route

    def two_opt(self, route):
        if len(route) < 4:
            return route
        best = route[:]
        improved = True
        while improved:
            improved = False
            for i in range(1, len(best) - 2):
                for j in range(i + 1, len(best)):
                    if j - i == 1:
                        continue
                    candidate = best[:]
                    candidate[i:j] = reversed(best[i:j])
                    if self.path_length(candidate) < self.path_length(best):
                        best = candidate
                        improved = True
                        break
                if improved:
                    break
        return best

    def path_length(self, route, start=(0.0, 0.0)):
        if not route:
            return 0.0
        total = self.distance(start, route[0])
        for a, b in zip(route, route[1:]):
            total += self.distance(a, b)
        return total

    def distance(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def is_same_point(self, a, b, tolerance=0.05):
        return self.distance(a, b) <= tolerance

    def send_points_sequentially(self):
        visited = []
        for idx, (x, y) in enumerate(self.optimized_points):
            if any(self.is_same_point((x, y), p) for p in visited):
                continue
            visited.append((x, y))
            self.get_logger().info(
                f"Navigating to optimized point {idx + 1}/{len(self.optimized_points)}: ({x}, {y})"
            )
            self.send_goal(x, y)

        if self.final_goal:
            fx, fy = self.final_goal
            if self.costmap_msg and self.is_occupied((fx, fy)):
                self.get_logger().warn(f"Final destination ({fx}, {fy}) is blocked; skipping.")
            elif not any(self.is_same_point((fx, fy), p) for p in visited):
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

    def costmap_callback(self, msg):
        self.costmap_msg = msg

    def wait_for_costmap(self, timeout_sec=2.0):
        if self.costmap_msg:
            return True
        deadline = self.get_clock().now() + Duration(seconds=float(timeout_sec))
        while rclpy.ok() and self.get_clock().now() < deadline and self.costmap_msg is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.costmap_msg is not None

    def filter_points_with_costmap(self, points, timeout_sec=2.0):
        if not points:
            return []
        if not self.wait_for_costmap(timeout_sec):
            self.get_logger().warn("Costmap unavailable, skipping obstacle filtering.")
            return points
        navigable = [pt for pt in points if not self.is_occupied(pt)]
        if not navigable:
            self.get_logger().warn("All grid points fall in obstacles; reverting to original list.")
            return points
        if len(navigable) < len(points):
            self.get_logger().info(f"Removed {len(points) - len(navigable)} blocked points via costmap.")
        return navigable

    def world_to_map_index(self, point):
        if self.costmap_msg is None:
            return None
        info = self.costmap_msg.info
        mx = int((point[0] - info.origin.position.x) / info.resolution)
        my = int((point[1] - info.origin.position.y) / info.resolution)
        if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
            return None
        return my * info.width + mx

    def is_occupied(self, point, threshold=50):
        index = self.world_to_map_index(point)
        if index is None:
            return True
        value = self.costmap_msg.data[index]
        return value != -1 and value >= threshold

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
