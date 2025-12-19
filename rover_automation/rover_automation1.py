#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
import random
import math
import sys

class RoverNavigator(Node):

    def __init__(self, arena_width, arena_height):
        super().__init__("rover_explorer_random")

        # Nav2 action client – robot ko goal bhejne ke kaam aata hai
        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        # Costmap subscribe (yeh batata hai konsi jagah safe hai)
        self.costmap_msg = None
        self.create_subscription(
            OccupancyGrid,
            "/global_costmap/costmap",
            self.costmap_callback,
            10
        )

        self.arena_width = float(arena_width)
        self.arena_height = float(arena_height)

        self.grid_resolution = 3.0  # kitne distance par ek point banega

        # Grid points banate hain poore area ke
        self.grid_points = self.generate_grid()

        # Already visited points ko track karne ke liye
        self.visited = set()

        # Jab tak costmap nahi milta tab tak wait karega
        self.wait_for_costmap()

        # Main exploration start hoti hai
        self.explore()

    #grid
    def generate_grid(self):
        """Poore area ke andar 3m spacing ke grid points banata hai."""
        pts = []
        half_w = self.arena_width / 2
        half_h = self.arena_height / 2

        xs = self.arange(-half_w, half_w, self.grid_resolution)
        ys = self.arange(-half_h, half_h, self.grid_resolution)

        for x in xs:
            for y in ys:
                pts.append((x, y))
        return pts

    def arange(self, a, b, step):
        """Float range generate karne ke liye small helper."""
        out = []
        while a <= b:
            out.append(round(a, 2))
            a += step
        return out

    #loop
    def explore(self):
        """Robot ko har step par next best/random goal bhejne ka loop."""
        curr_x, curr_y = 0.0, 0.0  # starting point assume map center

        while len(self.visited) < len(self.grid_points):

            # Smart + random mixture se next goal choose hota hai
            next_goal = self.pick_next_goal(curr_x, curr_y)

            if not next_goal:
                self.get_logger().warn("No reachable unvisited points left.")
                break

            gx, gy = next_goal
            self.visited.add(next_goal)  # mark visited

            # Thodi noise add kar dete hain to look more natural
            jx = gx + random.uniform(-0.3, 0.3)
            jy = gy + random.uniform(-0.3, 0.3)

            self.get_logger().info(f"Moving to: {jx:.2f}, {jy:.2f}")
            self.send_goal(jx, jy)

            curr_x, curr_y = gx, gy

        self.get_logger().info("Exploration finished.")
        rclpy.shutdown()

    #random element
    def pick_next_goal(self, cx, cy):
        """Agla point choose karta hai – thoda smart + thoda random."""
        candidates = []

        for (x, y) in self.grid_points:

            # Already visited point skip
            if (x, y) in self.visited:
                continue

            # Costmap se check karte hain ki jagah safe hai ki nahi
            cost = self.get_cost(x, y)
            if cost is None or cost >= 70:
                continue  # unsafe

            # Robot se distance
            dist = self.distance(cx, cy, x, y)

            # Intelligent score (lower = better)
            base_score = dist * 1.0 + cost * 2.0

            # Randomness factor se thoda unpredictability aati hai
            randomness = random.uniform(0.8, 1.25)

            final_score = base_score * randomness
            candidates.append((final_score, (x, y)))

        if not candidates:
            return None

        # Sort by score
        candidates.sort(key=lambda x: x[0])

        # Top 3 me se random choose – thoda unpredictability
        top_n = min(3, len(candidates))
        return random.choice(candidates[:top_n])[1]

   #costmap snippet
    def get_cost(self, x, y):
        """Costmap ka cost value deta hai – zyada cost = unsafe."""
        if self.costmap_msg is None:
            return None

        info = self.costmap_msg.info

        mx = int((x - info.origin.position.x) / info.resolution)
        my = int((y - info.origin.position.y) / info.resolution)

        if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
            return None

        return self.costmap_msg.data[my * info.width + mx]

    def costmap_callback(self, msg):
        """Yeh callback costmap update hota rahta hai."""
        self.costmap_msg = msg

    def wait_for_costmap(self):
        """Robot costmap aane tak rukta hai."""
        self.get_logger().info("Waiting for costmap...")
        while self.costmap_msg is None:
            rclpy.spin_once(self, timeout_sec=0.1)

     #nav2
    def send_goal(self, x, y):
        """Nav2 ko move command bhejta hai."""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 server not available!")
            return

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.0

        msg = NavigateToPose.Goal()
        msg.pose = pose

        future = self.nav_client.send_goal_async(msg)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()

        if not handle or not handle.accepted:
            self.get_logger().warn("Goal rejected")
            return

        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result)

   #helpers
    def distance(self, ax, ay, bx, by):
        return math.hypot(ax - bx, ay - by)


def main(args=None):
    rclpy.init()

    if len(sys.argv) < 3:
        print("Usage: explorer <arena_width> <arena_height>")
        return

    w = float(sys.argv[1])
    h = float(sys.argv[2])

    RoverNavigator(w, h)
    rclpy.spin()


if __name__ == "__main__":
    main()
