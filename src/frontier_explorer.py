#!/usr/bin/env python3
"""
Frontier Explorer Node
----------------------
Implements frontier-based exploration to autonomously solve a maze.
Fixes:
  - Goals are validated to be in free space before sending to Nav2
  - Minimum distance filter prevents navigating to already-nearby frontiers
  - Nav2 startup delay prevents premature goal sending

Subscribes:
  /map                  (nav_msgs/OccupancyGrid)
  /odom/filtered        (nav_msgs/Odometry)
  /scan                 (sensor_msgs/LaserScan)

Action Client:
  /navigate_to_pose     (nav2_msgs/action/NavigateToPose)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

import numpy as np
from enum import Enum, auto

from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class State(Enum):
    IDLE       = auto()
    WAITING    = auto()   # waiting for Nav2 to be ready
    FINDING    = auto()
    NAVIGATING = auto()
    DEAD_END   = auto()
    DONE       = auto()


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('min_frontier_size',   5)
        self.declare_parameter('min_goal_distance',   0.5)    # don't navigate to frontiers closer than this
        self.declare_parameter('clearance_threshold', 0.4)
        self.declare_parameter('unknown_threshold',   0.15)
        self.declare_parameter('nav_timeout_sec',     30.0)
        self.declare_parameter('occupied_threshold',  50)     # costmap value above this = occupied
        self.declare_parameter('nav2_wait_sec',       5.0)    # wait for Nav2 to fully start
        # ─────────────────────────────────────────────────────────────────

        self.min_frontier_size  = self.get_parameter('min_frontier_size').value
        self.min_goal_distance  = self.get_parameter('min_goal_distance').value
        self.clearance_threshold= self.get_parameter('clearance_threshold').value
        self.unknown_threshold  = self.get_parameter('unknown_threshold').value
        self.nav_timeout_sec    = self.get_parameter('nav_timeout_sec').value
        self.occupied_threshold = self.get_parameter('occupied_threshold').value
        self.nav2_wait_sec      = self.get_parameter('nav2_wait_sec').value

        # State
        self.state             = State.IDLE
        self.map_data          = None
        self.robot_x           = 0.0
        self.robot_y           = 0.0
        self.scan_ranges       = None
        self.scan_angles       = None
        self.failed_frontiers  = set()
        self.nav_start_time    = None
        self._goal_handle      = None
        self._current_goal_xy  = None
        self._start_time       = self.get_clock().now()

        # Subscriptions
        self.create_subscription(OccupancyGrid, '/map',           self.map_callback,  10)
        self.create_subscription(Odometry,      '/diff_drive_base_controller/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan,     '/scan',          self.scan_callback, 10)

        # Nav2 action client
        self._nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # State machine at 1 Hz
        self.create_timer(1.0, self.state_machine)

        self.get_logger().info('FrontierExplorer started — waiting for map...')

    # ── Callbacks ──────────────────────────────────────────────────────────
    def map_callback(self, msg: OccupancyGrid):
        self.map_data = msg
        if self.state == State.IDLE:
            self.state = State.WAITING
            self._start_time = self.get_clock().now()
            self.get_logger().info(
                f'Map received — waiting {self.nav2_wait_sec}s for Nav2 to be ready...')

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        self.scan_ranges = np.where(np.isfinite(ranges), ranges, 0.0)
        self.scan_angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

    # ── Frontier Detection ─────────────────────────────────────────────────
    def find_frontiers(self):
        msg    = self.map_data
        width  = msg.info.width
        height = msg.info.height
        res    = msg.info.resolution
        ox     = msg.info.origin.position.x
        oy     = msg.info.origin.position.y
        grid   = np.array(msg.data, dtype=np.int8).reshape((height, width))

        free    = (grid == 0)
        unknown = (grid == -1)

        unknown_neighbour = (
            np.roll(unknown,  1, axis=0) |
            np.roll(unknown, -1, axis=0) |
            np.roll(unknown,  1, axis=1) |
            np.roll(unknown, -1, axis=1)
        )
        frontier_mask  = free & unknown_neighbour
        frontier_cells = np.argwhere(frontier_mask)

        if len(frontier_cells) == 0:
            return []

        visited  = np.zeros((height, width), dtype=bool)
        clusters = []

        for cell in frontier_cells:
            r, c = int(cell[0]), int(cell[1])
            if visited[r, c]:
                continue
            cluster = []
            queue   = [(r, c)]
            visited[r, c] = True
            while queue:
                cr, cc = queue.pop(0)
                cluster.append((cr, cc))
                for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
                    nr, nc = cr+dr, cc+dc
                    if 0 <= nr < height and 0 <= nc < width:
                        if not visited[nr, nc] and frontier_mask[nr, nc]:
                            visited[nr, nc] = True
                            queue.append((nr, nc))

            if len(cluster) >= self.min_frontier_size:
                mean_r = np.mean([p[0] for p in cluster])
                mean_c = np.mean([p[1] for p in cluster])
                wx = ox + (mean_c + 0.5) * res
                wy = oy + (mean_r + 0.5) * res

                dist = np.hypot(wx - self.robot_x, wy - self.robot_y)

                # Skip frontiers that are too close
                if dist < self.min_goal_distance:
                    continue

                # Skip frontiers whose centroid lands in occupied/unknown space
                if not self.is_free(wx, wy):
                    # Try to find nearest free cell in cluster instead
                    adjusted = self.find_free_near_cluster(cluster, ox, oy, res)
                    if adjusted is None:
                        continue
                    wx, wy = adjusted
                    dist = np.hypot(wx - self.robot_x, wy - self.robot_y)
                    if dist < self.min_goal_distance:
                        continue

                score = len(cluster) / (dist + 0.01)
                clusters.append((wx, wy, score))

        clusters.sort(key=lambda f: f[2], reverse=True)
        return clusters

    def is_free(self, wx, wy):
        """Check if a world coordinate is in free space on the map."""
        msg  = self.map_data
        res  = msg.info.resolution
        ox   = msg.info.origin.position.x
        oy   = msg.info.origin.position.y
        col  = int((wx - ox) / res)
        row  = int((wy - oy) / res)
        if not (0 <= row < msg.info.height and 0 <= col < msg.info.width):
            return False
        idx = row * msg.info.width + col
        val = msg.data[idx]
        return val == 0  # strictly free (not unknown, not occupied)

    def find_free_near_cluster(self, cluster, ox, oy, res):
        """Find the free cell in the cluster closest to the robot."""
        best   = None
        best_d = float('inf')
        for r, c in cluster:
            wx = ox + (c + 0.5) * res
            wy = oy + (r + 0.5) * res
            if self.is_free(wx, wy):
                d = np.hypot(wx - self.robot_x, wy - self.robot_y)
                if d < best_d:
                    best_d = d
                    best   = (wx, wy)
        return best

    def check_exploration_done(self):
        grid     = np.array(self.map_data.data, dtype=np.int8)
        unknown  = np.sum(grid == -1)
        return (unknown / len(grid)) < self.unknown_threshold

    # ── Nav2 Async Interface ───────────────────────────────────────────────
    def send_nav_goal(self, wx, wy):
        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Nav2 action server not available.')
            self.state = State.FINDING
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id    = 'map'
        goal.pose.header.stamp       = self.get_clock().now().to_msg()
        goal.pose.pose.position.x    = wx
        goal.pose.pose.position.y    = wy
        goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'Navigating to frontier ({wx:.2f}, {wy:.2f})')
        self.nav_start_time   = self.get_clock().now()
        self._current_goal_xy = (wx, wy)

        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2.')
            self._mark_failed_and_find()
            return
        self._goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        status = future.result().status
        wx, wy = self._current_goal_xy
        key    = (round(wx, 1), round(wy, 1))

        if status == 4:  # SUCCEEDED
            self.get_logger().info(f'Reached frontier ({wx:.2f}, {wy:.2f})')
            self.failed_frontiers.discard(key)
        else:
            self.get_logger().warn(
                f'Failed to reach ({wx:.2f}, {wy:.2f}), status={status}')
            self.failed_frontiers.add(key)

        self.state = State.FINDING

    def _mark_failed_and_find(self):
        if self._current_goal_xy:
            wx, wy = self._current_goal_xy
            self.failed_frontiers.add((round(wx, 1), round(wy, 1)))
        self.state = State.FINDING

    # ── State Machine ──────────────────────────────────────────────────────
    def state_machine(self):
        if self.state == State.IDLE or self.map_data is None:
            return

        elif self.state == State.WAITING:
            elapsed = (self.get_clock().now() - self._start_time).nanoseconds / 1e9
            if elapsed >= self.nav2_wait_sec:
                self.get_logger().info('Nav2 ready — starting exploration.')
                self.state = State.FINDING

        elif self.state == State.NAVIGATING:
            if self.nav_start_time is not None:
                elapsed = (self.get_clock().now() - self.nav_start_time).nanoseconds / 1e9
                if elapsed > self.nav_timeout_sec:
                    self.get_logger().warn('Nav2 goal timed out — cancelling.')
                    if self._goal_handle:
                        self._goal_handle.cancel_goal_async()
                    self._mark_failed_and_find()

        elif self.state == State.FINDING:
            if self.check_exploration_done():
                self.state = State.DONE
                self.get_logger().info('Exploration complete! Maze fully mapped.')
                return

            frontiers = self.find_frontiers()
            self.get_logger().info(f'Found {len(frontiers)} valid frontiers.')

            if not frontiers:
                self.state = State.DEAD_END
                return

            target = None
            for fx, fy, fscore in frontiers:
                key = (round(fx, 1), round(fy, 1))
                if key not in self.failed_frontiers:
                    target = (fx, fy)
                    break

            if target is None:
                self.get_logger().warn('All frontiers tried — clearing failed list.')
                self.failed_frontiers.clear()
                self.state = State.DEAD_END
                return

            self.state = State.NAVIGATING
            self.send_nav_goal(*target)

        elif self.state == State.DEAD_END:
            self.get_logger().warn('Dead end — clearing failed frontiers and retrying.')
            self.failed_frontiers.clear()
            self.state = State.FINDING

        elif self.state == State.DONE:
            self.get_logger().info(
                'Maze exploration complete!', throttle_duration_sec=10.0)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()