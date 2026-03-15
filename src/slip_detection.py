#!/usr/bin/env python3
"""
Slip Detector Node
------------------
Compares consecutive LiDAR scans to detect wheel slip.
If the scan hasn't changed (robot hasn't moved) but odom says it's moving,
publishes the odom with the cached pose (last known good position) but
zeroed velocity — so EKF knows where we are but not that we're "moving".

Subscribes:
  /scan                              (sensor_msgs/LaserScan)
  /diff_drive_base_controller/odom   (nav_msgs/Odometry)

Publishes:
  /odom/corrected                    (nav_msgs/Odometry)
"""

import rclpy
from rclpy.node import Node
import numpy as np
from collections import deque

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class SlipDetector(Node):
    def __init__(self):
        super().__init__('slip_detector')

        # ── Tunable parameters ──────────────────────────────────────────────
        # Mean absolute difference between scans below this → robot hasn't moved
        self.declare_parameter('scan_diff_threshold', 0.005)   # metres

        # Velocity above this is considered "odom thinks we're moving"
        self.declare_parameter('velocity_threshold', 0.005)   # m/s

        # Minimum valid scan returns required (avoids false positives in open space)
        self.declare_parameter('min_valid_returns', 20)

        # How many consecutive "stuck" scan detections before we trigger slip
        self.declare_parameter('stuck_count_threshold', 3)

        # How many odom frames to cache (we use oldest pose on slip)
        self.declare_parameter('odom_cache_size', 10000)
        # ────────────────────────────────────────────────────────────────────

        self.scan_diff_threshold   = self.get_parameter('scan_diff_threshold').value
        self.velocity_threshold    = self.get_parameter('velocity_threshold').value
        self.min_valid_returns     = self.get_parameter('min_valid_returns').value
        self.stuck_count_threshold = self.get_parameter('stuck_count_threshold').value
        self.odom_cache_size       = self.get_parameter('odom_cache_size').value

        self.prev_scan   = None
        self.stuck_count = 0
        self.is_stuck    = False

        # Ring buffer of last N odom messages (good frames only)
        self.odom_cache: deque[Odometry] = deque(maxlen=self.odom_cache_size)

        # Subscriptions
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 15)
        self.odom_sub = self.create_subscription(
            Odometry, '/diff_drive_base_controller/odom', self.odom_callback, 15)

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom/corrected', 15)

        self.get_logger().info('SlipDetector node started')

    # ── LiDAR callback ────────────────────────────────────────────────────
    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)

        # Replace inf/nan with 0 for comparison
        ranges = np.where(np.isfinite(ranges), ranges, 0.0)

        if self.prev_scan is None:
            self.prev_scan = ranges
            return

        # Only compare where BOTH scans had valid returns
        both_valid = (ranges > 0) & (self.prev_scan > 0)
        n_both     = np.count_nonzero(both_valid)

        if n_both < self.min_valid_returns:
            # Not enough data — don't flag as stuck
            self.stuck_count = 0
            self.is_stuck    = False
            self.prev_scan   = ranges
            return

        mean_diff = np.mean(np.abs(ranges[both_valid] - self.prev_scan[both_valid]))

        if mean_diff < self.scan_diff_threshold:
            self.stuck_count += 1
        else:
            self.stuck_count = 0
            self.is_stuck    = False  # environment changed → no longer stuck

        if self.stuck_count >= self.stuck_count_threshold:
            self.is_stuck = True

        self.prev_scan = ranges

    # ── Odometry callback ─────────────────────────────────────────────────
    def odom_callback(self, msg: Odometry):
        vx     = msg.twist.twist.linear.x
        moving = abs(vx) > self.velocity_threshold

        slip_detected = self.is_stuck and moving

        if slip_detected:
            corrected = Odometry()

            # Restore last known good pose from cache, or fall back to current
            if len(self.odom_cache) == self.odom_cache_size:
                corrected.pose = self.odom_cache[0].pose  # oldest cached pose
            else:
                corrected.pose = msg.pose  # cache not full yet, use current

            # Zero out the entire twist — robot is not moving
            corrected.twist.twist.linear.x  = 0.0
            corrected.twist.twist.linear.y  = 0.0
            corrected.twist.twist.linear.z  = 0.0
            corrected.twist.twist.angular.x = 0.0
            corrected.twist.twist.angular.y = 0.0
            corrected.twist.twist.angular.z = 0.0

            # Preserve header and frame ids
            corrected.header         = msg.header
            corrected.child_frame_id = msg.child_frame_id

            self.get_logger().warn(
                f'Slip detected! odom vx={vx:.3f} m/s but scan unchanged '
                f'({self.stuck_count} frames). Publishing cached pose + zero velocity.',
                throttle_duration_sec=1.0
            )
            self.odom_pub.publish(corrected)

        else:
            # No slip — cache this good frame and publish as-is
            self.odom_cache.append(msg)
            self.odom_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SlipDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()