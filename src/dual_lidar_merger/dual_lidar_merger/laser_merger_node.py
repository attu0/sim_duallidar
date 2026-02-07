#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import tf2_ros
import math


class LaserMerger(Node):

    def __init__(self):
        super().__init__('laser_merger')

        # Subscribers
        self.sub_front = self.create_subscription(
            LaserScan, '/scan_front', self.front_cb, 10)

        self.sub_rear = self.create_subscription(
            LaserScan, '/scan_rear', self.rear_cb, 10)

        # Publisher
        self.pub = self.create_publisher(LaserScan, '/scan', 10)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self)

        self.front = None
        self.rear = None

        # Timer publish at 10 Hz
        self.timer = self.create_timer(0.1, self.merge)

        self.get_logger().info("Dual LiDAR merger started")

    # -------- CALLBACKS --------
    def front_cb(self, msg):
        self.front = msg

    def rear_cb(self, msg):
        self.rear = msg

    # -------- UTILS --------
    def scan_to_xy(self, scan):
        pts = []
        for i, r in enumerate(scan.ranges):

            if math.isinf(r) or math.isnan(r):
                continue

            ang = scan.angle_min + i * scan.angle_increment
            x = r * math.cos(ang)
            y = r * math.sin(ang)

            pts.append((x, y))

        return pts

    def transform_xy(self, pts, source_frame):

        try:
            tf = self.tf_buffer.lookup_transform(
                "base_link",
                source_frame,
                rclpy.time.Time())

            tx = tf.transform.translation.x
            ty = tf.transform.translation.y

            q = tf.transform.rotation
            yaw = math.atan2(
                2*(q.w*q.z + q.x*q.y),
                1 - 2*(q.y*q.y + q.z*q.z)
            )

            out = []
            for x, y in pts:

                # rotate
                xr = x*math.cos(yaw) - y*math.sin(yaw)
                yr = x*math.sin(yaw) + y*math.cos(yaw)

                # translate
                xr += tx
                yr += ty

                out.append((xr, yr))

            return out

        except:
            return []

    # -------- MERGE --------
    def merge(self):

        if self.front is None or self.rear is None:
            return

        f_pts = self.scan_to_xy(self.front)
        r_pts = self.scan_to_xy(self.rear)

        f_pts = self.transform_xy(
            f_pts, self.front.header.frame_id)

        r_pts = self.transform_xy(
            r_pts, self.rear.header.frame_id)

        all_pts = f_pts + r_pts

        # Build merged scan
        merged = LaserScan()

        merged.header.stamp = self.get_clock().now().to_msg()

        # publish in lidar-height frame
        merged.header.frame_id = "laser_merged"

        merged.angle_min = -math.pi
        merged.angle_max = math.pi
        merged.angle_increment = math.radians(1.0)

        merged.range_min = 0.05
        merged.range_max = 12.0

        bins = int(
            (merged.angle_max - merged.angle_min) /
            merged.angle_increment)

        ranges = [float('inf')] * bins

        # Fill bins
        for x, y in all_pts:

            r = math.hypot(x, y)
            a = math.atan2(y, x)

            idx = int(
                (a - merged.angle_min) /
                merged.angle_increment)

            if 0 <= idx < bins and r < ranges[idx]:
                ranges[idx] = r

        # Replace inf
        merged.ranges = [
            r if r != float('inf')
            else merged.range_max
            for r in ranges
        ]

        self.pub.publish(merged)


def main():
    rclpy.init()
    node = LaserMerger()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
