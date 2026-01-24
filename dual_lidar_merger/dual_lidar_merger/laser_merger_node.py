#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LaserMerger(Node):
    def __init__(self):
        super().__init__('laser_merger')
        
        # Subscribers
        self.sub_front = self.create_subscription(
            LaserScan, '/scan_front', self.front_callback, 10)
        self.sub_rear = self.create_subscription(
            LaserScan, '/scan_rear', self.rear_callback, 10)
        
        # Publisher
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        
        # Storage for latest scans
        self.front_scan = None
        self.rear_scan = None
        
        self.get_logger().info('Laser Merger Node Started')
    
    def front_callback(self, msg):
        self.front_scan = msg
        self.merge_and_publish()
    
    def rear_callback(self, msg):
        self.rear_scan = msg
        self.merge_and_publish()
    
    def merge_and_publish(self):
        if self.front_scan is None or self.rear_scan is None:
            return
        
        # Create merged scan
        merged = LaserScan()
        merged.header = self.front_scan.header
        merged.header.frame_id = 'chassis'

        # Use front scan parameters
        merged.angle_min = -math.pi
        merged.angle_max = math.pi
        merged.angle_increment = self.front_scan.angle_increment
        merged.time_increment = self.front_scan.time_increment
        merged.scan_time = self.front_scan.scan_time
        merged.range_min = min(self.front_scan.range_min, self.rear_scan.range_min)
        merged.range_max = max(self.front_scan.range_max, self.rear_scan.range_max)
        
        # Calculate number of points
        num_points = int((merged.angle_max - merged.angle_min) / merged.angle_increment)
        
        # Initialize with max range
        merged.ranges = [merged.range_max] * num_points
        merged.intensities = [0.0] * num_points
        
        # Add front scan data
        for i, r in enumerate(self.front_scan.ranges):
            angle = self.front_scan.angle_min + i * self.front_scan.angle_increment
            index = int((angle - merged.angle_min) / merged.angle_increment)
            if 0 <= index < num_points:
                merged.ranges[index] = r
                if len(self.front_scan.intensities) > i:
                    merged.intensities[index] = self.front_scan.intensities[i]
        
        # Add rear scan data (offset by 180 degrees)
        for i, r in enumerate(self.rear_scan.ranges):
            angle = self.rear_scan.angle_min + i * self.rear_scan.angle_increment + math.pi
            # Normalize angle to [-pi, pi]
            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi
            
            index = int((angle - merged.angle_min) / merged.angle_increment)
            if 0 <= index < num_points:
                # Take minimum range if both sensors see something
                if merged.ranges[index] == merged.range_max or r < merged.ranges[index]:
                    merged.ranges[index] = r
                    if len(self.rear_scan.intensities) > i:
                        merged.intensities[index] = self.rear_scan.intensities[i]
        
        self.publisher.publish(merged)

def main(args=None):
    rclpy.init(args=args)
    node = LaserMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()