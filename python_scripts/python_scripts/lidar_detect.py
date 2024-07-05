#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point

class SensorChecking(Node):

    def __init__(self):
        super().__init__('sensor_checking')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_cb,
            10)
        self.subscription  # prevent unused variable warning
        self.lidar_pub = self.create_publisher(Point, '/lidar_detect', 1)

    def lidar_cb(self, msg):
        region_a = min(min(msg.ranges[0:120]), 10)
        region_b = min(min(msg.ranges[121:240]), 10)
        region_c = min(min(msg.ranges[241:360]), 10)
        self.get_logger().info(f"A: {round(region_a, 3)} B: {round(region_b, 3)} C: {round(region_c, 3)}")

        # Create and publish the Point message
        point_msg = Point()
        point_msg.x = region_a
        point_msg.y = region_b
        point_msg.z = region_c
        self.lidar_pub.publish(point_msg)

def main(args=None):
    rclpy.init(args=args)
    sensor_checking = SensorChecking()
    rclpy.spin(sensor_checking)

    sensor_checking.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
