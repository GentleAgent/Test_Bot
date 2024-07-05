#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool 

class LaserReadingNode(Node):
    def __init__(self):
        super().__init__('reading_laser')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.sub = self.create_subscription(LaserScan, '/scan', self.clbk_laser, 1)
        self.lidar_pub = self.create_publisher(Bool, '/lidar_value', 1)

    def clbk_laser(self, msg):
        ranges_len = len(msg.ranges)
        
        regions = {
            'right': min(min(msg.ranges[0:143]), 10) if ranges_len > 143 else 10,
            'fright': min(min(msg.ranges[144:287]), 10) if ranges_len > 287 else 10,
            'front': min(min(msg.ranges[288:431]), 10) if ranges_len > 431 else 10,
            'fleft': min(min(msg.ranges[432:575]), 10) if ranges_len > 575 else 10,
            'left': min(min(msg.ranges[576:713]), 10) if ranges_len > 713 else 10,
        }
        self.take_action(regions)

    def take_action(self, regions):
        msg = Twist()
        linear_x = 0.0
        angular_z = 0.0

        state_description = ''

        if regions['front'] > 1 and regions['fleft'] > 1 and regions['fright'] > 1:
            state_description = 'case 1 - nothing'
            linear_x = 0.4
            angular_z = 0.0
        elif regions['front'] < 1 and regions['fleft'] > 1 and regions['fright'] > 1:
            state_description = 'case 2 - front'
            linear_x = 0.0
            angular_z = -0.3
        elif regions['front'] > 1 and regions['fleft'] > 1 and regions['fright'] < 1:
            state_description = 'case 3 - fright'
            linear_x = 0.0
            angular_z = -0.3
        elif regions['front'] > 1 and regions['fleft'] < 1 and regions['fright'] > 1:
            state_description = 'case 4 - fleft'
            linear_x = 0.0
            angular_z = 0.3
        elif regions['front'] < 1 and regions['fleft'] > 1 and regions['fright'] < 1:
            state_description = 'case 5 - front and fright'
            linear_x = 0.0
            angular_z = -0.3
        elif regions['front'] < 1 and regions['fleft'] < 1 and regions['fright'] > 1:
            state_description = 'case 6 - front and fleft'
            linear_x = 0.0
            angular_z = 0.3
        elif regions['front'] < 1 and regions['fleft'] < 1 and regions['fright'] < 1:
            state_description = 'case 7 - front and fleft and fright'
            linear_x = 0.0
            angular_z = -0.3
        elif regions['front'] > 1 and regions['fleft'] < 1 and regions['fright'] < 1:
            state_description = 'case 8 - fleft and fright'
            linear_x = 0.0
            angular_z = -0.3
        else:
            state_description = 'unknown case'
            self.get_logger().info(str(regions))

        self.get_logger().info(state_description)
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.pub.publish(msg)
        self.connect(state_description)  

    def connect(self, state_description):
        lidar_msg = Bool()
        
        if state_description == 'case 1 - nothing':
            lidar_msg.data = False
        else:
            lidar_msg.data = True
        
        self.get_logger().info(f"Publishing LIDAR value: {lidar_msg.data}")
        self.lidar_pub.publish(lidar_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaserReadingNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
