#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String 

class ByeWorld(Node):
    def __init__(self):
        super().__init__('bye_world')
        self.get_logger().info(' bye World! Node Started.')
         
        self.declare_parameter("message","Hello, World from parameter!")
        self.msg_str = self.get_parameter("message").get_parameter_value().string_value 
        self.publisher_ = self.create_publisher(String, 'chatter1', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.i = 0
        self.multiple = 0

    def timer_callback(self):
        msg = String()
        msg.data = self.msg_str
        # msg.data = list(['Bye World!', 'See you later!', 'Goodbye!',"apple","bannana","eeee"])[self.i % 6]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}" message send: "{self.i}"')
                                # times, multiple of 5 is: "{self.multiple}"')
        self.i += 1
        self.multiple = self.i * 5

def main(args=None):
    rclpy.init(args=args)
    node = ByeWorld()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
