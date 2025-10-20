#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node

class hello_world(Node):
    def __init__(self):
        super().__init__('hello_world')
        self.get_logger().info('Hello, World!')

def main(args=None):
    rclpy.init(args=args)
    node = hello_world()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()