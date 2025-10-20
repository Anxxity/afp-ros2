#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node

class bye_world(Node):
    def __init__(self):
        super().__init__('bye_world')
        self.get_logger().info('bye, World!')

def main(args=None):
    rclpy.init(args=args)
    node = hello_world()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()