#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args)
    node = Node("py_test")
    node.get_logger().info("Hello ROS2")
    rclpy.spin(node) # keep the node running until it is shutdown
    rclpy.shutdown() # shutdown the ROS client library for Python

if __name__ == "__main__":
    main()