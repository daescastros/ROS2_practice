#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class MyCustomNode(Node):  # Modify name
    def __init__(self):
        super().__init__("node_name")  # Modify name
        self.get_logger().info("My Custom Node has been started.")


def main(args=None):
    rclpy.init(args=args)
    node = MyCustomNode()  # Modify name
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
