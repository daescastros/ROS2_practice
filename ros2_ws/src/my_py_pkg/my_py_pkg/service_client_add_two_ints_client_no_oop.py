#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

# This node is meant to be created, make a request to
# the server, and be deleted. That's why it doesn't
# use the .spin(node) function


def main(args=None):
    rclpy.init(args=args)
    node = Node("add_two_ints_client_no_oop")

    client = node.create_client(AddTwoInts, "add_two_ints")
    # Check if the service exists, wait 2.0 seconds
    while not client.wait_for_service(2.0):
        node.get_logger().warn("Waiting for server Add Two Ints...")
    request = AddTwoInts.Request()
    request.a = 3
    request.b = 2
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node=node, future=future)
    try:
        response = future.result()
        node.get_logger().info(str(request.a) + " + " + str(request.b) + " = " + str(response.sum))
    except Exception as e:
        # If couldn't get response, print error mesage
        node.get_logger().error("Service call failed %r" % (e,))

    rclpy.shutdown()


if __name__ == "__main__":
    main()
