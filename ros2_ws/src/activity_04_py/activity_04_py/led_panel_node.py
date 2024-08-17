#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from activity_04_interfaces.srv import SetLed
from activity_04_interfaces.msg import LedPanelState


class LedPanelNode(Node):
    def __init__(self):
        super().__init__("led_panel_server_node")
        self.leds_panel_ = [0, 0, 0]
        self.server_ = self.create_service(
            SetLed, "leds_state_service", self.callback_change_led_state
        )
        self.publisher_ = self.create_publisher(LedPanelState, "led_panel_status", 10)
        self.publish_timer_ = self.create_timer(1.0, self.publish_panel_status)
        self.get_logger().info("LED Panel Server Node has been started.")

    def callback_change_led_state(self, request, response):
        if request.state == "off":
            state_bool = 0
        elif request.state == "on":
            state_bool = 1
        else:
            response.success = False
            self.get_logger().info(
                "LED: "
                + str(request.led_number)
                + "\nState: "
                + str(request.state)
                + "\nSuccess: "
                + str(response.success)
            )
            return response
        self.leds_panel_[request.led_number - 1] = state_bool
        response.success = True
        self.get_logger().info(
            "LED: "
            + str(request.led_number)
            + "\nState: "
            + str(request.state)
            + "\nSuccess: "
            + str(response.success)
        )
        return response
    
    def publish_panel_status(self):
        msg = LedPanelState()
        msg.led1 = self.leds_panel_[0]
        msg.led2 = self.leds_panel_[1]
        msg.led3 = self.leds_panel_[2]
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LedPanelNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
