#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from activity_04_interfaces.srv import SetLed


# This node is meant to change state between charged and discharged with a time period.
# When discarged, will ask to the server to turn an LED on.
# When charged, will ask the server to turn of the LED.


class BatteryNode(Node):
    def __init__(self):
        super().__init__("battery_node")
        self.get_logger().info("Battery Node has been started.")
        self.battery_is_charged_status_ = True
        self.state_ = "off"
        self.led_number_ = 3
        self.timer_ = self.create_timer(2.0, self.callback_change_led_state_client)
        """
        # Segunda opcion de timer:
        # Medicion del tiempo para crear dos tareas separadas por tiempo:
        self.battery_state_ = "full"
        self.last_time_battery_state_changed_ = self.get_current_time_seconds()
        self.battery_timer_ = self.create_timer(0.1, self.check_battery_state)

    def get_current_time_seconds(self):
        secs, nanosecs = self.get_clock().now().seconds_nanoseconds()
        return secs + nanosecs / 1000000000.0

    def check_battery_state(self):
        time_now = self.get_current_time_seconds()
        if self.battery_state_ == "full":
            if time_now - self.last_time_battery_state_changed_ > 4.0:
                self.battery_state_ = "empty"
                self.get_logger().info("Battery is empty! Charging battery...")
                self.last_time_battery_state_changed_ = time_now
                self.callback_change_led_state_client(self.battery_state_)
        else:
            if time_now - self.last_time_battery_state_changed_ > 6.0:
                self.battery_state_ = "full"
                self.get_logger().info("Battery is full! Not charging battery...")
                self.last_time_battery_state_changed_ = time_now
                self.callback_change_led_state_client(self.battery_state_)
    
    def callback_change_led_state_client(self, battery_state):
        if battery_state == 'full'
            self.battery_is_charged_status_ = True
        elif battery_state == 'empty'
            self.battery_is_charged_status_ = False
        client = self.create_client(SetLed, "leds_state_service")
        while not client.wait_for_service(2.0):
            self.get_logger().warn("Waiting for server LEDs State...")
        if self.battery_is_charged_status_:
            self.state_ = "off"
        else:
            self.state_ = "on"
        request = SetLed.Request()
        request.led_number = self.led_number_
        request.state = self.state_
        future = client.call_async(request)
        future.add_done_callback(self.callback_print_confirm)
        """

    def callback_change_led_state_client(self):
        self.battery_is_charged_status_ = not self.battery_is_charged_status_
        client = self.create_client(SetLed, "leds_state_service")
        while not client.wait_for_service(2.0):
            self.get_logger().warn("Waiting for server LEDs State...")
        if self.battery_is_charged_status_:
            self.state_ = "off"
        else:
            self.state_ = "on"
        request = SetLed.Request()
        request.led_number = self.led_number_
        request.state = self.state_
        future = client.call_async(request)
        future.add_done_callback(self.callback_print_confirm)

    def callback_print_confirm(self, future):
        try:
            response = future.result()
            self.get_logger().info(
                "Battery is charged: "
                + str(self.battery_is_charged_status_)
                + "\n"
                + "LED number: "
                + str(self.led_number_)
                + "\n"
                + "State: "
                + self.state_
                + "\n"
                + "Success response: "
                + str(response.success)
            )
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
