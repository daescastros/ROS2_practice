#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/hardware_status.hpp"

// This node is meant to create a topic/station with a custom msg interface

class HardwareStatusPublisherNode : public rclcpp::Node
{
public:
    HardwareStatusPublisherNode() : Node("hardware_status_publisher")
    {
        publisher_ = this->create_publisher<my_robot_interfaces::msg::HardwareStatus>("hardware_status", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&HardwareStatusPublisherNode::publishNews, this));
        RCLCPP_INFO(this->get_logger(), "Hardware Status Publisher Node has been started.");
    }

private:
    void publishNews()
    {
        auto msg = my_robot_interfaces::msg::HardwareStatus();
        msg.temperature = 35;
        msg.are_motors_ready = false;
        msg.debug_message = std::string("Hi, this is my first custom message interface");
        publisher_->publish(msg);
    }

    rclcpp::Publisher<my_robot_interfaces::msg::HardwareStatus>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HardwareStatusPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}