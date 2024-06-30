#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

// This node is intended to subscribe/listen to the robot_news_station/topic

class NumberCounterNode : public rclcpp::Node
{
public:
    NumberCounterNode() : Node("number_counter")
    {
        subscriber_ = this->create_subscription<example_interfaces::msg::Int64>("number", 10, std::bind(&NumberCounterNode::callbackNumberCounterTask, this, std::placeholders::_1));
        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);
        RCLCPP_INFO(this->get_logger(), "Number Counter Node has been started.");
    }

private:
    void callbackNumberCounterTask(const example_interfaces::msg::Int64::SharedPtr msg)
    {
        
        count = count + msg->data;
        auto count_msg = example_interfaces::msg::Int64();
        count_msg.data = count;
        publisher_->publish(count_msg);
        // RCLCPP_INFO(this->get_logger(), "%ld", msg->data); // commented to only publish and not print
    }

    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    int64_t count = 0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}