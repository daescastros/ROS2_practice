#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

// This node is meant to create a topic/station

class RobotNewsStationNode : public rclcpp::Node
{
public:
    RobotNewsStationNode() : Node("robot_news_station"), robot_name_("R2D2")
    {
        //Node Parameter
        this->declare_parameter("timer_period", 1);
        timer_period_ = this->get_parameter("timer_period").as_int();
        this->declare_parameter("robot_name", "R2D2");
        robot_name_ = this->get_parameter("robot_name").as_string();

        publisher_ = this->create_publisher<example_interfaces::msg::String>("robot_news", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(timer_period_), std::bind(&RobotNewsStationNode::publishNews, this));
        RCLCPP_INFO(this->get_logger(), "Robot News Station has been started.");
    }

private:
    void publishNews()
    {
        auto msg = example_interfaces::msg::String();
        msg.data = std::string("Hi, this is ") + robot_name_ + std::string(" from the Robot News Station.");
        publisher_->publish(msg);
    }

    std::string robot_name_;
    rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int timer_period_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotNewsStationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}