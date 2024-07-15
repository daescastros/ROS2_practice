#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
// Server functionality includes
#include "example_interfaces/srv/set_bool.hpp"
using std::placeholders::_1;
using std::placeholders::_2;

// This node is intended to subscribe/listen to the number/topic and
// publish the sum of the received numbers to /number_count

// A second functionality has been added: Create a service server, and
// receive a command from clients to reset the counter

class NumberCounterNode : public rclcpp::Node
{
public:
    NumberCounterNode() : Node("number_counter")
    {
        subscriber_ = this->create_subscription<example_interfaces::msg::Int64>("number", 10, std::bind(&NumberCounterNode::callbackNumberCounterTask, this, std::placeholders::_1));
        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);

        // Server functionality from here
        server_ = this->create_service<example_interfaces::srv::SetBool>(
            "reset_counter",
            std::bind(&NumberCounterNode::callbackResetCounter,
                      this, _1, _2));

        RCLCPP_INFO(this->get_logger(), "Number Counter Node has been started.");
    }

    // Server functionality from here

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

    // Server functionality from here
    const char * success_call_;
    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr server_;
    void callbackResetCounter(
        const example_interfaces::srv::SetBool::Request::SharedPtr request,
        const example_interfaces::srv::SetBool::Response::SharedPtr response)
    {
        if (request->data == true)
        {
            count = 0;
            success_call_ = "successful";
            response->success = true;
            response->message = "Message accepted.";
        }
        else if (request->data == false)
        {
            success_call_ = "not successful";
            response->success = false;
            response->message = "Received call with unexpected value. Expected: bool true";
        }
        else
        {
            response->success = false;
            response->message = "Received call with unknown value. Expected: bool true";
        };
        RCLCPP_INFO(
            this->get_logger(),
            "Counter reset was %s", success_call_);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}