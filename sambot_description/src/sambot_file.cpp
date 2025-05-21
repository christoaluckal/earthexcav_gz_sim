#include <functional>
#include <memory>
#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;


class sambotClass : public rclcpp::Node
{
    public:
    sambotClass()
    : Node("sambot_node"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(
        50ms, std::bind(&sambotClass::timer_callback, this));
        subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&sambotClass::topic_callback, this, _1));

        this->declare_parameter("bool_value", rclcpp::PARAMETER_BOOL);
        this->declare_parameter("int_number", rclcpp::PARAMETER_INTEGER);
        this->declare_parameter("double_number", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("str_text", rclcpp::PARAMETER_STRING);


        bool_var = this->get_parameter("bool_value").as_bool();
        int_var = this->get_parameter("int_number").as_int();
        double_var = this->get_parameter("double_number").as_double();
        string_var = this->get_parameter("str_text").as_string();

        count_ = int_var;
    }

    private:
    void topic_callback(const std_msgs::msg::String & msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    size_t count_;

    std::string string_var;
    int int_var;
    double double_var;
    bool bool_var;


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sambotClass>());
  rclcpp::shutdown();
  return 0;
}
