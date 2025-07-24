#ifndef INCLUDE__ESE_HPP_
#define INCLUDE__ESE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Talker : public rclcpp::Node {
public:
    // Make the constructor public so the test can create an instance
    Talker();

private:
    void publish_message();

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif 