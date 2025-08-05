#ifndef INCLUDE__ESE_HPP_
#define INCLUDE__ESE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>

class Talker : public rclcpp::Node {
public:
    // Make the constructor public so the test can create an instance
    Talker();
    void set_active(bool active);
    void set_data_valid(bool valid);
    void set_manual_override(bool override);


private:



    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool is_active_;
    bool data_is_valid_;
    bool manual_override_;

};

#endif 