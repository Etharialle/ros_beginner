#include "include/ese.hpp"

Talker::Talker() : Node("ese") {
    publisher_ = this->create_publisher<std_msgs::msg::String>("/perception/ese/ego_state", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&Talker::publish_message, this)
    );
}

void Talker::publish_message() {
    auto message = std_msgs::msg::String();
    message.data = "Ego State";
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
}