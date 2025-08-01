#include "include/ese.hpp"

// Constructor is unchanged
Talker::Talker() : Node("ese"), is_active_(false), data_is_valid_(false), manual_override_(false) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("/perception/ese/ego_state", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&Talker::publish_message, this)
    );
}

// Methods to control the state for testing
void Talker::set_active(bool active) {
    is_active_ = active;
}

void Talker::set_data_valid(bool valid) {
    data_is_valid_ = valid;
}

void Talker::set_manual_override(bool override) {
    manual_override_ = override;
}


void Talker::publish_message() {
    // This is our complex decision for MC/DC
    // A = is_active_
    // B = data_is_valid_
    // C = manual_override_
    // Decision: A && (B || C)
    if (is_active_ && (data_is_valid_ || manual_override_)) {
        auto message = std_msgs::msg::String();
        message.data = "Ego State Test";
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    } else {
        RCLCPP_WARN(this->get_logger(), "Conditions not met for publishing. [active: %d, valid: %d, override: %d]", is_active_, data_is_valid_, manual_override_);
    }
}