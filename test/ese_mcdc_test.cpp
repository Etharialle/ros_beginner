#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "include/ese.hpp"  // adjust include path as needed

class TalkerTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        node_ = std::make_shared<Talker>();
    }

    void TearDown() override {
        rclcpp::shutdown();
    }

    std::shared_ptr<Talker> node_;
};

TEST_F(TalkerTest, A0B0C0_NoPublish) {
    node_->set_active(false);
    node_->set_data_valid(false);
    node_->set_manual_override(false);
    node_->publish_message();  // Expect no publish (warn log)
}

TEST_F(TalkerTest, A1B0C0_NoPublish) {
    node_->set_active(true);
    node_->set_data_valid(false);
    node_->set_manual_override(false);
    node_->publish_message();  // No publish
}

TEST_F(TalkerTest, A1B1C0_Publish) {
    node_->set_active(true);
    node_->set_data_valid(true);
    node_->set_manual_override(false);
    node_->publish_message();  // Publish
}

TEST_F(TalkerTest, A1B0C1_Publish) {
    node_->set_active(true);
    node_->set_data_valid(false);
    node_->set_manual_override(true);
    node_->publish_message();  // Publish
}

TEST_F(TalkerTest, A0B1C1_NoPublish) {
    node_->set_active(false);
    node_->set_data_valid(true);
    node_->set_manual_override(true);
    node_->publish_message();  // No publish
}

TEST_F(TalkerTest, A1B1C1_Publish) {
    node_->set_active(true);
    node_->set_data_valid(true);
    node_->set_manual_override(true);
    node_->publish_message();  // Publish
}
