#include <gtest/gtest.h>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "include/ese.hpp" // Include the class we want to test

// Test Fixture for initializing and shutting down ROS
class TalkerTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
    }

    void TearDown() override {
        rclcpp::shutdown();
    }
};

// Verifies that the talker node publishes the correct message on its topic.
TEST_F(TalkerTest, TestMessageIsPublished) {
    // ARRANGE
    auto talker_node = std::make_shared<Talker>();
    auto subscriber_node = rclcpp::Node::make_shared("test_subscriber");
    bool message_received = false;
    std_msgs::msg::String received_message;

    auto sub = subscriber_node->create_subscription<std_msgs::msg::String>(
        "/perception/ese/ego_state", 10,
        [&](const std_msgs::msg::String& msg) {
            message_received = true;
            received_message = msg;
        });

    // ACT
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(talker_node);
    executor.add_node(subscriber_node);

    std::promise<bool> dummy_promise;
    std::shared_future<bool> dummy_future = dummy_promise.get_future();
    executor.spin_until_future_complete(dummy_future, std::chrono::seconds(1));

    // ASSERT
    ASSERT_TRUE(message_received) << "Test failed: No message was received on the topic.";
    EXPECT_EQ(received_message.data, "Ego State");
}

// Verifies that the publisher was created on the correct topic.
TEST_F(TalkerTest, TestPublisherIsCreated) {
    // ARRANGE
    auto talker_node = std::make_shared<Talker>();

    // ACT & ASSERT
    size_t publisher_count = talker_node->count_publishers("/perception/ese/ego_state");
    EXPECT_EQ(publisher_count, 1);
}