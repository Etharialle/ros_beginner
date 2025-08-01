#include <gtest/gtest.h>
#include <memory>
#include <chrono>
#include <vector>
#include <string>
#include <tuple>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "include/ese.hpp"

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

// Verifies that the publisher was created on the correct topic.
// This test is still valuable as it checks the constructor.
TEST_F(TalkerTest, TestPublisherIsCreated) {
    // ARRANGE
    auto talker_node = std::make_shared<Talker>();

    // ACT & ASSERT
    size_t publisher_count = talker_node->count_publishers("/perception/ese/ego_state");
    EXPECT_EQ(publisher_count, 1);
}

// ---- MC/DC Test Suite ----

// Define a struct to hold the parameters for our test cases
struct MCDCTestParams {
    bool is_active;
    bool data_is_valid;
    bool manual_override;
    bool should_publish;
    std::string test_name; // For clear test output
};

// Create a new test fixture that is parameterized with our struct
class TalkerMCDCTest : public TalkerTest,
                       public ::testing::WithParamInterface<MCDCTestParams> {};

// The parameterized test case
TEST_P(TalkerMCDCTest, PublishingLogic) {
    // ARRANGE
    auto params = GetParam(); // Get the parameters for this specific test run
    auto talker_node = std::make_shared<Talker>();
    auto subscriber_node = rclcpp::Node::make_shared("test_subscriber");

    // Set the state of the talker based on the test parameters
    talker_node->set_active(params.is_active);
    talker_node->set_data_valid(params.data_is_valid);
    talker_node->set_manual_override(params.manual_override);

    bool message_received = false;
    auto sub = subscriber_node->create_subscription<std_msgs::msg::String>(
        "/perception/ese/ego_state", 10,
        [&](const std_msgs::msg::String&) {
            message_received = true;
        });

    // ACT
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(talker_node);
    executor.add_node(subscriber_node);

    // Spin for a short duration to allow the timer to fire at least once
    executor.spin_for(std::chrono::milliseconds(600));

    // ASSERT
    ASSERT_EQ(message_received, params.should_publish)
        << "Test failed for case: " << params.test_name;
}

// Instantiate the test suite with our set of MC/DC cases
INSTANTIATE_TEST_SUITE_P(
    MCDC,
    TalkerMCDCTest,
    ::testing::Values(
        MCDCTestParams{true,  true,  false, true,  "Active_Valid_NoOverride_Publishes"}, // Pair for A (active) & B (valid)
        MCDCTestParams{false, true,  false, false, "Inactive_Valid_NoOverride_NoPublish"}, // Pair for A (active)
        MCDCTestParams{true,  false, false, false, "Active_Invalid_NoOverride_NoPublish"}, // Pair for B (valid) & C (override)
        MCDCTestParams{true,  false, true,  true,  "Active_Invalid_Override_Publishes"}  // Pair for C (override)
    ),
    // This lambda function generates human-readable test names
    [](const ::testing::TestParamInfo<MCDCTestParams>& info) {
        return info.param.test_name;
    }
);