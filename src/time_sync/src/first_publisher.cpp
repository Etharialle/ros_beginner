#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "time_sync/msg_types.hpp"
#include "rclcpp/rclcpp.hpp"


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("first_publisher"), count_(0)
    {
      try {
        std::cout << "Current path: " << rcpputils::fs::current_path() << std::endl;
        std::string share_path = ament_index_cpp::get_package_share_directory("time_sync");
        std::cout << "Share path: " << share_path << std::endl;
        YAML::Node config = YAML::LoadFile(share_path + "/first_publisher.yaml");
        topic_name = config["output_topic"].as<std::string>();
      } catch (const std::exception& e) {
        std::cerr << "Error loading YAML file: " << e.what() << std::endl;
      }
      publisher_ = this->create_publisher<geometry_msgs::msg::Pose>(topic_name, 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::pose_updater, this));
    }

  private:

    void pose_updater()
    {
      auto message = geometry_msgs::msg::Pose();
        
        // Set the position
        message.position.x = count_;
        message.position.y = count_ + 1.0;
        message.position.z = 0.0;

        // Set the orientation (as a quaternion)
        message.orientation.x = 0.0;
        message.orientation.y = 0.0;
        message.orientation.z = 0.0;
        message.orientation.w = 1.0; // No rotation

        RCLCPP_INFO(this->get_logger(), "Publishing pose: [x: %f, y: %f, z: %f]", 
                    message.position.x, message.position.y, message.position.z);
        
        publisher_->publish(message);
        count_++;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
    //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
    std::string topic_name;
    //std::vector<std::string> output_topics_; 

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}