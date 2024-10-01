#include <memory>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include "rclcpp/rclcpp.hpp"
#include "time_sync/msg_types.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("first_subscriber")
    {
      try {
        std::cout << "Current path: " << rcpputils::fs::current_path() << std::endl;
        std::string share_path = ament_index_cpp::get_package_share_directory("time_sync");
        std::cout << "Share path: " << share_path << std::endl;
        YAML::Node config = YAML::LoadFile(share_path + "/first_subscriber.yaml");
        topic_name = config["input_topic"].as<std::string>();
        std::cout << "Subscribed to: " << topic_name << std::endl;
      } catch (const std::exception& e) {
        std::cerr << "Error loading YAML file: " << e.what() << std::endl;
      }
      subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
      topic_name, 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const geometry_msgs::msg::Pose::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Received pose: [x: %f, y: %f, z: %f]",
                    msg->position.x, msg->position.y, msg->position.z);
    }
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
    std::string topic_name;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}