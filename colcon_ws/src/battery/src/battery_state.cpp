#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <stonefish_ros2/msg/thruster_state.hpp>
#include "std_msgs/msg/string.hpp"

class Thrust : public rclcpp::Node
{
public:
  Thrust()
  : Node("battery")

  {
    auto topic_callback =
      [this](stonefish_ros2::msg::ThrusterState::SharedPtr msg) -> void {
	for (int i = 0; i<8; i++){
	  this->values[i] = msg->thrust[i]/9.81;
	}
      };
    subscription_ = this->create_subscription<stonefish_ros2::msg::ThrusterState>("/bluerov/controller/thruster_state", 10, topic_callback);
  }

  std::vector<double> values = {0.,0.,0.,0.,0.,0.,0.,0.};

private:
  rclcpp::Subscription<stonefish_ros2::msg::ThrusterState>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Thrust>());
  rclcpp::shutdown();
  return 0;
}
