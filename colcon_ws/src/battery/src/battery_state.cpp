#include <memory>
#include <chrono>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <stonefish_ros2/msg/thruster_state.hpp>
#include "std_msgs/msg/string.hpp"

#include "battery_without_charge/battery.h"
#include "config.h"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class BatteryState : public rclcpp::Node
{
public:
  BatteryState()
  : Node("battery"), count_(0)

  {
    auto topic_callback =
      [this](stonefish_ros2::msg::ThrusterState::SharedPtr msg) -> void {
			for (int i = 0; i<8; i++){
			this->values[i] = msg->thrust[i]/9.81;
			}
		};
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
    500ms, std::bind(&BatteryState::timer_callback, this));
    subscription_ = this->create_subscription<stonefish_ros2::msg::ThrusterState>("/bluerov/controller/thruster_state", 10, topic_callback);
	}


private:
void timer_callback()
	{
		auto message = std_msgs::msg::String();
		message.data = "Hello, world! " + std::to_string(count_++);
		RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
		publisher_->publish(message);
	}
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
	size_t count_;
  rclcpp::Subscription<stonefish_ros2::msg::ThrusterState>::SharedPtr subscription_;
  std::vector<double> values = {0.,0.,0.,0.,0.,0.,0.,0.};
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BatteryState>());
  rclcpp::shutdown();
  return 0;
}
