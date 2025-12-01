#include <memory>
#include <chrono>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <stonefish_ros2/msg/thruster_state.hpp>
#include "std_msgs/msg/string.hpp"

#include "battery/battery.h"
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
    1000ms, std::bind(&BatteryState::timer_callback, this));
    subscription_ = this->create_subscription<stonefish_ros2::msg::ThrusterState>("/bluerov/controller/thruster_state", 10, topic_callback);
		for(int i = 0; i < (int)thrust_to_power_data.size(); i++){
			voltage_data.push_back(thrust_to_power_data[i][0]);
			thrust_data.push_back(thrust_to_power_data[i][1]);
			power_data.push_back(thrust_to_power_data[i][2]);
		}
		thrust_to_power = s_init(thrust_data, voltage_data, power_data);
	}



private:
	Battery  battery =  Battery(VMIN * SERIES, VMAX * SERIES, CMAX, CMIN, (double)(R * SERIES / PARA), NC, 8.5 * SERIES * PARA, SERIES, PARA, DISCHARGING_FILENAME);
	std::vector<std::vector<double>> thrust_to_power_data = readCSV("/home/mig/simulator/colcon_ws/src/battery/src/battery/thrust_to_power.csv");
	std::vector<double> voltage_data;
	std::vector<double> thrust_data;
	std::vector<double> power_data;
	surface thrust_to_power;
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
	size_t count_;
  rclcpp::Subscription<stonefish_ros2::msg::ThrusterState>::SharedPtr subscription_;
  std::vector<double> values = {0.,0.,0.,0.,0.,0.,0.,0.};
	std::vector<double> power = {0.,0.,0.,0.,0.,0.,0.,0.};
	void timer_callback()
	{
		double ptot = 50;
		for(int i = 0; i < 8; i++){
			power[i] = interpolate(values[i], battery.U, thrust_to_power);
			ptot += power[i];
		}
		std::cout<<"U : "<< battery.U << ", P : "<<ptot<< ", I : "<< battery.I << ", b : "  << battery.b << std::endl;
		battery.update( -ptot, 1./3600.);

		/*
		auto message = std_msgs::msg::String();
		message.data = "P demandée : " + std::to_string(ptot) ;
		RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
		publisher_->publish(message);
		*/
	}

};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BatteryState>());
  rclcpp::shutdown();
  return 0;
}
