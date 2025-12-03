#include <memory>
#include <chrono>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <stonefish_ros2/msg/thruster_state.hpp>
#include "battery/msg/current_infos.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/battery_state.hpp"


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
			this->values[i] = msg->thrust[i]/9.81*(-1);
			}
		};
    state_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>("/battery/battery_state", 10);
		current_pub_ = this->create_publisher<battery::msg::CurrentInfos>("/battery/current_infos", 10);
		ardu_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/battery/ardu_mon", 10);
    timer_ = this->create_wall_timer(
    10ms, std::bind(&BatteryState::timer_callback, this));
    subscription_ = this->create_subscription<stonefish_ros2::msg::ThrusterState>("/bluerov/controller/thruster_state", 10, topic_callback);
		for(int i = 0; i < (int)thrust_to_power_data.size(); i++){
			voltage_data.push_back(thrust_to_power_data[i][0]);
			thrust_data.push_back(thrust_to_power_data[i][1]);
			power_data.push_back(thrust_to_power_data[i][2]);
			rpm_data.push_back(thrust_to_power_data[i][3]);
		}
		thrust_to_power = s_init(thrust_data, voltage_data, power_data);
		thrust_to_rpm = s_init(thrust_data, voltage_data, rpm_data);
	}



private:
	Battery battery = Battery(VMIN * SERIES, VMAX * SERIES, CMAX, CMIN, (double)(R * SERIES / PARA), NC, 8.75 * SERIES * PARA, SERIES, PARA, DISCHARGING_FILENAME);
	std::vector<std::vector<double>> thrust_to_power_data = readCSV("/home/mig/simulator/colcon_ws/src/battery/src/battery/thrust_to_power.csv");
	std::vector<double> voltage_data;
	std::vector<double> thrust_data;
	std::vector<double> power_data;
	std::vector<double> rpm_data;
	surface thrust_to_power;
	surface thrust_to_rpm;
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr state_pub_;
	rclcpp::Publisher<battery::msg::CurrentInfos>::SharedPtr current_pub_;	
	rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr ardu_pub_;
	size_t count_;
  rclcpp::Subscription<stonefish_ros2::msg::ThrusterState>::SharedPtr subscription_;
  std::vector<double> values = {0.,0.,0.,0.,0.,0.,0.,0.};
	std::vector<double> power = {0.,0.,0.,0.,0.,0.,0.,0.};
	std::vector<double> rpm = {0.,0.,0.,0.,0.,0.,0.,0.};
	void timer_callback()
	{
		double ptot = 50;
		for(int i = 0; i < 8; i++){
			power[i] = interpolate(values[i], battery.U, thrust_to_power);
			ptot += power[i];
			rpm[i] = interpolate(values[i], battery.U, thrust_to_rpm);
		}

		battery.update( -ptot, 1./360000.);

		auto message = sensor_msgs::msg::BatteryState();
		message.voltage = battery.U;
		message.temperature = NAN;
		message.current = battery.I;
		message.charge = battery.b/battery.U;
		message.capacity = NAN;
		message.design_capacity = NC*PARA;
		message.percentage = battery.b/(battery.U*NC*PARA);
		message.power_supply_status = 2;
		message.power_supply_health = 0;
		message.power_supply_technology = 2;
		message.present = true;
		for(int i = 0; i < SERIES; i++){
			message.cell_voltage.push_back(battery.U/SERIES);
			message.cell_temperature.push_back(NAN);
		}
		message.location = LOCATION;
		message.serial_number = SERIAL_NUMBER;
		message.voltage = battery.U;
		message.current = battery.I;
		message.charge = battery.b/battery.U;
		message.percentage = battery.b/(battery.U*NC*PARA);
		for(int i = 0; i < SERIES; i++){
			message.cell_voltage[i] = battery.U/SERIES;
		}

		// Concaténation finale
		std::string output = "U : " + std::to_string(message.voltage) +
											 ", I : " + std::to_string(message.current) +
											 ", charge : " + std::to_string(message.charge); 

		RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", output.c_str());
		state_pub_->publish(message);

		auto current_msg = battery::msg::CurrentInfos();
		current_msg.battery_current = -battery.I;
		for(int i = 0; i < 8; i++){
			current_msg.motor_current.push_back(-battery.I * power[i] / ptot);
			//current_msg.motor_current.push_back(values[i]);
			current_msg.motor_rpm.push_back(rpm[i]);
		}
		
		current_pub_->publish(current_msg);


		auto ardu_msg = std_msgs::msg::Float32MultiArray();
		ardu_msg.data.push_back(-battery.I);
		ardu_msg.data.push_back(battery.U);
		ardu_pub_->publish(ardu_msg);
	}

};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BatteryState>());
  rclcpp::shutdown();
  return 0;
}
