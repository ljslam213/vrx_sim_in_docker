/**
 * @brief Vehicle GPS position uORB topic listener example
 * @file vehicle_global_position_listener.cpp
 * @addtogroup examples
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <px4_msgs/msg/wamv_command.hpp>
// #include <px4_msgs/msg/vehicle_global_position.hpp>
// #include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/wamv_global_position.hpp>
#include <px4_msgs/msg/wamv_attitude.hpp>

/**
 * @brief Vehicle GPS position uORB topic data callback
 */
class VehicleGpsPositionListener : public rclcpp::Node
{
public:
	explicit VehicleGpsPositionListener() : Node("vehicle_global_position_listener")
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		
		wcmd_sub_ = this->create_subscription<px4_msgs::msg::WamvCommand>("/fmu/out/wamv_command", qos,
		[this](const px4_msgs::msg::WamvCommand::UniquePtr msg) {
			std_msgs::msg::Float64 left_thrust;
			left_thrust.data = msg->left[0];

			std_msgs::msg::Float64 left_angle;
			left_angle.data = msg->left[1];

			std_msgs::msg::Float64 right_thrust;
			right_thrust.data = msg->right[0];

			std_msgs::msg::Float64 right_angle;
			right_angle.data = msg->right[1];

			this->left_thrust_pub_->publish(left_thrust);
			this->left_angle_pub_->publish(left_angle);
			this->right_thrust_pub_->publish(right_thrust);
			this->right_angle_pub_->publish(right_angle);

		});
		
		left_thrust_pub_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/left/thrust", 10);
		left_angle_pub_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/left/pos", 10);
		right_thrust_pub_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/right/thrust", 10);
		right_angle_pub_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/right/pos", 10);

		wamv_glb_pos_pub_ = this->create_publisher<px4_msgs::msg::WamvGlobalPosition>("/fmu/in/wamv_global_position", 10);
		wamv_att_pub_ = this->create_publisher<px4_msgs::msg::WamvAttitude>("/fmu/in/wamv_attitude", 10);

		gps_sub_  = this->create_subscription<sensor_msgs::msg::NavSatFix>("/wamv/sensors/gps/gps/fix", 10, [this](const sensor_msgs::msg::NavSatFix::UniquePtr msg) {

			px4_msgs::msg::WamvGlobalPosition wgp;
			wgp.lat = msg->latitude;
			wgp.lon = msg->longitude;

			this->wamv_glb_pos_pub_->publish(wgp);
		});

		imu_sub_  = this->create_subscription<sensor_msgs::msg::Imu>("/wamv/sensors/imu/imu/data", 10, [this](const sensor_msgs::msg::Imu::UniquePtr msg) {
			px4_msgs::msg::WamvAttitude watt;
			watt.q[0] = msg->orientation.x;
			watt.q[1] = msg->orientation.y;
			watt.q[2] = msg->orientation.z;
			watt.q[3] = msg->orientation.w;
			
			this->wamv_att_pub_->publish(watt);
		});

	}

private:
	
	// WamvCommand [px4 --> ros2]
	rclcpp::Subscription<px4_msgs::msg::WamvCommand>::SharedPtr wcmd_sub_;
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_thrust_pub_;
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_angle_pub_;
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_thrust_pub_;
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_angle_pub_;

	// GPS and Imu [ros2 --> px4]
	rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
	rclcpp::Publisher<px4_msgs::msg::WamvGlobalPosition>::SharedPtr wamv_glb_pos_pub_;
	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
	rclcpp::Publisher<px4_msgs::msg::WamvAttitude>::SharedPtr wamv_att_pub_;

};

int main(int argc, char *argv[])
{
	std::cout << "Starting px4_wamv_command listener node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<VehicleGpsPositionListener>());

	rclcpp::shutdown();
	return 0;
}
