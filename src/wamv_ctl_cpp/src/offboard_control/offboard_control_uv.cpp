
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/actuator_servos.hpp>

#include <geometry_msgs/msg/twist.hpp>

#include <chrono>
#include <iostream>
#include <memory>

#include <cmath>
#include <Eigen/Eigen>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using std::placeholders::_1;


// two helper functions
double constrain(double cur_val, double min_val, double max_val) {
    return (cur_val < min_val) ? min_val : (cur_val > max_val) ? max_val : cur_val;
}

Eigen::Vector2d computeThrustVector(double Fx, double Fy) {
    
    Eigen::Vector2d force(Fx, Fy);
    
    double thrust_magnitude = force.norm();
    double thrust_angle = (thrust_magnitude > 0.01) ? std::atan2(Fy, Fx) : 0.0;

    if (thrust_angle > M_PI / 2) {
        thrust_angle -= M_PI;
        thrust_magnitude = -thrust_magnitude;
    } else {
        thrust_angle += M_PI;
        thrust_magnitude = -thrust_magnitude;
    }

    return Eigen::Vector2d(thrust_magnitude, thrust_angle);
}

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("uv_control")
	{
        
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

        actuator_motors_publisher_ = this->create_publisher<ActuatorMotors>("/fmu/in/actuator_motors", 10);
        actuator_servos_publisher_ = this->create_publisher<ActuatorServos>("/fmu/in/actuator_servos", 10);

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/wamv/cmd_vel", 10, std::bind(&OffboardControl::cmd_vel_callback, this, _1));

        read_params();

        // init actuator value
        motor_left_ = 0.5;
        motor_right_ = 0.5; 
        servo_left_ = 0.0; 
        servo_right_ = 0.0;

        // record last data
        last_left_angle_ = 0.0;
        last_right_angle_ = 0.0;
        last_time_ = this->get_clock()->now().nanoseconds() / 1e9;

        // handing offboard mode
		offboard_setpoint_counter_ = 0;

		auto timer_callback = [this]() -> void {

			if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				// Arm the vehicle
				this->arm();
			}

			// offboard_control_mode needs to be paired with actuator [motors or servos]
			publish_offboard_control_mode();
            publish_actuator_controls();

			// stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
			}
		};
		timer_ = this->create_wall_timer(100ms, timer_callback);
        
        
	}

	void arm();
	void disarm();

    void set_actuator_value(double m_l, double m_r, double s_l, double s_r);

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

    rclcpp::Publisher<ActuatorMotors>::SharedPtr actuator_motors_publisher_;
    rclcpp::Publisher<ActuatorServos>::SharedPtr actuator_servos_publisher_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

    double motor_left_, motor_right_, servo_left_, servo_right_;

	void publish_offboard_control_mode();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void publish_actuator_controls();

    void read_params();
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    Eigen::Vector3d computeThrustTorqueFromTwist(Eigen::Vector3d t);
    Eigen::Vector4d allocateThrustTorqueToThrusters(Eigen::Vector3d tt);
    Eigen::Vector2d computeRPMFromThrusterForce(Eigen::Vector2d f);

    // fixed parameter
    double thrust_factor_;
    double v_x_gain_;
    double v_y_gain_;
    double w_z_gain_;
    double left_pos_x_;
    double left_pos_y_;
    double right_pos_x_;
    double right_pos_y_;
    double max_thrust_;
    double max_angle_;
    double max_angle_rate_;
    double max_linear_;
    double max_angular_;
    bool thruster_mode_;
    
    // propeller param
    double fluid_density_;
    double thrust_coefficient_;
    double propeller_diameter_;

    geometry_msgs::msg::Twist cmd_vel_;

    // last angle and time
    double last_left_angle_;
    double last_right_angle_;
    double last_time_;
};

void OffboardControl::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {

    cmd_vel_ = *msg;
    Eigen::Vector3d twist(cmd_vel_.linear.x, cmd_vel_.linear.y, cmd_vel_.angular.z);

    Eigen::Vector3d force_in_body = computeThrustTorqueFromTwist(twist);
    Eigen::Vector4d thruster_data = allocateThrustTorqueToThrusters(force_in_body);

    if (thruster_mode_) {
        Eigen::Vector2d thrust(thruster_data[0], thruster_data[1]);
        Eigen::Vector2d n = computeRPMFromThrusterForce(thrust);
        thruster_data[0] = n[0];
        thruster_data[1] = n[1];
    }

    // TODO remap to [0,1] or [-1, 1]
    motor_left_ = thruster_data[0];
    motor_right_ = thruster_data[1];
    servo_left_ = thruster_data[2];
    servo_right_ = thruster_data[3];
}

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 */
void OffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = false;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
    msg.thrust_and_torque = false;
    msg.direct_actuator = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish the actuator motor and servo command
 */
void OffboardControl::publish_actuator_controls() {

    ActuatorMotors motors_msg{};
	motors_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    motors_msg.control[0] = motor_left_; // left  [0, 1]
    motors_msg.control[1] = motor_right_; // right
    actuator_motors_publisher_->publish(motors_msg);

    ActuatorServos servos_msg{};
	servos_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    servos_msg.control[0] = servo_left_; // left  [-1, 1]
    servos_msg.control[1] = servo_right_; // right 
    actuator_servos_publisher_->publish(servos_msg);

}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

void OffboardControl::set_actuator_value(double m_l, double m_r, double s_l, double s_r) {

    motor_left_ = m_l;
    motor_right_ = m_r;
    servo_left_ = s_l;
    servo_right_ = s_r;
}

// [ux, uy, wz] -> [fx, fy, mz] 
Eigen::Vector3d OffboardControl::computeThrustTorqueFromTwist(Eigen::Vector3d t) {

    // TODO 

    Eigen::Vector3d three_vel(t);
    
    three_vel[0] = constrain(three_vel[0], -max_linear_, max_linear_);
    three_vel[1] = constrain(three_vel[1], -max_linear_, max_linear_);
    three_vel[2] = constrain(three_vel[2], -max_angular_, max_angular_);
    
    Eigen::Vector3d force;
    force[0] = v_x_gain_ * three_vel[0];
    force[1] = v_y_gain_ * three_vel[1];
    force[2] = w_z_gain_ * three_vel[2];

    return force;
}

// [fx, fy mz] -> [f1, f2, theta_1, theta_2]
Eigen::Vector4d OffboardControl::allocateThrustTorqueToThrusters(Eigen::Vector3d tt) {

    double x_off = std::abs(left_pos_x_); 
    double y_off = std::abs(left_pos_y_); 
    
    // 使用伪逆法求解欠定方程的解析解
    double F1x = 0.5 * tt[0] - x_off / ( 2 * y_off) * tt[1] - tt[2] / ( 2 * y_off);
    double F1y = 0.5 * tt[1];
    double F2x = 0.5 * tt[0] + x_off / ( 2 * y_off) * tt[1] + tt[2] / ( 2 * y_off);
    double F2y = 0.5 * tt[1];
    
    Eigen::Vector2d thruster_left = computeThrustVector(F1x, F1y);
    Eigen::Vector2d thruster_right = computeThrustVector(F2x, F2y);
    
    thruster_left[0] = constrain(thruster_left[0], -max_thrust_, max_thrust_);
    thruster_left[1] = constrain(thruster_left[1], -max_angle_, max_angle_);
    thruster_right[0] = constrain(thruster_right[0], -max_thrust_, max_thrust_);
    thruster_right[1] = constrain(thruster_right[1], -max_angle_, max_angle_);
    
    Eigen::Vector4d thruster_data(thruster_left[0], thruster_right[0], thruster_left[1], thruster_right[1]);
    return thruster_data;
}

// [f1, f2] -> [n1, n2]
Eigen::Vector2d OffboardControl::computeRPMFromThrusterForce(Eigen::Vector2d f) {

    double n1 = 2 * M_PI / 60 * std::sqrt( f[0] / fluid_density_ * thrust_coefficient_ * std::pow(propeller_diameter_, 4));
    double n2 = 2 * M_PI / 60 * std::sqrt( f[1] / fluid_density_ * thrust_coefficient_ * std::pow(propeller_diameter_, 4));

    return Eigen::Vector2d(n1, n2);
}

void OffboardControl::read_params() {
    
    // define default value
    this->declare_parameter<double>("thrust_factor", 1.0);
    this->declare_parameter<double>("v_x_gain", 1.0);
    this->declare_parameter<double>("v_y_gain", 300.0);
    this->declare_parameter<double>("w_z_gain", 300.0);
    this->declare_parameter<double>("left_pos_x", 1.0);
    this->declare_parameter<double>("left_pos_y", 1.0);
    this->declare_parameter<double>("right_pos_x", 1.0);
    this->declare_parameter<double>("right_pos_y", -1.0);
    this->declare_parameter<double>("max_thrust", 100.0);
    this->declare_parameter<double>("max_angle", 100.0);
    this->declare_parameter<double>("max_angle_rate", 0.52);
    this->declare_parameter<double>("max_linear", 3.0);
    this->declare_parameter<double>("max_angular", 0.5);
    this->declare_parameter<bool>("thruster_mode", false); // true: rpm mode. false: force mode

    // propeller params
    this->declare_parameter<double>("pp_fluid_density", 1000.0);
    this->declare_parameter<double>("pp_thrust_coefficient", 0.5);
    this->declare_parameter<double>("pp_diameter", 0.3);
    
    // set parameter
    thrust_factor_ = this->get_parameter("thrust_factor").as_double();
    v_x_gain_ = this->get_parameter("v_x_gain").as_double();
    v_y_gain_ = this->get_parameter("v_y_gain").as_double();
    w_z_gain_ = this->get_parameter("w_z_gain").as_double();
    left_pos_x_ = this->get_parameter("left_pos_x").as_double();
    left_pos_y_ = this->get_parameter("left_pos_y").as_double();
    right_pos_x_ = this->get_parameter("right_pos_x").as_double();
    right_pos_y_ = this->get_parameter("right_pos_y").as_double();
    max_thrust_ = this->get_parameter("max_thrust").as_double();
    max_angle_ = this->get_parameter("max_angle").as_double();
    max_angle_rate_ = this->get_parameter("max_angle_rate").as_double();
    max_linear_ = this->get_parameter("max_linear").as_double();
    max_angular_ = this->get_parameter("max_angular").as_double();
    thruster_mode_ = this->get_parameter("thruster_mode").as_bool();
    fluid_density_ = this->get_parameter("pp_fluid_density").as_double();
    thrust_coefficient_ = this->get_parameter("pp_thrust_coefficient").as_double();
    propeller_diameter_ = this->get_parameter("pp_diameter").as_double();
    
    
}

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}
