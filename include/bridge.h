#pragma once
#include <chrono>
#include <functional>
#include <memory>

#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <px4_msgs/msg/detail/vehicle_command__struct.hpp>
#include <px4_msgs/msg/detail/vehicle_rates_setpoint__struct.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

//mavros_msgs
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>

//ros_msgs
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

//px4ctrl_msgs
#include <px4ctrl_msgs/msg/command.hpp>

//px4_msgs
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <spdlog/spdlog.h>

#include "types.h"

namespace px4ctrl {

using namespace std::chrono_literals;

template <typename ITEM, typename TIME>
using IPX4_STATE = Px4DataPtr<std::pair<ITEM, TIME>>;

struct Px4State {
  IPX4_STATE<mavros_msgs::msg::State::ConstSharedPtr, clock::time_point> state;
  IPX4_STATE<mavros_msgs::msg::ExtendedState::ConstSharedPtr, clock::time_point> ext_state;
  IPX4_STATE<sensor_msgs::msg::BatteryState::ConstSharedPtr, clock::time_point> battery;
  IPX4_STATE<nav_msgs::msg::Odometry::ConstSharedPtr, clock::time_point> odom;
  IPX4_STATE<sensor_msgs::msg::Imu::ConstSharedPtr, clock::time_point> imu;
  IPX4_STATE<px4ctrl_msgs::msg::Command::ConstSharedPtr, clock::time_point> ctrl_command;
  IPX4_STATE<px4_msgs::msg::VehicleStatus::ConstSharedPtr,clock::time_point> vehicle_status;
  Px4State(){
    state = std::make_shared<Px4Data<std::pair<mavros_msgs::msg::State::ConstSharedPtr, clock::time_point>>>();
    ext_state = std::make_shared<Px4Data<std::pair<mavros_msgs::msg::ExtendedState::ConstSharedPtr, clock::time_point>>>();
    battery = std::make_shared<Px4Data<std::pair<sensor_msgs::msg::BatteryState::ConstSharedPtr, clock::time_point>>>();
    odom = std::make_shared<Px4Data<std::pair<nav_msgs::msg::Odometry::ConstSharedPtr, clock::time_point>>>();
    imu = std::make_shared<Px4Data<std::pair<sensor_msgs::msg::Imu::ConstSharedPtr, clock::time_point>>>();
    ctrl_command = std::make_shared<Px4Data<std::pair<px4ctrl_msgs::msg::Command::ConstSharedPtr, clock::time_point>>>();
    vehicle_status = std::make_shared<Px4Data<std::pair<px4_msgs::msg::VehicleStatus::ConstSharedPtr,clock::time_point>>>();
  }
};

class Px4CtrlRosBridge{
public:
  Px4CtrlRosBridge() = delete;
  Px4CtrlRosBridge(rclcpp::Node::SharedPtr node, std::shared_ptr<Px4State> px4_state);

  bool set_mode(const std::string &mode);
  bool set_arm(const bool arm);
  bool force_disarm();
  bool enter_offboard();
  bool exit_offboard();
  bool restart_fcu();
  
  //ctrl interface
  void pub_bodyrates_target(const double thrust, const Eigen::Vector3d& bodyrates);
  void pub_attitude_target(const double thrust, const Eigen::Quaterniond& quat);
  void pub_torque_target(const double thrust, const Eigen::Vector3d& torque);
  void pub_actuator_target(const Eigen::Vector4d& motors);

  void pub_allow_cmdctrl(bool allow);
  void pub_offboard_control_mode_msg(const controller::ControlType type);//called on every cycle
  void spin_once();

private:
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<Px4State> px4_state_;

  // Timers
  rclcpp::TimerBase::SharedPtr offboardTimer;

  // subscribers
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr px4_state_sub;
  rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr px4_extended_state_sub;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr bat_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ext_odom_sub;
  rclcpp::Subscription<px4ctrl_msgs::msg::Command>::SharedPtr ctrl_cmd_sub;

  // Publishers
  rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr actuator_motors_pub;
  rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr attitude_setpoint_pub;
  rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr rates_setpoint_pub;
  rclcpp::Publisher<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr thrust_setpoint_pub;
  rclcpp::Publisher<px4_msgs::msg::VehicleTorqueSetpoint>::SharedPtr torque_setpoint_pub;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr vehicle_odometry_pub;//pub px4 odom to ros (convert NED to ENU)
  // rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr px4_cmd_pub; deprecated
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr allow_cmdctrl_pub;
  // Client for Service
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr px4_set_mode_client;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr px4_arming_client;
  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr px4_cmd_client;
  rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr px4_vehicle_command_client;

  // Topics namescontroller_
  // sub
  std::string vehicle_status_sub_topic, vehicle_odometry_sub_topic;
  std::string px4_state_sub_topic, px4_extended_state_sub_topic;
  std::string imu_sub_topic, bat_sub_topic, ext_odom_sub_topic, ctrl_cmd_sub_topic;

  // pub
  std::string actuator_motors_pub_topic;
  std::string rates_setpoint_pub_topic;
  std::string attitude_setpoint_pub_topic;
  std::string thrust_setpoint_pub_topic;
  std::string torque_setpoint_pub_topic;
  std::string offboard_control_mode_pub_topic;
  std::string allow_cmdctrl_pub_topic;
  std::string vehicle_odometry_pub_topic;

  // client
  std::string px4_set_mode_client_topic;
  std::string px4_arming_client_topic;
  std::string px4_cmd_client_topic;
  std::string vehicle_command_client_topic;

  void load_params();
  void vehicle_odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
  void vehicle_odom_from_px4_to_ros(const px4_msgs::msg::VehicleOdometry& msg, nav_msgs::msg::Odometry& odom);
};

template <typename MSG_PTR> using CB_FUNC = std::function<void(MSG_PTR)>;

// template <IPX4_ITEM ITEM, IPX4_TIME TIME>
template<typename ITEM, typename TIME>
inline CB_FUNC<ITEM> build_px4ros_cb(IPX4_STATE<ITEM, TIME> &msg) {
  return std::bind(
      [](ITEM msg_ptr, IPX4_STATE<ITEM, TIME> msg) {
        msg->post(std::make_pair(msg_ptr, std::chrono::system_clock::now()));
      },
      std::placeholders::_1, msg);
}

} // namespace px4ctrl