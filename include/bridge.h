#pragma once
#include <array>
#include <chrono>
#include <functional>
#include <memory>
// #include <type_traits>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>

#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <nav_msgs/msg/odometry.hpp>

//px4ctrl
#include <px4ctrl_msgs/msg/command.hpp>

//px4_msgs
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <spdlog/spdlog.h>

#include "types.h"

namespace px4ctrl {

using namespace std::chrono_literals;

// template <typename T, typename... U>
// concept any_of = (std::same_as<T, U> || ...);

// template <typename T>
// using remove_cvref_t = typename std::remove_cvref<T>::type;

// template <typename T>
// concept IPX4_ITEM = requires(T t) {
//   {
//     remove_cvref_t<T>(t)
//   }
//   -> any_of<mavros_msgs::msg::State::ConstSharedPtr, 
//             mavros_msgs::msg::ExtendedState::ConstSharedPtr,
//             sensor_msgs::msg::BatteryState::ConstSharedPtr, 
//             nav_msgs::msg::Odometry::ConstSharedPtr,
//             sensor_msgs::msg::Imu::ConstSharedPtr, 
//             px4ctrl_msgs::msg::Command::ConstSharedPtr>;
// };

// template <typename T>
// concept IPX4_TIME = requires(T t) {
//   { std::remove_cvref_t<T>(t) } -> std::same_as<clock::time_point>;
// };

// template <IPX4_ITEM ITEM, IPX4_TIME TIME>
template <typename ITEM, typename TIME>
using IPX4_STATE = Px4DataPtr<std::pair<ITEM, TIME>>;

struct Px4State {
  IPX4_STATE<mavros_msgs::msg::State::ConstSharedPtr, clock::time_point> state;
  IPX4_STATE<mavros_msgs::msg::ExtendedState::ConstSharedPtr, clock::time_point> ext_state;
  IPX4_STATE<sensor_msgs::msg::BatteryState::ConstSharedPtr, clock::time_point> battery;
  IPX4_STATE<nav_msgs::msg::Odometry::ConstSharedPtr, clock::time_point> odom;
  IPX4_STATE<sensor_msgs::msg::Imu::ConstSharedPtr, clock::time_point> imu;
  IPX4_STATE<px4ctrl_msgs::msg::Command::ConstSharedPtr, clock::time_point> ctrl_command;

  Px4State(){
    state = std::make_shared<Px4Data<std::pair<mavros_msgs::msg::State::ConstSharedPtr, clock::time_point>>>();
    ext_state = std::make_shared<Px4Data<std::pair<mavros_msgs::msg::ExtendedState::ConstSharedPtr, clock::time_point>>>();
    battery = std::make_shared<Px4Data<std::pair<sensor_msgs::msg::BatteryState::ConstSharedPtr, clock::time_point>>>();
    odom = std::make_shared<Px4Data<std::pair<nav_msgs::msg::Odometry::ConstSharedPtr, clock::time_point>>>();
    imu = std::make_shared<Px4Data<std::pair<sensor_msgs::msg::Imu::ConstSharedPtr, clock::time_point>>>();
    ctrl_command = std::make_shared<Px4Data<std::pair<px4ctrl_msgs::msg::Command::ConstSharedPtr, clock::time_point>>>();
  }
};

class Px4CtrlRosBridge{
public:
  Px4CtrlRosBridge() = delete;
  Px4CtrlRosBridge(const rclcpp::Node::SharedPtr& node, std::shared_ptr<Px4State> px4_state);

  bool set_mode(const std::string &mode);
  bool set_arm(const bool arm);
  bool force_disarm();
  bool enter_offboard();
  bool exit_offboard();
  
  bool pub_bodyrates_target(const double thrust, const std::array<double, 3> &bodyrates);
  bool pub_attitude_target(const double thrust, const std::array<double, 4> quat);
  void pub_allow_cmdctrl(bool allow);
  void spin_once();

private:
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<Px4State> px4_state_;

  // Timers
  rclcpp::TimerBase::SharedPtr controllerTimer;
  rclcpp::TimerBase::SharedPtr offboardTimer;

  // subscribers
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;
  
  // Publishers
  rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr actuator_motors_publisher_;
  rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr attitude_setpoint_publisher_;
  rclcpp::Publisher<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr thrust_setpoint_publisher_;
  rclcpp::Publisher<px4_msgs::msg::VehicleTorqueSetpoint>::SharedPtr torque_setpoint_publisher_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;


  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr px4_state_sub;
  rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr px4_extended_state_sub;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr px4_imu_sub;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr px4_bat_sub;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vio_odom_sub;
  rclcpp::Subscription<px4ctrl_msgs::msg::Command>::SharedPtr ctrl_cmd_sub;

  rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr px4_cmd_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr allow_cmdctrl_pub;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr px4_set_mode_client;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr px4_arming_client;
  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr px4_cmd_client;
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