#pragma once
#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <type_traits>

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <ros/service_client.h>

#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <px4ctrl_lux/Command.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <spdlog/spdlog.h>

#include "types.h"

namespace px4ctrl {

template <typename T, typename... U>
concept any_of = (std::same_as<T, U> || ...);

template <typename T>
using remove_cvref_t = typename std::remove_cvref<T>::type;
// 判断是否为std::pair<T,clock::time_point>的类型
template <typename T>
concept IPX4_ITEM = requires(T t) {
  {
    remove_cvref_t<T>(t)
  }
  -> any_of<mavros_msgs::State::ConstPtr, mavros_msgs::ExtendedState::ConstPtr,
            sensor_msgs::BatteryState::ConstPtr, nav_msgs::Odometry::ConstPtr,
            sensor_msgs::Imu::ConstPtr, px4ctrl_lux::Command::ConstPtr>;
};

template <typename T>
concept IPX4_TIME = requires(T t) {
  { std::remove_cvref_t<T>(t) } -> std::same_as<clock::time_point>;
};

template <IPX4_ITEM ITEM, IPX4_TIME TIME>
using IPX4_STATE = Px4DataPtr<std::pair<ITEM, TIME>>;

struct Px4State {
  IPX4_STATE<mavros_msgs::State::ConstPtr, clock::time_point> state;
  IPX4_STATE<mavros_msgs::ExtendedState::ConstPtr, clock::time_point> ext_state;
  IPX4_STATE<sensor_msgs::BatteryState::ConstPtr, clock::time_point> battery;
  IPX4_STATE<nav_msgs::Odometry::ConstPtr, clock::time_point> odom;
  IPX4_STATE<sensor_msgs::Imu::ConstPtr, clock::time_point> imu;
  IPX4_STATE<px4ctrl_lux::Command::ConstPtr, clock::time_point> ctrl_command;

  Px4State(){
    state = std::make_shared<Px4Data<std::pair<mavros_msgs::State::ConstPtr, clock::time_point>>>();
    ext_state = std::make_shared<Px4Data<std::pair<mavros_msgs::ExtendedState::ConstPtr, clock::time_point>>>();
    battery = std::make_shared<Px4Data<std::pair<sensor_msgs::BatteryState::ConstPtr, clock::time_point>>>();
    odom = std::make_shared<Px4Data<std::pair<nav_msgs::Odometry::ConstPtr, clock::time_point>>>();
    imu = std::make_shared<Px4Data<std::pair<sensor_msgs::Imu::ConstPtr, clock::time_point>>>();
    ctrl_command = std::make_shared<Px4Data<std::pair<px4ctrl_lux::Command::ConstPtr, clock::time_point>>>();
  }
};

class Px4CtrlRosBridge {

public:
  Px4CtrlRosBridge() = delete;
  Px4CtrlRosBridge(const ros::NodeHandle &nh, std::shared_ptr<Px4State> px4_state);

  bool set_mode(const std::string &mode);
  bool set_arm(const bool arm);
  bool force_disarm();
  bool enter_offboard();
  bool exit_offboard();
  
  bool pub_bodyrates_target(
      const double thrust,
      const std::array<double, 3> &bodyrates); // 目前只实现控制bodyrate
  bool pub_attitude_target(const double thrust,
                            const std::array<double, 4> quat);
  void pub_allow_cmdctrl(bool allow);
  void spin_once();

private:
  ros::NodeHandle nh_;

  ros::Subscriber px4_state_sub, px4_extended_state_sub, px4_imu_sub,
      px4_bat_sub;

  ros::Subscriber vio_odom_sub, ctrl_cmd_sub, user_cmd_sub;

  ros::Publisher px4_cmd_pub, allow_cmdctrl_pub;
  ros::ServiceClient px4_set_mode_client, px4_arming_client, px4_cmd_client;

  std::shared_ptr<Px4State> px4_state_;
};

template <typename MSG_PTR> using CB_FUNC = std::function<void(MSG_PTR)>;

template <IPX4_ITEM ITEM, IPX4_TIME TIME>
inline CB_FUNC<ITEM> build_px4ros_cb(IPX4_STATE<ITEM, TIME> &msg // reference
) {
  return std::bind(
      [](ITEM msg_ptr, IPX4_STATE<ITEM, TIME> msg) {
        msg->post(std::make_pair(msg_ptr, std::chrono::system_clock::now()));
      },
      std::placeholders::_1, msg);
}

} // namespace px4ctrl