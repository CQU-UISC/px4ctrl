/*************************************************************/
/* Acknowledgement:                                          */
/* github.com/uzh-rpg/rpg_quadrotor_control                  */
/* github.com/fastlab                                        */
/*************************************************************/
#pragma once

#include "types.h"
#include "params.h"
#include <Eigen/Dense>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <queue>

namespace px4ctrl {
namespace controller {
inline double yawFromQuat(const Eigen::Quaterniond &q) {
  // atan2(2.0f * (w * z + x * y), w * w + x * x - y * y - z * z);
  double yaw =
      atan2(2 * (q.x() * q.y() + q.w() * q.z()),
            q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
  return yaw;
}

struct DesiredState {
  Eigen::Vector3d p;
  Eigen::Vector3d v;
  Eigen::Vector3d a;
  Eigen::Vector3d j;
  Eigen::Quaterniond q;
  double yaw;
  double yaw_rate;
  bool control_attitude;

  DesiredState() {
    p = Eigen::Vector3d::Zero();
    v = Eigen::Vector3d::Zero();
    a = Eigen::Vector3d::Zero();
    j = Eigen::Vector3d::Zero();
    q = Eigen::Quaterniond::Identity();
    yaw = 0;
    yaw_rate = 0;
    control_attitude = false;//TODO
  };

  DesiredState(const nav_msgs::msg::Odometry &odom) {
    p = Eigen::Vector3d(odom.pose.pose.position.x, odom.pose.pose.position.y,
                        odom.pose.pose.position.z);
    v = Eigen::Vector3d(odom.twist.twist.linear.x, odom.twist.twist.linear.y,
                        odom.twist.twist.linear.z);
    a = Eigen::Vector3d::Zero();
    j = Eigen::Vector3d::Zero();
    q = Eigen::Quaterniond(
        odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y, odom.pose.pose.orientation.z);
    yaw = yawFromQuat(q);
    yaw_rate = 0;
    control_attitude = false;
  };
};


struct ControlCommand
// NOTE: 该结构体是控制器的输出
{
  params::ControlType type = params::ControlType::BODY_RATES;
  // attitude in local frame
  Eigen::Quaterniond attitude;
  // Body rates in body frame
  Eigen::Vector3d bodyrates; // [rad/s]

  // Collective mass normalized thrust
  double thrust;
};



class Se3Control {
public:
  Se3Control() = delete;
  Se3Control(const params::ControlParams &ctrl_params, const params::QuadrotorParams &quad_params);

  ControlCommand calculateControl(const DesiredState &des,
                                  const nav_msgs::msg::Odometry &odom,
                                  const sensor_msgs::msg::Imu &imu);

  // thrust mapping
  bool estimateThrustModel(const Eigen::Vector3d &est_a,const clock::time_point &est_time);
  void resetThrustMapping();
  double thrustMap(const double collective_thrust);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
  const params::ControlParams ctrl_params_;
  const params::QuadrotorParams quad_params_;
  // integral term for position control
  Eigen::Vector3d vel_error_integral_;

  Eigen::Vector3d vee(const Eigen::Matrix3d &m);

  template <typename T>
  inline T limit(const T &value, const T &min, const T &max) {
    if (value > max) {
      return max;
    }
    if (value < min) {
      return min;
    }
    return value;
  }

  // Thrust mapping
  std::queue<std::pair<clock::time_point, double>> timed_thrust;
  static constexpr double kMinNormalizedCollectiveThrust = 3.0;
  // Thrust-accel mapping params
  const double rho2 = 0.998; // do not change
  double thr2acc;
  double P;
};

} // namespace controller
} // namespace px4ctrl