/*************************************************************/
/* Acknowledgement: github.com/uzh-rpg/rpg_quadrotor_control */
/*************************************************************/
#pragma once

#include "px4ctrl_state.h"
#include "sensor_msgs/Imu.h"
#include <Eigen/Dense>
#include <mavros_msgs/AttitudeTarget.h>
#include <nav_msgs/Odometry.h>
#include <queue>
#include <uav_utils/geometry_utils.h>

namespace px4ctrl {
namespace controller {

struct DesiredState {
  Eigen::Vector3d p;
  Eigen::Vector3d v;
  Eigen::Vector3d a;
  Eigen::Vector3d j;
  Eigen::Quaterniond q;
  double yaw;
  double yaw_rate;

  DesiredState() {
    p = Eigen::Vector3d::Zero();
    v = Eigen::Vector3d::Zero();
    a = Eigen::Vector3d::Zero();
    j = Eigen::Vector3d::Zero();
    q = Eigen::Quaterniond::Identity();
    yaw = 0;
    yaw_rate = 0;
  };

  DesiredState(const nav_msgs::Odometry &odom) {
    p = Eigen::Vector3d(odom.pose.pose.position.x, odom.pose.pose.position.y,
                        odom.pose.pose.position.z);
    v = Eigen::Vector3d(odom.twist.twist.linear.x, odom.twist.twist.linear.y,
                        odom.twist.twist.linear.z);
    a = Eigen::Vector3d::Zero();
    j = Eigen::Vector3d::Zero();
    q = Eigen::Quaterniond(
        odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y, odom.pose.pose.orientation.z);
    yaw = uav_utils::get_yaw_from_quaternion(q);
    yaw_rate = 0;
  };
};

enum class ControlType{
  BODY_RATES,
  ATTITUDE
};

struct ControlCommand
// NOTE: 该结构体是控制器的输出
{
  ControlType type = ControlType::BODY_RATES;
  //attitude in local frame
  Eigen::Quaterniond attitude;
  // Body rates in body frame
  Eigen::Vector3d bodyrates; // [rad/s]

  // Collective mass normalized thrust
  double thrust;
};

struct ControlParams{
    double mass;
    double g;
    double Kp,Kv,Ka,Kw;
    double hover_percentage;
    double Ix,Iy,Iz;//TODO
    bool bodyrates_control;
};

class LinearControl {
public:
  LinearControl()=delete;
  LinearControl(const ControlParams& params);
  ControlCommand calculateControl(const DesiredState &des,const nav_msgs::Odometry &odom,const sensor_msgs::Imu &imu);

  //thrust mapping
  bool estimateThrustModel(const Eigen::Vector3d &est_a, const clock::time_point& est_time);
  void resetThrustMapping();
  double thrustMap(const double collective_thrust);
  double fromQuaternion2yaw( const Eigen::Quaterniond & q );

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
  ControlParams params;
  Eigen::Vector3d vee(const Eigen::Matrix3d &m);
  
  //Thrust mapping
  std::queue<std::pair<clock::time_point, double>> timed_thrust;
  static constexpr double kMinNormalizedCollectiveThrust = 3.0;
  // Thrust-accel mapping params
  const double rho2 = 0.998; // do not change
  double thr2acc;
  double P;
};

} // namespace controller
} // namespace px4ctrl