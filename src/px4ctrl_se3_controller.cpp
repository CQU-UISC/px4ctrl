#include "px4ctrl_se3_controller.h"
#include "px4ctrl_state.h"
#include <chrono>
#include <cmath>

namespace px4ctrl {
namespace controller {

LinearControl::LinearControl(const ControlParams &params) : params(params) {
  // TODO
  resetThrustMapping();
}

// Output bodyrates and thrust $\in [0,1]$
ControlCommand LinearControl::calculateControl(const DesiredState &des,
                                               const nav_msgs::Odometry &odom,
                                               const sensor_msgs::Imu &imu) {
  ControlCommand ret;
  Eigen::Vector3d err_a, err_v, err_p;
  Eigen::Vector3d ez(0, 0, 1);
  Eigen::Vector3d acc, vel, pos;
  Eigen::Quaterniond quat;
  Eigen::Matrix3d rot;

  acc << imu.linear_acceleration.x, imu.linear_acceleration.y,
      imu.linear_acceleration.z;
  vel << odom.twist.twist.linear.x, odom.twist.twist.linear.y,
      odom.twist.twist.linear.z;
  pos << odom.pose.pose.position.x, odom.pose.pose.position.y,
      odom.pose.pose.position.z;
  quat = Eigen::Quaterniond(
      odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
      odom.pose.pose.orientation.y, odom.pose.pose.orientation.z);
  rot = quat.toRotationMatrix();

  err_p = pos - des.p;
  err_v = vel - des.v;
  err_a = acc - des.a;

  Eigen::Vector3d des_acc = -1 * params.Ka * err_a - params.Kv * err_v -
                            params.Kp * err_p + params.g * ez;

  double collective_thrust = des_acc.dot(quat * ez);
  ret.thrust = thrustMap(collective_thrust);

  Eigen::Vector3d zb = des_acc.normalized();
  Eigen::Vector3d xc(std::cos(des.yaw), std::sin(des.yaw), 0);
  Eigen::Vector3d yb = zb.cross(xc).normalized();
  Eigen::Vector3d xb = yb.cross(zb);
  Eigen::Matrix3d des_rot;
  des_rot << xb, yb, zb;

  Eigen::Matrix3d err_rot = des_rot.transpose() * rot;
  Eigen::Vector3d bodyrates =
      -params.Kw * 1 / 2.f * vee(err_rot - err_rot.transpose());
  ret.bodyrates = bodyrates;

  // Used for thrust-accel mapping estimation
  timed_thrust.push(
      std::pair<clock::time_point, double>(clock::now(), ret.thrust));

  while (timed_thrust.size() > 100) {
    timed_thrust.pop();
  }
  ret.type = ControlType::BODY_RATES;
  return ret;
}

Eigen::Vector3d LinearControl::vee(const Eigen::Matrix3d &m) {
  Eigen::Vector3d ret;
  ret << m(2, 1) - m(1, 2), m(0, 2) - m(2, 0), m(1, 0) - m(0, 1);
  ret /= 2;
  return ret;
}

/*
  compute throttle percentage
  des_acc: desired acceleration in world frame
  return: throttle percentage
*/
double LinearControl::thrustMap( const double collective_thrust ) {
    double throttle_percentage( 0.0 );

    /* compute throttle, thr2acc has been estimated before */
    throttle_percentage = collective_thrust / thr2acc;

    return throttle_percentage;
}

bool LinearControl::estimateThrustModel(const Eigen::Vector3d &est_a,const clock::time_point& est_time) {
  // clock::time_point t_now = clock::now();
  const clock::time_point t_now  = est_time;
  while (timed_thrust.size() >= 1) {
    // Choose data before 35~45ms ago
    std::pair<clock::time_point, double> t_t = timed_thrust.front();
    long time_passed =
        std::chrono::duration_cast<std::chrono::milliseconds>(t_now - t_t.first)
            .count();
    if (time_passed > 45L) // thrust data is too old, more than 45ms
    {
      timed_thrust.pop();
      continue;
    }
    if (time_passed < 35L) // thrust data is too new, less than 35ms
    {
      return false;
    }

    /***********************************************************/
    /* Recursive least squares algorithm with vanishing memory */
    /***********************************************************/
    double thr = t_t.second;
    timed_thrust.pop();

    /************************************/
    /* Model: est_a(2) = thr2acc * thrust */
    /* thr = collective_thrust / thr2acc     */
    /************************************/
    double gamma = 1 / (rho2 + thr * P * thr);
    double K = gamma * P * thr;
    thr2acc = thr2acc + K * (est_a(2) - thr * thr2acc);
    P = (1 - K * thr) * P / rho2;

    return true;
  }
  return false;
}

/*
thr2acc 是一个常数，用于将油门值转换为加速度
let des_acc be x, gravity be g, hover_percentage be h, thrust be t, then
t = (1+x/g)*h = h + (h/g)*x, h need to be estimated
简易推力模型是一个线性模型，认为油门值和产生的加速度是一个线性关系，
会在线根据期望机体z轴加速度和实际机体z轴加速度估计线性模型的斜率.
*/
void LinearControl::resetThrustMapping() {
  thr2acc = params.g / params.hover_percentage;
  P = 1e6;
}

} // namespace controller

} // namespace px4ctrl