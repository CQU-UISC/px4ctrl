#include "controller.h"
#include "params.h"
#include <Eigen/src/Geometry/Quaternion.h>
#include <chrono>
#include <cmath>
#include <spdlog/spdlog.h>

namespace px4ctrl {
namespace controller {

Se3Control::Se3Control(const params::ControlParams &ctrl_params, const params::QuadrotorParams &quad_params)
  :ctrl_params_(ctrl_params),quad_params_(quad_params) { 
  resetThrustMapping();
  vel_error_integral_ = Eigen::Vector3d::Zero();
}

// Output bodyrates and thrust $\in [0,1]$
ControlCommand Se3Control::runControl(const DesiredState &des,
                                              const nav_msgs::msg::Odometry &odom,
                                              const sensor_msgs::msg::Imu &imu) {
  ControlCommand ret;
  Eigen::Vector3d err_a, err_v, err_p;
  Eigen::Vector3d ez(0, 0, 1);
  Eigen::Vector3d acc, vel, pos;
  Eigen::Quaterniond quat;
  Eigen::Matrix3d rot;

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

  Eigen::Vector3d des_acc = des.a + quad_params_.g * ez;
  err_p = err_p.cwiseMin(ctrl_params_.max_pos_error).cwiseMax(-ctrl_params_.max_pos_error);
  err_v = err_v.cwiseMin(ctrl_params_.max_vel_error).cwiseMax(-ctrl_params_.max_vel_error);
  vel_error_integral_ += err_v*ctrl_params_.Kd_pos+err_p*ctrl_params_.Kp_pos;
  vel_error_integral_ = vel_error_integral_.cwiseMin(ctrl_params_.max_vel_int).cwiseMax(-ctrl_params_.max_vel_int);
  des_acc -= ctrl_params_.Kp_pos * err_p + ctrl_params_.Kd_pos * err_v + ctrl_params_.Ki_pos * vel_error_integral_;

  double collective_thrust = des_acc.dot(quat * ez);
  ret.thrust = thrustMap(collective_thrust);

  Eigen::Vector3d zb = des_acc.normalized();
  Eigen::Vector3d xc(std::cos(des.yaw), std::sin(des.yaw), 0);
  Eigen::Vector3d yb = zb.cross(xc).normalized();
  Eigen::Vector3d xb = yb.cross(zb);
  Eigen::Matrix3d des_rot;
  des_rot << xb, yb, zb;
  const auto &imu_q = imu.orientation;
  const auto &odom_q = odom.pose.pose.orientation;
  const auto imu_quat = Eigen::Quaterniond(imu_q.w, imu_q.x, imu_q.y, imu_q.z);
  const auto odom_quat = Eigen::Quaterniond(odom_q.w, odom_q.x, odom_q.y, odom_q.z);
  const auto des_quat = Eigen::Quaterniond(des_rot);
  des_rot =
      (imu_quat *
       odom_quat.inverse() *
       des_quat)
          .toRotationMatrix();

  if (ctrl_params_.type==ControlType::BODY_RATES) {
    const Eigen::Quaterniond q_e = odom_quat.inverse() * des_quat;
    Eigen::Vector3d bodyrates;
    if (q_e.w() >= 0) {
      bodyrates.x() = 2.0 * ctrl_params_.Kw_rp  * q_e.x();
      bodyrates.y() = 2.0 * ctrl_params_.Kw_rp  * q_e.y();
      bodyrates.z() = 2.0 * ctrl_params_.Kw_yaw  * q_e.z();
    } else {
      bodyrates.x() = -2.0 * ctrl_params_.Kw_rp  * q_e.x();
      bodyrates.y() = -2.0 * ctrl_params_.Kw_rp  * q_e.y();
      bodyrates.z() = -2.0 * ctrl_params_.Kw_yaw  * q_e.z();
    }
    ret.type = ControlType::BODY_RATES;
    ret.bodyrates = bodyrates;
  } else if(ctrl_params_.type==ControlType::ATTITUDE) {
    ret.type = ControlType::ATTITUDE;
    ret.attitude = Eigen::Quaterniond(des_rot);
  }


  // Used for thrust-accel mapping estimation
  timed_thrust.push(
      std::pair<clock::time_point, double>(clock::now(), ret.thrust));

  while (timed_thrust.size() > 100) {
    timed_thrust.pop();
  }
  ret.source = ControlSource::SE3;
  return ret;
}

ControlCommand Se3Control::runSafeControl(const sensor_msgs::msg::Imu &imu){
  const auto k = 0.95;
  double t = k*thrustMap(quad_params_.g);
  ControlCommand ret;
  ret.source = ControlSource::SAFETY;
  ret.type = ControlType::ATTITUDE;
  ret.attitude = Eigen::Quaterniond(1,0,0,0);
  ret.thrust = t;
  return ret;
}


Eigen::Vector3d Se3Control::vee(const Eigen::Matrix3d &m) {
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
double Se3Control::thrustMap(const double collective_thrust) {
  double throttle_percentage(0.0);

  /* compute throttle, thr2acc has been estimated before */
  throttle_percentage = collective_thrust / thr2acc;

  return throttle_percentage;
}

bool Se3Control::estimateThrustModel(const Eigen::Vector3d &est_a,
                                        const clock::time_point &est_time) {
  // clock::time_point t_now = clock::now();
  const clock::time_point t_now = est_time;
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
    thr2acc =
        thr2acc +
        K * (est_a(2) -
             thr * thr2acc); // collective_thrust = g (imu z value),
                             // collective_thrust/thurst2acc = hover_percentage;
    P = (1 - K * thr) * P / rho2;
    spdlog::debug("Estimated hoving percentage:{}",quad_params_.g/thr2acc);
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
void Se3Control::resetThrustMapping() {
  auto init_thr2acc = quad_params_.g / quad_params_.init_hover_thrust;
  P = 1e6;
  spdlog::info("Reset thrust mapping(Linear Model), hover thrust from {} to {}", quad_params_.g/thr2acc, quad_params_.g/init_thr2acc);
  thr2acc = init_thr2acc;
  while(!timed_thrust.empty()){
    timed_thrust.pop();
  }
}

} // namespace controller

} // namespace px4ctrl