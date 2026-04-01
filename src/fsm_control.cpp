#include "fsm.h"

#include <algorithm>
#include <array>

#include <px4ctrl_msgs/msg/command.hpp>

namespace px4ctrl {

void Px4Ctrl::build_proof_alive(const MissionContext &ctx,
                                controller::ControlCommand &ctrl_cmd) const {
  ctrl_cmd.type = params::ControlType::ATTITUDE;
  if (ctx.imu_msg != nullptr) {
    const auto &quat = ctx.imu_msg->orientation;
    ctrl_cmd.attitude = Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z);
  } else {
    ctrl_cmd.attitude = Eigen::Quaterniond::Identity();
  }
  ctrl_cmd.thrust = 0.01;
}

bool Px4Ctrl::build_se3_command(const MissionContext &ctx, MissionPhase phase,
                                controller::ControlCommand &ctrl_cmd) {
  if (ctx.odom_msg == nullptr || ctx.imu_msg == nullptr) {
    return false;
  }

  const double speed = px4ctrl_params->statemachine_params.l2_takeoff_landing_speed;
  controller::DesiredState des;

  switch (phase) {
  case MissionPhase::WAIT_FOR_RC:
  case MissionPhase::STANDBY:
    ctrl_cmd.type = params::ControlType::BODYRATES;
    ctrl_cmd.thrust = 0.1;
    ctrl_cmd.bodyrates = Eigen::Vector3d::Zero();
    return true;

  case MissionPhase::TAKEOFF: {
    if (!takeoff_.initialized) {
      return false;
    }
    des.p = takeoff_.start_pos;
    des.q = takeoff_.start_q;
    des.yaw = controller::yawFromQuat(des.q);
    des.v = Eigen::Vector3d(0, 0, speed);

    const double elapsed_s = timeDuration(takeoff_.start_time, ctx.now) / 1000.0;
    const double target_z =
        takeoff_.start_pos.z() + px4ctrl_params->statemachine_params.l2_takeoff_height;
    des.p.z() = takeoff_.start_pos.z() + speed * elapsed_s;
    if (des.p.z() >= target_z) {
      des.p.z() = target_z;
      des.v = Eigen::Vector3d::Zero();
    }

    ctrl_cmd = se3_controller_->runControl(des, *ctx.odom_msg, *ctx.imu_msg);
    estimate_thrust_from_imu();
    return true;
  }

  case MissionPhase::HOVER:
  case MissionPhase::CMD_CTRL_READY:
  case MissionPhase::CMD_CTRL: {
    if (!hover_.initialized) {
      hover_.pos = Eigen::Vector3d(ctx.odom_msg->pose.pose.position.x,
                                   ctx.odom_msg->pose.pose.position.y,
                                   ctx.odom_msg->pose.pose.position.z);
      const Eigen::Quaterniond q(ctx.odom_msg->pose.pose.orientation.w,
                                 ctx.odom_msg->pose.pose.orientation.x,
                                 ctx.odom_msg->pose.pose.orientation.y,
                                 ctx.odom_msg->pose.pose.orientation.z);
      const auto yaw = controller::yawFromQuat(q);
      hover_.q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
      hover_.initialized = true;
    }
    des.p = hover_.pos;
    des.q = hover_.q;
    des.yaw = controller::yawFromQuat(des.q);

    ctrl_cmd = se3_controller_->runControl(des, *ctx.odom_msg, *ctx.imu_msg);
    estimate_thrust_from_imu();
    return true;
  }

  case MissionPhase::LANDING:
  case MissionPhase::FAILSAFE: {
    if (phase == MissionPhase::FAILSAFE && active_guard_action_ == params::Guard::HOLD) {
      des.p = hover_.pos;
      des.q = hover_.q;
      des.yaw = controller::yawFromQuat(des.q);
      ctrl_cmd = se3_controller_->runControl(des, *ctx.odom_msg, *ctx.imu_msg);
      estimate_thrust_from_imu();
      return true;
    }

    if (!landing_.initialized) {
      landing_.start_pos = Eigen::Vector3d(ctx.odom_msg->pose.pose.position.x,
                                           ctx.odom_msg->pose.pose.position.y,
                                           ctx.odom_msg->pose.pose.position.z);
      landing_.start_q = Eigen::Quaterniond(ctx.odom_msg->pose.pose.orientation.w,
                                            ctx.odom_msg->pose.pose.orientation.x,
                                            ctx.odom_msg->pose.pose.orientation.y,
                                            ctx.odom_msg->pose.pose.orientation.z);
      landing_.start_time = ctx.now;
      landing_.initialized = true;
      landing_.c12_started = false;
    }

    des.p = landing_.start_pos;
    des.q = landing_.start_q;
    des.yaw = controller::yawFromQuat(des.q);
    des.v = Eigen::Vector3d(0, 0, -speed);
    des.p.z() -= speed * (timeDuration(landing_.start_time, ctx.now) / 1000.0);

    ctrl_cmd = se3_controller_->runControl(des, *ctx.odom_msg, *ctx.imu_msg);
    estimate_thrust_from_imu();
    return true;
  }
  }

  return false;
}

bool Px4Ctrl::build_external_command(const MissionContext &ctx,
                                     controller::ControlCommand &ctrl_cmd) {
  if (ctx.cmd_msg == nullptr) {
    return false;
  }

  switch (ctx.cmd_msg->type) {
  case px4ctrl_msgs::msg::Command::ROTORS_FORCE:
    spdlog::error("not supported type:ROTORS_FORCE");
    return false;

  case px4ctrl_msgs::msg::Command::THRUST_BODYRATE:
    ctrl_cmd.type = params::ControlType::BODYRATES;
    ctrl_cmd.thrust = se3_controller_->thrustMap(ctx.cmd_msg->u[0]);
    ctrl_cmd.bodyrates =
        Eigen::Vector3d(ctx.cmd_msg->u[1], ctx.cmd_msg->u[2], ctx.cmd_msg->u[3]);
    return true;

  case px4ctrl_msgs::msg::Command::THRUST_TORQUE:
    spdlog::error("not supported type:THRUST_TORQUE");
    return false;

  case px4ctrl_msgs::msg::Command::DESIRED_POS: {
    if (ctx.odom_msg == nullptr || ctx.imu_msg == nullptr) {
      return false;
    }
    controller::DesiredState des;
    const auto &des_pos = ctx.cmd_msg->pos;
    const auto &des_vel = ctx.cmd_msg->vel;
    const auto &des_acc = ctx.cmd_msg->acc;
    const auto &des_jerk = ctx.cmd_msg->jerk;
    const auto &des_quat = ctx.cmd_msg->quat;

    des.p = Eigen::Vector3d(des_pos[0], des_pos[1], des_pos[2]);
    des.v = Eigen::Vector3d(des_vel[0], des_vel[1], des_vel[2]);
    des.a = Eigen::Vector3d(des_acc[0], des_acc[1], des_acc[2]);
    des.j = Eigen::Vector3d(des_jerk[0], des_jerk[1], des_jerk[2]);
    des.q = Eigen::Quaterniond(des_quat[0], des_quat[1], des_quat[2], des_quat[3]);
    des.yaw = ctx.cmd_msg->yaw;

    ctrl_cmd = se3_controller_->runControl(des, *ctx.odom_msg, *ctx.imu_msg);
    estimate_thrust_from_imu();
    return true;
  }

  case px4ctrl_msgs::msg::Command::THRUST_QUAT: {
    ctrl_cmd.type = params::ControlType::ATTITUDE;
    const auto &des_quat = ctx.cmd_msg->quat;
    ctrl_cmd.thrust = se3_controller_->thrustMap(ctx.cmd_msg->u[0]);
    ctrl_cmd.attitude =
        Eigen::Quaterniond(des_quat[0], des_quat[1], des_quat[2], des_quat[3]);
    return true;
  }
  }

  return false;
}

bool Px4Ctrl::build_safe_landing_command(const MissionContext &ctx,
                                         controller::ControlCommand &ctrl_cmd) {
  if (ctx.imu_msg == nullptr) {
    return false;
  }

  if (!landing_.initialized) {
    landing_.start_time = ctx.now;
    landing_.initialized = true;
    landing_.c12_started = false;
  }

  const auto &imu_q = ctx.imu_msg->orientation;
  const Eigen::Quaterniond q_imu(imu_q.w, imu_q.x, imu_q.y, imu_q.z);
  const double yaw = controller::yawFromQuat(q_imu);
  const Eigen::Quaterniond level_q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));

  const auto min_thrust = px4ctrl_params->quadrotor_params.min_thrust;
  const auto max_thrust = px4ctrl_params->quadrotor_params.max_thrust;
  const double hover =
      std::clamp(se3_controller_->getHoverThrustEstimate(), min_thrust, max_thrust);

  const double descent_speed =
      px4ctrl_params->statemachine_params.l2_takeoff_landing_speed;
  const double descent_ratio =
      std::clamp(descent_speed / std::max(1.0, px4ctrl_params->quadrotor_params.g),
                 0.05, 0.25);
  const double safe_thrust =
      std::clamp(hover * (1.0 - descent_ratio), min_thrust, max_thrust);

  ctrl_cmd.type = params::ControlType::ATTITUDE;
  ctrl_cmd.attitude = level_q;
  ctrl_cmd.thrust = safe_thrust;

  estimate_thrust_from_imu();
  return true;
}

void Px4Ctrl::build_command(const MissionContext &ctx, MissionPhase phase,
                            ControlSource source,
                            controller::ControlCommand &ctrl_cmd) {
  bool ok_build = false;
  switch (source) {
  case ControlSource::PROOF_ALIVE:
    build_proof_alive(ctx, ctrl_cmd);
    return;
  case ControlSource::SE3:
    ok_build = build_se3_command(ctx, phase, ctrl_cmd);
    break;
  case ControlSource::SAFE_LANDING:
    ok_build = build_safe_landing_command(ctx, ctrl_cmd);
    break;
  case ControlSource::EXTERNAL_CMD:
    ok_build = build_external_command(ctx, ctrl_cmd);
    if (!ok_build) {
      spdlog::warn("External cmd invalid, fallback to hover controller");
      ok_build = build_se3_command(ctx, MissionPhase::HOVER, ctrl_cmd);
    }
    break;
  }

  if (!ok_build) {
    if (!build_safe_landing_command(ctx, ctrl_cmd)) {
      build_proof_alive(ctx, ctrl_cmd);
    }
  }
}

void Px4Ctrl::apply_control(const controller::ControlCommand &cmd) {
  double thrust =
      std::clamp(cmd.thrust, px4ctrl_params->quadrotor_params.min_thrust,
                 px4ctrl_params->quadrotor_params.max_thrust);

  switch (cmd.type) {
  case px4ctrl::params::ControlType::BODYRATES: {
    Eigen::Vector3d bodyrates =
        cmd.bodyrates.cwiseMax(-px4ctrl_params->quadrotor_params.max_bodyrate)
            .cwiseMin(px4ctrl_params->quadrotor_params.max_bodyrate);

    last_ctrl_cmd = cmd;
    last_ctrl_cmd.type = params::ControlType::BODYRATES;
    last_ctrl_cmd.thrust = thrust;
    last_ctrl_cmd.bodyrates = bodyrates;

    std::array<double, 3> bodyrates_arr = {bodyrates.x(), bodyrates.y(),
                                           bodyrates.z()};
    px4_bridge->pub_bodyrates_target(thrust, bodyrates_arr);
    break;
  }
  case px4ctrl::params::ControlType::ATTITUDE: {
    last_ctrl_cmd = cmd;
    last_ctrl_cmd.type = params::ControlType::ATTITUDE;
    last_ctrl_cmd.thrust = thrust;

    const std::array<double, 4> quat = {cmd.attitude.w(), cmd.attitude.x(),
                                        cmd.attitude.y(), cmd.attitude.z()};
    px4_bridge->pub_attitude_target(thrust, quat);
    break;
  }
  }
}

} // namespace px4ctrl
