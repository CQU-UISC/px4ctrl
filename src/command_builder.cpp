#include "command_builder.h"

#include <algorithm>
#include <cmath>

#include <px4ctrl_msgs/msg/command.hpp>
#include <spdlog/spdlog.h>

#include "params.h"

namespace px4ctrl {

CommandBuilder::CommandBuilder(std::shared_ptr<controller::Se3Control> ctrl,
                               std::shared_ptr<Context> ctx)
    : controller_(std::move(ctrl)), ctx_(std::move(ctx)) {}

// --- Public API ---

ControlSource CommandBuilder::build(const MissionContextSnapshot &snap,
                                    controller::ControlCommand &out) {
  auto source = select_source(snap);
  bool ok = false;

  switch (source) {
  case ControlSource::PROOF_ALIVE:
    build_proof_alive(snap, out);
    return source;
  case ControlSource::SE3:
    ok = build_se3(snap, out);
    break;
  case ControlSource::SAFE_LANDING:
    ok = build_safe_landing(snap, out);
    break;
  case ControlSource::EXTERNAL_CMD:
    ok = build_external(snap, out);
    if (!ok) {
      spdlog::warn("External cmd invalid, fallback to hover controller");
      ok = build_se3(snap, out);
    }
    break;
  }

  if (!ok) {
    if (!build_safe_landing(snap, out)) {
      build_proof_alive(snap, out);
    }
  }

  return source;
}

// --- Control source selection ---

ControlSource
CommandBuilder::select_source(const MissionContextSnapshot &snap) const {
  if (!snap.offboard || !snap.armed) {
    return ControlSource::PROOF_ALIVE;
  }

  if (ctx_->phase == MissionPhase::FAILSAFE) {
    if (ctx_->active_guard_action == params::Guard::HOLD && snap.odom_fresh) {
      return ControlSource::SE3;
    }
    return ControlSource::SAFE_LANDING;
  }

  if (ctx_->phase == MissionPhase::CMD_CTRL && snap.cmd_fresh &&
      ctx_->cmdctrl_hz >= static_cast<int>(
          ctx_->params()->statemachine_params.l2_cmd_ctrl_min_hz)) {
    return ControlSource::EXTERNAL_CMD;
  }

  if (!snap.odom_fresh &&
      (ctx_->phase == MissionPhase::TAKEOFF ||
       ctx_->phase == MissionPhase::HOVER ||
       ctx_->phase == MissionPhase::CMD_CTRL ||
       ctx_->phase == MissionPhase::LANDING)) {
    return ControlSource::SAFE_LANDING;
  }

  return ControlSource::SE3;
}

// --- Command builders ---

void CommandBuilder::build_proof_alive(const MissionContextSnapshot &snap,
                                       controller::ControlCommand &out) const {
  out.type = params::ControlType::ATTITUDE;
  if (snap.imu_msg != nullptr) {
    const auto &quat = snap.imu_msg->orientation;
    out.attitude = Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z);
  } else {
    out.attitude = Eigen::Quaterniond::Identity();
  }
  out.thrust = 0.01;
}

bool CommandBuilder::build_se3(const MissionContextSnapshot &snap,
                               controller::ControlCommand &out) {
  if (snap.odom_msg == nullptr || snap.imu_msg == nullptr) return false;

  const double speed = ctx_->params()->statemachine_params.l2_takeoff_landing_speed;
  controller::DesiredState des;

  switch (ctx_->phase) {
  case MissionPhase::STANDBY:
    out.type = params::ControlType::BODYRATES;
    out.thrust = 0.1;
    out.bodyrates = Eigen::Vector3d::Zero();
    return true;

  case MissionPhase::TAKEOFF: {
    if (!ctx_->takeoff.initialized) return false;
    des.p = ctx_->takeoff.start_pos;
    des.q = ctx_->takeoff.start_q;
    des.yaw = controller::yawFromQuat(des.q);
    des.v = Eigen::Vector3d(0, 0, speed);

    const double elapsed_s =
        timeDuration(ctx_->takeoff.start_time, snap.now) / 1000.0;
    const double target_z =
        ctx_->takeoff.start_pos.z() +
        ctx_->params()->statemachine_params.l2_takeoff_height;
    des.p.z() = ctx_->takeoff.start_pos.z() + speed * elapsed_s;
    if (des.p.z() >= target_z) {
      des.p.z() = target_z;
      des.v = Eigen::Vector3d::Zero();
    }

    out = controller_->runControl(des, *snap.odom_msg, *snap.imu_msg);
    return true;
  }

  case MissionPhase::HOVER:
  case MissionPhase::CMD_CTRL: {
    if (!ctx_->hover.initialized) {
      ctx_->hover.pos = Eigen::Vector3d(snap.odom_msg->pose.pose.position.x,
                                         snap.odom_msg->pose.pose.position.y,
                                         snap.odom_msg->pose.pose.position.z);
      const Eigen::Quaterniond q(snap.odom_msg->pose.pose.orientation.w,
                                  snap.odom_msg->pose.pose.orientation.x,
                                  snap.odom_msg->pose.pose.orientation.y,
                                  snap.odom_msg->pose.pose.orientation.z);
      const auto yaw = controller::yawFromQuat(q);
      ctx_->hover.q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
      ctx_->hover.initialized = true;
    }
    des.p = ctx_->hover.pos;
    des.q = ctx_->hover.q;
    des.yaw = controller::yawFromQuat(des.q);

    out = controller_->runControl(des, *snap.odom_msg, *snap.imu_msg);
    return true;
  }

  case MissionPhase::LANDING:
  case MissionPhase::FAILSAFE: {
    if (ctx_->phase == MissionPhase::FAILSAFE &&
        ctx_->active_guard_action == params::Guard::HOLD) {
      des.p = ctx_->hover.pos;
      des.q = ctx_->hover.q;
      des.yaw = controller::yawFromQuat(des.q);
      out = controller_->runControl(des, *snap.odom_msg, *snap.imu_msg);
      return true;
    }

    if (!ctx_->landing.initialized) {
      ctx_->landing.start_pos =
          Eigen::Vector3d(snap.odom_msg->pose.pose.position.x,
                          snap.odom_msg->pose.pose.position.y,
                          snap.odom_msg->pose.pose.position.z);
      ctx_->landing.start_q =
          Eigen::Quaterniond(snap.odom_msg->pose.pose.orientation.w,
                             snap.odom_msg->pose.pose.orientation.x,
                             snap.odom_msg->pose.pose.orientation.y,
                             snap.odom_msg->pose.pose.orientation.z);
      ctx_->landing.start_time = snap.now;
      ctx_->landing.initialized = true;
      ctx_->landing.c12_started = false;
    }

    des.p = ctx_->landing.start_pos;
    des.q = ctx_->landing.start_q;
    des.yaw = controller::yawFromQuat(des.q);
    des.v = Eigen::Vector3d(0, 0, -speed);
    des.p.z() -= speed * (timeDuration(ctx_->landing.start_time, snap.now) / 1000.0);

    out = controller_->runControl(des, *snap.odom_msg, *snap.imu_msg);
    return true;
  }
  }

  return false;
}

bool CommandBuilder::build_external(const MissionContextSnapshot &snap,
                                    controller::ControlCommand &out) {
  if (snap.cmd_msg == nullptr) return false;

  switch (snap.cmd_msg->type) {
  case px4ctrl_msgs::msg::Command::ROTORS_FORCE:
    spdlog::error("not supported type:ROTORS_FORCE");
    return false;

  case px4ctrl_msgs::msg::Command::THRUST_BODYRATE:
    out.type = params::ControlType::BODYRATES;
    out.thrust = controller_->thrustMap(snap.cmd_msg->u[0]);
    out.bodyrates =
        Eigen::Vector3d(snap.cmd_msg->u[1], snap.cmd_msg->u[2], snap.cmd_msg->u[3]);
    return true;

  case px4ctrl_msgs::msg::Command::THRUST_TORQUE:
    spdlog::error("not supported type:THRUST_TORQUE");
    return false;

  case px4ctrl_msgs::msg::Command::DESIRED_POS: {
    if (snap.odom_msg == nullptr || snap.imu_msg == nullptr) return false;
    controller::DesiredState des;
    const auto &des_pos = snap.cmd_msg->pos;
    const auto &des_vel = snap.cmd_msg->vel;
    const auto &des_acc = snap.cmd_msg->acc;
    const auto &des_jerk = snap.cmd_msg->jerk;
    const auto &des_quat = snap.cmd_msg->quat;

    des.p = Eigen::Vector3d(des_pos[0], des_pos[1], des_pos[2]);
    des.v = Eigen::Vector3d(des_vel[0], des_vel[1], des_vel[2]);
    des.a = Eigen::Vector3d(des_acc[0], des_acc[1], des_acc[2]);
    des.j = Eigen::Vector3d(des_jerk[0], des_jerk[1], des_jerk[2]);
    des.q = Eigen::Quaterniond(des_quat[0], des_quat[1], des_quat[2], des_quat[3]);
    des.yaw = snap.cmd_msg->yaw;

    out = controller_->runControl(des, *snap.odom_msg, *snap.imu_msg);
    return true;
  }

  case px4ctrl_msgs::msg::Command::THRUST_QUAT: {
    out.type = params::ControlType::ATTITUDE;
    const auto &des_quat = snap.cmd_msg->quat;
    out.thrust = controller_->thrustMap(snap.cmd_msg->u[0]);
    out.attitude =
        Eigen::Quaterniond(des_quat[0], des_quat[1], des_quat[2], des_quat[3]);
    return true;
  }
  }

  return false;
}

bool CommandBuilder::build_safe_landing(const MissionContextSnapshot &snap,
                                        controller::ControlCommand &out) {
  if (snap.imu_msg == nullptr) return false;

  if (!ctx_->landing.initialized) {
    ctx_->landing.start_time = snap.now;
    ctx_->landing.initialized = true;
    ctx_->landing.c12_started = false;
  }

  const auto &imu_q = snap.imu_msg->orientation;
  const Eigen::Quaterniond q_imu(imu_q.w, imu_q.x, imu_q.y, imu_q.z);
  const double yaw = controller::yawFromQuat(q_imu);
  const Eigen::Quaterniond level_q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));

  const auto min_thrust = ctx_->params()->quadrotor_params.min_thrust;
  const auto max_thrust = ctx_->params()->quadrotor_params.max_thrust;
  const double hover =
      std::clamp(controller_->getHoverThrustEstimate(), min_thrust, max_thrust);

  const double descent_speed = ctx_->params()->statemachine_params.l2_takeoff_landing_speed;
  const double descent_ratio =
      std::clamp(descent_speed / std::max(1.0, ctx_->params()->quadrotor_params.g),
                 0.05, 0.25);
  const double safe_thrust =
      std::clamp(hover * (1.0 - descent_ratio), min_thrust, max_thrust);

  out.type = params::ControlType::ATTITUDE;
  out.attitude = level_q;
  out.thrust = safe_thrust;

  return true;
}

} // namespace px4ctrl
