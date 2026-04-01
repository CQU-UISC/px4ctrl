#include "fsm.h"

#include <cmath>

#include "fsm_internal.h"

namespace px4ctrl {

void Px4Ctrl::guard(const MissionContext &ctx, GuardDecision &decision) {
  decision = GuardDecision{};

  params::Guard selected = params::Guard::HOLD;
  std::string selected_reason;
  uint32_t selected_flag = 0;

  auto select_guard = [&](const params::Guard action, const std::string &reason,
                          const uint32_t flag) {
    if (fsm_internal::guardSeverity(action) >=
        fsm_internal::guardSeverity(selected)) {
      selected = action;
      selected_reason = reason;
    }
    selected_flag |= flag;
  };

  if (px4ctrl_params->guard_params.use_rc && !ctx.rc_valid) {
    if (ctx.armed || armed_state_) {
      select_guard(px4ctrl_params->guard_params.rc_triggered, "rc signal lost",
                   fsm_internal::kGuardRcLost);
    } else {
      decision.rc_required_block = true;
      selected_flag |= fsm_internal::kGuardRcRequired;
    }
  }

  if (ctx.state_msg == nullptr ||
      ctx.state_age_ms > px4ctrl_params->guard_params.mavros_timeout) {
    select_guard(px4ctrl_params->guard_params.mavros_triggered,
                 "mavros timeout", fsm_internal::kGuardMavrosTimeout);
  }

  if (ctx.odom_msg == nullptr ||
      ctx.odom_age_ms > px4ctrl_params->guard_params.odom_timeout) {
    select_guard(px4ctrl_params->guard_params.localization_loss_triggered,
                 "odom timeout", fsm_internal::kGuardOdomTimeout);
  } else if (odom_hz < static_cast<int>(px4ctrl_params->guard_params.odom_min_hz)) {
    select_guard(px4ctrl_params->guard_params.odom_triggered, "odom low hz",
                 fsm_internal::kGuardOdomLowHz);
  }

  const bool in_flight =
      (ctx.offboard || offboard_state_) && (ctx.armed || armed_state_);
  if (in_flight && has_client_cmd &&
      timeDuration(last_client_cmd_time, ctx.now) >
          px4ctrl_params->guard_params.ui_timeout) {
    select_guard(px4ctrl_params->guard_params.ui_triggered, "ui timeout",
                 fsm_internal::kGuardUiTimeout);
  }

  if (ctx.battery_msg != nullptr &&
      ctx.battery_msg->voltage < px4ctrl_params->guard_params.low_battery_voltage) {
    select_guard(px4ctrl_params->guard_params.lowvolt_triggered, "low battery",
                 fsm_internal::kGuardLowBattery);
  }

  if (ctx.odom_msg != nullptr) {
    const auto &pose = ctx.odom_msg->pose.pose;
    const Eigen::Vector3d pos(pose.position.x, pose.position.y, pose.position.z);
    const Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x,
                               pose.orientation.y, pose.orientation.z);
    const Eigen::Vector3d vel(ctx.odom_msg->twist.twist.linear.x,
                              ctx.odom_msg->twist.twist.linear.y,
                              ctx.odom_msg->twist.twist.linear.z);

    if (px4ctrl_params->guard_params.enable_geofence) {
      const auto &mn = px4ctrl_params->guard_params.geofence_min;
      const auto &mx = px4ctrl_params->guard_params.geofence_max;
      const bool out_of_geo = pos.x() < mn[0] || pos.y() < mn[1] ||
                              pos.z() < mn[2] || pos.x() > mx[0] ||
                              pos.y() > mx[1] || pos.z() > mx[2];
      if (out_of_geo) {
        select_guard(px4ctrl_params->guard_params.geofence_triggered,
                     "geofence violated", fsm_internal::kGuardGeofence);
      }
    }

    if (px4ctrl_params->guard_params.enable_attitude_fence) {
      const auto rpy_deg = fsm_internal::quatToRpyDeg(q);
      const double roll_deg = std::abs(rpy_deg[0]);
      const double pitch_deg = std::abs(rpy_deg[1]);
      const double yaw_deg = std::abs(rpy_deg[2]);

      const auto over_limit = [&](const double angle_deg, const double limit_deg) {
        return limit_deg != -1.0 && angle_deg > limit_deg;
      };

      if (over_limit(roll_deg, px4ctrl_params->guard_params.max_roll_deg) ||
          over_limit(pitch_deg, px4ctrl_params->guard_params.max_pitch_deg) ||
          over_limit(yaw_deg, px4ctrl_params->guard_params.max_yaw_deg)) {
        select_guard(px4ctrl_params->guard_params.attitude_triggered,
                     "attitude fence violated",
                     fsm_internal::kGuardAttitudeFence);
      }
    }

    if (px4ctrl_params->guard_params.enable_velocity_fence &&
        vel.norm() > px4ctrl_params->guard_params.max_velocity_norm) {
      select_guard(px4ctrl_params->guard_params.velocity_triggered,
                   "velocity fence violated", fsm_internal::kGuardVelocityFence);
    }
  }

  decision.flags = selected_flag;
  decision.action = selected;
  decision.reason = selected_reason;
  decision.triggered = !selected_reason.empty();

  guard_flags = decision.flags;

  if (decision.triggered) {
    const bool same_guard =
        has_last_guard_ && decision.action == last_guard_action_ &&
        decision.reason == last_guard_reason_ &&
        decision.flags == last_guard_flags_;

    if (!same_guard) {
      spdlog::warn("Guard triggered action:{} reason:{} guards:{}",
                   fsm_internal::guardActionName(decision.action), decision.reason,
                   fsm_internal::guardFlagsToString(decision.flags));
      has_last_guard_ = true;
      last_guard_action_ = decision.action;
      last_guard_reason_ = decision.reason;
      last_guard_flags_ = decision.flags;
      guard_repeat_suppressed_ = 0;
      last_guard_repeat_log_time_ = ctx.now;
    } else if (timeDuration(last_guard_repeat_log_time_, ctx.now) > 2000) {
      spdlog::warn(
          "Guard still active action:{} reason:{} guards:{} (suppressed {} repeats)",
          fsm_internal::guardActionName(decision.action), decision.reason,
          fsm_internal::guardFlagsToString(decision.flags),
          guard_repeat_suppressed_);
      guard_repeat_suppressed_ = 0;
      last_guard_repeat_log_time_ = ctx.now;
    } else {
      ++guard_repeat_suppressed_;
    }
  } else {
    has_last_guard_ = false;
    guard_repeat_suppressed_ = 0;
  }

  if (decision.rc_required_block && timePassed(last_guard_log_time) > 1000) {
    spdlog::warn("use_rc enabled: waiting for valid RC signal before start");
    last_guard_log_time = ctx.now;
  }
}

Px4Ctrl::MissionPhase Px4Ctrl::evaluate_phase(const MissionContext &ctx,
                                              const GuardDecision &decision) {
  MissionPhase next = phase_;
  const auto request = requested_phase_;
  requested_phase_.reset();

  if (!ctx.offboard || !ctx.armed) {
    return (px4ctrl_params->guard_params.use_rc && !ctx.rc_valid)
               ? MissionPhase::WAIT_FOR_RC
               : MissionPhase::STANDBY;
  }

  if (decision.triggered) {
    active_guard_action_ = decision.action;
    if (decision.action == params::Guard::DISARM) {
      if (!px4_bridge->force_disarm()) {
        spdlog::error("Guard force disarm failed");
      }
      se3_controller_->resetThrustMapping();
      return MissionPhase::STANDBY;
    }
    if (phase_ == MissionPhase::FAILSAFE && landing_.initialized &&
        timeDuration(landing_.start_time, ctx.now) >
            px4ctrl_params->guard_params.land_timeout) {
      spdlog::warn("Failsafe landing timeout reached, force disarm");
      if (!px4_bridge->force_disarm()) {
        spdlog::error("Failsafe timeout disarm failed");
      }
      se3_controller_->resetThrustMapping();
      return MissionPhase::STANDBY;
    }
    return MissionPhase::FAILSAFE;
  }

  if (next == MissionPhase::WAIT_FOR_RC) {
    next = MissionPhase::STANDBY;
  }

  if (next == MissionPhase::FAILSAFE) {
    next = MissionPhase::HOVER;
  }

  if (request.has_value()) {
    switch (*request) {
    case MissionPhase::TAKEOFF:
      if (next == MissionPhase::STANDBY) {
        next = MissionPhase::TAKEOFF;
      }
      break;
    case MissionPhase::HOVER:
      if (next != MissionPhase::STANDBY && next != MissionPhase::WAIT_FOR_RC) {
        next = MissionPhase::HOVER;
      }
      break;
    case MissionPhase::CMD_CTRL_READY:
      if (next == MissionPhase::HOVER) {
        next = MissionPhase::CMD_CTRL_READY;
      }
      break;
    case MissionPhase::LANDING:
      if (next != MissionPhase::STANDBY && next != MissionPhase::WAIT_FOR_RC) {
        next = MissionPhase::LANDING;
      }
      break;
    default:
      break;
    }
  }

  switch (next) {
  case MissionPhase::TAKEOFF:
    if (check_takeoff_finished(ctx)) {
      next = MissionPhase::HOVER;
    }
    break;
  case MissionPhase::CMD_CTRL_READY:
    if (ctx.cmd_fresh &&
        cmdctrl_hz >= static_cast<int>(
                          px4ctrl_params->statemachine_params.l2_cmd_ctrl_min_hz)) {
      next = MissionPhase::CMD_CTRL;
    }
    break;
  case MissionPhase::CMD_CTRL:
    if (!ctx.cmd_fresh ||
        cmdctrl_hz < static_cast<int>(
                         px4ctrl_params->statemachine_params.l2_cmd_ctrl_min_hz)) {
      next = MissionPhase::HOVER;
    }
    break;
  case MissionPhase::LANDING:
    if (check_landing_finished(ctx)) {
      if (!px4_bridge->force_disarm()) {
        spdlog::error("Failed to force disarm after landing");
      }
      se3_controller_->resetThrustMapping();
      next = MissionPhase::STANDBY;
    }
    break;
  case MissionPhase::FAILSAFE:
  case MissionPhase::WAIT_FOR_RC:
  case MissionPhase::STANDBY:
  case MissionPhase::HOVER:
    break;
  }

  return next;
}

Px4Ctrl::ControlSource
Px4Ctrl::select_control_source(const MissionContext &ctx, MissionPhase phase,
                               const GuardDecision &decision) const {
  (void)decision;
  if (!ctx.offboard || !ctx.armed) {
    return ControlSource::PROOF_ALIVE;
  }

  if (phase == MissionPhase::FAILSAFE) {
    if (active_guard_action_ == params::Guard::HOLD && ctx.odom_fresh) {
      return ControlSource::SE3;
    }
    return ControlSource::SAFE_LANDING;
  }

  if (phase == MissionPhase::CMD_CTRL && ctx.cmd_fresh &&
      cmdctrl_hz >= static_cast<int>(
                        px4ctrl_params->statemachine_params.l2_cmd_ctrl_min_hz)) {
    return ControlSource::EXTERNAL_CMD;
  }

  if (!ctx.odom_fresh &&
      (phase == MissionPhase::TAKEOFF || phase == MissionPhase::HOVER ||
       phase == MissionPhase::CMD_CTRL_READY || phase == MissionPhase::LANDING)) {
    return ControlSource::SAFE_LANDING;
  }

  return ControlSource::SE3;
}

void Px4Ctrl::on_phase_enter(MissionPhase phase, const MissionContext &ctx,
                             const GuardDecision &decision) {
  phase_enter_time_ = ctx.now;

  switch (phase) {
  case MissionPhase::WAIT_FOR_RC:
  case MissionPhase::STANDBY:
    takeoff_.initialized = false;
    landing_.initialized = false;
    break;

  case MissionPhase::TAKEOFF: {
    if (ctx.odom_msg != nullptr) {
      takeoff_.start_pos =
          Eigen::Vector3d(ctx.odom_msg->pose.pose.position.x,
                          ctx.odom_msg->pose.pose.position.y,
                          ctx.odom_msg->pose.pose.position.z);
      takeoff_.start_q =
          Eigen::Quaterniond(ctx.odom_msg->pose.pose.orientation.w,
                             ctx.odom_msg->pose.pose.orientation.x,
                             ctx.odom_msg->pose.pose.orientation.y,
                             ctx.odom_msg->pose.pose.orientation.z);
    } else {
      takeoff_.start_pos = hover_.pos;
      if (ctx.imu_msg != nullptr) {
        const auto &q = ctx.imu_msg->orientation;
        takeoff_.start_q = Eigen::Quaterniond(q.w, q.x, q.y, q.z);
      } else {
        takeoff_.start_q = Eigen::Quaterniond::Identity();
      }
    }
    takeoff_.start_time = ctx.now;
    takeoff_.initialized = true;
    spdlog::info("Taking off from:{} {} {}", takeoff_.start_pos.x(),
                 takeoff_.start_pos.y(), takeoff_.start_pos.z());
    break;
  }

  case MissionPhase::HOVER: {
    landing_.initialized = false;
    landing_.c12_started = false;
    if (last_phase_ == MissionPhase::TAKEOFF && takeoff_.initialized) {
      hover_.pos = takeoff_.start_pos;
      hover_.pos.z() += px4ctrl_params->statemachine_params.l2_takeoff_height;
      hover_.q = takeoff_.start_q;
    } else if (ctx.odom_msg != nullptr) {
      hover_.pos = Eigen::Vector3d(ctx.odom_msg->pose.pose.position.x,
                                   ctx.odom_msg->pose.pose.position.y,
                                   ctx.odom_msg->pose.pose.position.z);
      const Eigen::Quaterniond q(ctx.odom_msg->pose.pose.orientation.w,
                                 ctx.odom_msg->pose.pose.orientation.x,
                                 ctx.odom_msg->pose.pose.orientation.y,
                                 ctx.odom_msg->pose.pose.orientation.z);
      const auto yaw = controller::yawFromQuat(q);
      hover_.q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    }
    hover_.initialized = true;
    spdlog::info("Hovering at->X:{} Y:{} Z:{}", hover_.pos.x(), hover_.pos.y(),
                 hover_.pos.z());
    break;
  }

  case MissionPhase::CMD_CTRL_READY:
    landing_.initialized = false;
    landing_.c12_started = false;
    if (ctx.odom_msg != nullptr) {
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
    break;

  case MissionPhase::CMD_CTRL:
    break;

  case MissionPhase::LANDING:
  case MissionPhase::FAILSAFE: {
    if (phase == MissionPhase::FAILSAFE && decision.action == params::Guard::HOLD) {
      if (ctx.odom_msg != nullptr) {
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
      break;
    }

    if (ctx.odom_msg != nullptr) {
      landing_.start_pos =
          Eigen::Vector3d(ctx.odom_msg->pose.pose.position.x,
                          ctx.odom_msg->pose.pose.position.y,
                          ctx.odom_msg->pose.pose.position.z);
      landing_.start_q =
          Eigen::Quaterniond(ctx.odom_msg->pose.pose.orientation.w,
                             ctx.odom_msg->pose.pose.orientation.x,
                             ctx.odom_msg->pose.pose.orientation.y,
                             ctx.odom_msg->pose.pose.orientation.z);
    } else {
      landing_.start_pos = hover_.pos;
      if (ctx.imu_msg != nullptr) {
        const auto &q = ctx.imu_msg->orientation;
        landing_.start_q = Eigen::Quaterniond(q.w, q.x, q.y, q.z);
      } else {
        landing_.start_q = Eigen::Quaterniond::Identity();
      }
    }
    landing_.start_time = ctx.now;
    landing_.initialized = true;
    landing_.c12_started = false;
    break;
  }
  }
}

bool Px4Ctrl::check_takeoff_finished(const MissionContext &ctx) const {
  if (!takeoff_.initialized || ctx.odom_msg == nullptr) {
    return false;
  }
  const Eigen::Vector3d cur_pos(ctx.odom_msg->pose.pose.position.x,
                                ctx.odom_msg->pose.pose.position.y,
                                ctx.odom_msg->pose.pose.position.z);
  Eigen::Vector3d des_pos = takeoff_.start_pos;
  des_pos.z() += px4ctrl_params->statemachine_params.l2_takeoff_height;
  return (cur_pos - des_pos).norm() < 0.1;
}

bool Px4Ctrl::check_landing_finished(const MissionContext &ctx) {
  if (!landing_.initialized) {
    return false;
  }

  const auto elapsed_ms = timeDuration(landing_.start_time, ctx.now);
  if (elapsed_ms > px4ctrl_params->guard_params.land_timeout) {
    spdlog::warn("Landing timeout reached, forcing disarm path");
    return true;
  }

  if (ctx.odom_msg == nullptr || !ctx.odom_fresh) {
    return false;
  }

  const double speed =
      px4ctrl_params->statemachine_params.l2_takeoff_landing_speed;
  const double des_z = landing_.start_pos.z() - speed * (elapsed_ms / 1000.0);

  const Eigen::Vector3d vel(ctx.odom_msg->twist.twist.linear.x,
                            ctx.odom_msg->twist.twist.linear.y,
                            ctx.odom_msg->twist.twist.linear.z);

  const double POSITION_DEVIATION_C =
      px4ctrl_params->statemachine_params.l2_land_position_deviation_c;
  const double VELOCITY_THR_C =
      px4ctrl_params->statemachine_params.l2_land_velocity_thr_c;
  const double TIME_KEEP_C = px4ctrl_params->statemachine_params.l2_land_time_keep_c;

  const bool c12_satisfy =
      (des_z - ctx.odom_msg->pose.pose.position.z) < POSITION_DEVIATION_C &&
      vel.norm() < VELOCITY_THR_C;

  if (c12_satisfy) {
    if (!landing_.c12_started) {
      landing_.c12_reached_time = ctx.now;
      landing_.c12_started = true;
    }
    if (timeDuration(landing_.c12_reached_time, ctx.now) > TIME_KEEP_C) {
      spdlog::info("Successfully landed");
      return true;
    }
  } else {
    landing_.c12_started = false;
  }

  return false;
}

} // namespace px4ctrl
