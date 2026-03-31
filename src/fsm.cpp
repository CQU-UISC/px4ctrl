#include "fsm.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstring>
#include <mavros_msgs/msg/state.hpp>
#include <memory>
#include <px4ctrl_msgs/msg/command.hpp>
#include <thread>

namespace px4ctrl {
namespace {
constexpr uint32_t kGuardMavrosTimeout = 1u << 0;
constexpr uint32_t kGuardOdomTimeout = 1u << 1;
constexpr uint32_t kGuardUiTimeout = 1u << 2;
constexpr uint32_t kGuardLowBattery = 1u << 3;
constexpr uint32_t kGuardGeofence = 1u << 4;
constexpr uint32_t kGuardAttitudeFence = 1u << 5;
constexpr uint32_t kGuardVelocityFence = 1u << 6;
constexpr uint32_t kGuardOdomLowHz = 1u << 7;
constexpr uint32_t kGuardRcLost = 1u << 8;
constexpr uint32_t kGuardRcRequired = 1u << 9;
constexpr double kRadToDeg = 57.29577951308232;

int guardSeverity(const params::Guard action) {
  switch (action) {
  case params::Guard::HOLD:
    return 1;
  case params::Guard::LAND:
    return 2;
  case params::Guard::DISARM:
    return 3;
  }
  return 0;
}

double normalize_deg(double rad) {
  return std::atan2(std::sin(rad), std::cos(rad)) * kRadToDeg;
}

std::array<double, 3> quatToRpyDeg(const Eigen::Quaterniond &q_in) {
  constexpr double kHalfPi = 1.5707963267948966;
  const Eigen::Quaterniond q = q_in.normalized();

  const double sinr_cosp = 2.0 * (q.w() * q.x() + q.y() * q.z());
  const double cosr_cosp = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
  const double roll = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
  const double pitch =
      (std::abs(sinp) >= 1.0) ? std::copysign(kHalfPi, sinp) : std::asin(sinp);

  const double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
  const double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  const double yaw = std::atan2(siny_cosp, cosy_cosp);

  return {normalize_deg(roll), normalize_deg(pitch), normalize_deg(yaw)};
}

const char *phaseName(Px4Ctrl::MissionPhase phase) {
  switch (phase) {
  case Px4Ctrl::MissionPhase::WAIT_FOR_RC:
    return "WAIT_FOR_RC";
  case Px4Ctrl::MissionPhase::STANDBY:
    return "STANDBY";
  case Px4Ctrl::MissionPhase::TAKEOFF:
    return "TAKEOFF";
  case Px4Ctrl::MissionPhase::HOVER:
    return "HOVER";
  case Px4Ctrl::MissionPhase::CMD_CTRL_READY:
    return "CMD_CTRL_READY";
  case Px4Ctrl::MissionPhase::CMD_CTRL:
    return "CMD_CTRL";
  case Px4Ctrl::MissionPhase::LANDING:
    return "LANDING";
  case Px4Ctrl::MissionPhase::FAILSAFE:
    return "FAILSAFE";
  }
  return "UNKNOWN";
}

const char *sourceName(Px4Ctrl::ControlSource source) {
  switch (source) {
  case Px4Ctrl::ControlSource::PROOF_ALIVE:
    return "PROOF_ALIVE";
  case Px4Ctrl::ControlSource::SE3:
    return "SE3";
  case Px4Ctrl::ControlSource::SAFE_LANDING:
    return "SAFE_LANDING";
  case Px4Ctrl::ControlSource::EXTERNAL_CMD:
    return "EXTERNAL_CMD";
  }
  return "UNKNOWN";
}

const char *guardActionName(const params::Guard action) {
  switch (action) {
  case params::Guard::HOLD:
    return "HOLD";
  case params::Guard::LAND:
    return "LAND";
  case params::Guard::DISARM:
    return "DISARM";
  }
  return "UNKNOWN";
}

std::string guardFlagsToString(const uint32_t flags) {
  if (flags == 0) {
    return "none";
  }

  std::string out;
  auto append = [&](const char *name) {
    if (!out.empty()) {
      out += "|";
    }
    out += name;
  };

  if (flags & kGuardMavrosTimeout) {
    append("mavros_timeout");
  }
  if (flags & kGuardOdomTimeout) {
    append("odom_timeout");
  }
  if (flags & kGuardUiTimeout) {
    append("ui_timeout");
  }
  if (flags & kGuardLowBattery) {
    append("low_battery");
  }
  if (flags & kGuardGeofence) {
    append("geofence");
  }
  if (flags & kGuardAttitudeFence) {
    append("attitude_fence");
  }
  if (flags & kGuardVelocityFence) {
    append("velocity_fence");
  }
  if (flags & kGuardOdomLowHz) {
    append("odom_low_hz");
  }
  if (flags & kGuardRcLost) {
    append("rc_lost");
  }
  if (flags & kGuardRcRequired) {
    append("rc_required");
  }
  return out;
}
} // namespace

Px4Ctrl::Px4Ctrl(std::shared_ptr<Px4CtrlRosBridge> px4_bridge,
                 std::shared_ptr<Px4State> px4_state,
                 std::shared_ptr<Px4CtrlParams> px4ctrl_params,
                 std::shared_ptr<ui::Px4Server> px4_server)
    : px4_bridge(std::move(px4_bridge)), px4_state(std::move(px4_state)),
      px4_server(std::move(px4_server)),
      px4ctrl_params(std::move(px4ctrl_params)) {
  assert(this->px4_bridge != nullptr && this->px4_state != nullptr &&
         this->px4ctrl_params != nullptr);
  if (!init()) {
    spdlog::error("Px4Ctrl init failed");
    throw std::runtime_error("Px4Ctrl init failed");
  }
  se3_controller_ = std::make_shared<controller::Se3Control>(
      this->px4ctrl_params->control_params,
      this->px4ctrl_params->quadrotor_params);
}

bool Px4Ctrl::init() {
  phase_ = MissionPhase::STANDBY;
  last_phase_ = phase_;
  phase_enter_time_ = clock::now();

  requested_phase_.reset();
  allow_cmdctrl_pub_state_ = false;
  guard_flags = 0;
  telemetry_seq = 0;
  has_client_cmd = false;

  const auto now = clock::now();
  last_client_cmd_time = now;
  last_guard_log_time = now;
  last_guard_repeat_log_time_ = now;
  last_log_state_time = now;
  odom_last_time = now;
  cmdctrl_last_time = now;
  last_guard_action_ = params::Guard::HOLD;
  last_guard_reason_.clear();
  last_guard_flags_ = 0;
  guard_repeat_suppressed_ = 0;
  has_last_guard_ = false;
  odom_low_log_time_ = now;
  odom_low_suppressed_ = 0;
  odom_low_active_ = false;

  offboard_state_ = false;
  armed_state_ = false;
  roll_deg_ = 0.0;
  pitch_deg_ = 0.0;
  yaw_deg_ = 0.0;

  last_ctrl_cmd.type = params::ControlType::ATTITUDE;
  last_ctrl_cmd.attitude = Eigen::Quaterniond::Identity();
  last_ctrl_cmd.bodyrates = Eigen::Vector3d::Zero();
  last_ctrl_cmd.thrust = 0.0;

  return true;
}

void Px4Ctrl::stop() { ok = false; }

void Px4Ctrl::run() {
  int delta_t =
      static_cast<int>(1000 / std::max<uint32_t>(1, px4ctrl_params->statemachine_params.freq));

  odom_hold = px4_state->odom->observe([&](auto &) { odom_count++; });
  ctrl_hold = px4_state->ctrl_command->observe([&](auto &) { cmdctrl_count++; });
  client_hold = px4_server->client_data.observe([&](auto &cmd) {
    client_command_callback(cmd);
  });

  while (ok) {
    px4_bridge->spin_once();
    compute_hz();
    process();
    px4_server->pub(fill_server_payload());
    std::this_thread::sleep_for(std::chrono::milliseconds(delta_t));
  }
}

void Px4Ctrl::compute_hz() {
  if (timePassed(odom_last_time) > 100) {
    odom_hz = odom_count * 10;
    odom_count = 0;
    odom_last_time = clock::now();
    if (odom_hz < 100) {
      if (!odom_low_active_) {
        spdlog::warn("odom hz low:{}", odom_hz);
        odom_low_active_ = true;
        odom_low_suppressed_ = 0;
        odom_low_log_time_ = odom_last_time;
      } else if (timeDuration(odom_low_log_time_, odom_last_time) > 2000) {
        spdlog::warn("odom hz low:{} (suppressed {} repeated logs)", odom_hz,
                     odom_low_suppressed_);
        odom_low_suppressed_ = 0;
        odom_low_log_time_ = odom_last_time;
      } else {
        ++odom_low_suppressed_;
      }
    } else if (odom_low_active_) {
      spdlog::info("odom hz recovered:{} (suppressed {} repeated low-hz logs)",
                   odom_hz, odom_low_suppressed_);
      odom_low_active_ = false;
      odom_low_suppressed_ = 0;
      odom_low_log_time_ = odom_last_time;
    }
  }
  if (timePassed(cmdctrl_last_time) > 100) {
    cmdctrl_hz = cmdctrl_count * 10;
    cmdctrl_count = 0;
    cmdctrl_last_time = clock::now();
  }
}

Px4Ctrl::MissionContext Px4Ctrl::build_context() const {
  MissionContext ctx;
  ctx.now = clock::now();

  const auto state = px4_state->state->value();
  const auto ext_state = px4_state->ext_state->value();
  const auto odom = px4_state->odom->value();
  const auto imu = px4_state->imu->value();
  const auto battery = px4_state->battery->value();
  const auto cmd = px4_state->ctrl_command->value();

  ctx.state_msg = state.first;
  ctx.ext_state_msg = ext_state.first;
  ctx.odom_msg = odom.first;
  ctx.imu_msg = imu.first;
  ctx.battery_msg = battery.first;
  ctx.cmd_msg = cmd.first;

  ctx.state_age_ms =
      (ctx.state_msg != nullptr) ? timeDuration(state.second, ctx.now) : -1.0;
  ctx.odom_age_ms =
      (ctx.odom_msg != nullptr) ? timeDuration(odom.second, ctx.now) : -1.0;
  ctx.imu_age_ms =
      (ctx.imu_msg != nullptr) ? timeDuration(imu.second, ctx.now) : -1.0;
  ctx.battery_age_ms =
      (ctx.battery_msg != nullptr) ? timeDuration(battery.second, ctx.now) : -1.0;
  ctx.cmd_age_ms =
      (ctx.cmd_msg != nullptr) ? timeDuration(cmd.second, ctx.now) : -1.0;

  if (ctx.state_msg != nullptr) {
    ctx.offboard =
        ctx.state_msg->mode == mavros_msgs::msg::State::MODE_PX4_OFFBOARD;
    ctx.armed = ctx.state_msg->armed;
  }

  ctx.odom_fresh = ctx.odom_msg != nullptr &&
                   ctx.odom_age_ms <= px4ctrl_params->guard_params.odom_timeout;

  const double cmd_fresh_ms =
      std::max(100.0,
               2000.0 /
                   std::max(1.0, px4ctrl_params->statemachine_params.l2_cmd_ctrl_min_hz));
  ctx.cmd_fresh = ctx.cmd_msg != nullptr && ctx.cmd_age_ms <= cmd_fresh_ms;

  ctx.rc_valid = has_valid_rc_signal();
  return ctx;
}

bool Px4Ctrl::has_valid_rc_signal() const {
  const auto rcin = px4_state->rcin->value();
  if (rcin.first == nullptr) {
    return false;
  }
  if (timeDuration(rcin.second, clock::now()) >
      px4ctrl_params->guard_params.rc_timeout) {
    return false;
  }
  if (rcin.first->channels.empty()) {
    return false;
  }
  return true;
}

bool Px4Ctrl::validate_safety_limit(double v) const {
  return v == -1.0 || (v > 0.0 && v <= 180.0);
}

void Px4Ctrl::update_safety_limits(const ui::SafetyLimitsPayload &limits) {
  const bool limit_valid = validate_safety_limit(limits.max_roll_deg) &&
                           validate_safety_limit(limits.max_pitch_deg) &&
                           validate_safety_limit(limits.max_yaw_deg);
  if (!limit_valid) {
    spdlog::warn("Reject SET_SAFETY_LIMITS: invalid roll/pitch/yaw limits");
    return;
  }

  const Eigen::Vector3d geo_min(limits.geofence_min[0], limits.geofence_min[1],
                                limits.geofence_min[2]);
  const Eigen::Vector3d geo_max(limits.geofence_max[0], limits.geofence_max[1],
                                limits.geofence_max[2]);
  if ((geo_min.array() > geo_max.array()).any()) {
    spdlog::warn("Reject SET_SAFETY_LIMITS: geofence min > max");
    return;
  }

  px4ctrl_params->guard_params.geofence_min = {
      limits.geofence_min[0], limits.geofence_min[1], limits.geofence_min[2]};
  px4ctrl_params->guard_params.geofence_max = {
      limits.geofence_max[0], limits.geofence_max[1], limits.geofence_max[2]};
  px4ctrl_params->guard_params.max_roll_deg = limits.max_roll_deg;
  px4ctrl_params->guard_params.max_pitch_deg = limits.max_pitch_deg;
  px4ctrl_params->guard_params.max_yaw_deg = limits.max_yaw_deg;
  px4ctrl_params->guard_params.enable_geofence = limits.enable_geofence != 0;
  px4ctrl_params->guard_params.enable_attitude_fence =
      limits.enable_attitude_fence != 0;

  spdlog::info(
      "Applied SET_SAFETY_LIMITS: geofence[{}, {}, {}]-[{}, {}, {}], "
      "max_rpy_deg[{},{},{}], en_geo={}, en_att={}",
      limits.geofence_min[0], limits.geofence_min[1], limits.geofence_min[2],
      limits.geofence_max[0], limits.geofence_max[1], limits.geofence_max[2],
      limits.max_roll_deg, limits.max_pitch_deg, limits.max_yaw_deg,
      static_cast<int>(limits.enable_geofence),
      static_cast<int>(limits.enable_attitude_fence));
}

void Px4Ctrl::guard(const MissionContext &ctx, GuardDecision &decision) {
  decision = GuardDecision{};

  params::Guard selected = params::Guard::HOLD;
  std::string selected_reason;
  uint32_t selected_flag = 0;

  auto select_guard = [&](const params::Guard action, const std::string &reason,
                          const uint32_t flag) {
    if (guardSeverity(action) >= guardSeverity(selected)) {
      selected = action;
      selected_reason = reason;
    }
    selected_flag |= flag;
  };

  if (px4ctrl_params->guard_params.use_rc && !ctx.rc_valid) {
    if (ctx.armed || armed_state_) {
      select_guard(px4ctrl_params->guard_params.rc_triggered, "rc signal lost",
                   kGuardRcLost);
    } else {
      decision.rc_required_block = true;
      selected_flag |= kGuardRcRequired;
    }
  }

  if (ctx.state_msg == nullptr ||
      ctx.state_age_ms > px4ctrl_params->guard_params.mavros_timeout) {
    select_guard(px4ctrl_params->guard_params.mavros_triggered,
                 "mavros timeout", kGuardMavrosTimeout);
  }

  if (ctx.odom_msg == nullptr ||
      ctx.odom_age_ms > px4ctrl_params->guard_params.odom_timeout) {
    select_guard(px4ctrl_params->guard_params.localization_loss_triggered,
                 "odom timeout", kGuardOdomTimeout);
  } else if (odom_hz < static_cast<int>(px4ctrl_params->guard_params.odom_min_hz)) {
    select_guard(px4ctrl_params->guard_params.odom_triggered, "odom low hz",
                 kGuardOdomLowHz);
  }

  const bool in_flight = (ctx.offboard || offboard_state_) && (ctx.armed || armed_state_);
  if (in_flight && has_client_cmd &&
      timeDuration(last_client_cmd_time, ctx.now) >
          px4ctrl_params->guard_params.ui_timeout) {
    select_guard(px4ctrl_params->guard_params.ui_triggered, "ui timeout",
                 kGuardUiTimeout);
  }

  if (ctx.battery_msg != nullptr &&
      ctx.battery_msg->voltage <
          px4ctrl_params->guard_params.low_battery_voltage) {
    select_guard(px4ctrl_params->guard_params.lowvolt_triggered, "low battery",
                 kGuardLowBattery);
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
      const bool out_of_geo =
          pos.x() < mn[0] || pos.y() < mn[1] || pos.z() < mn[2] || pos.x() > mx[0] ||
          pos.y() > mx[1] || pos.z() > mx[2];
      if (out_of_geo) {
        select_guard(px4ctrl_params->guard_params.geofence_triggered,
                     "geofence violated", kGuardGeofence);
      }
    }

    if (px4ctrl_params->guard_params.enable_attitude_fence) {
      const auto rpy_deg = quatToRpyDeg(q);
      const double roll_deg = std::abs(rpy_deg[0]);
      const double pitch_deg = std::abs(rpy_deg[1]);
      const double yaw_deg = std::abs(rpy_deg[2]);

      const auto over_limit = [&](double angle_deg, double limit_deg) {
        return limit_deg != -1.0 && angle_deg > limit_deg;
      };

      if (over_limit(roll_deg, px4ctrl_params->guard_params.max_roll_deg) ||
          over_limit(pitch_deg, px4ctrl_params->guard_params.max_pitch_deg) ||
          over_limit(yaw_deg, px4ctrl_params->guard_params.max_yaw_deg)) {
        select_guard(px4ctrl_params->guard_params.attitude_triggered,
                     "attitude fence violated", kGuardAttitudeFence);
      }
    }

    if (px4ctrl_params->guard_params.enable_velocity_fence &&
        vel.norm() > px4ctrl_params->guard_params.max_velocity_norm) {
      select_guard(px4ctrl_params->guard_params.velocity_triggered,
                   "velocity fence violated", kGuardVelocityFence);
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
                   guardActionName(decision.action), decision.reason,
                   guardFlagsToString(decision.flags));
      has_last_guard_ = true;
      last_guard_action_ = decision.action;
      last_guard_reason_ = decision.reason;
      last_guard_flags_ = decision.flags;
      guard_repeat_suppressed_ = 0;
      last_guard_repeat_log_time_ = ctx.now;
    } else if (timeDuration(last_guard_repeat_log_time_, ctx.now) > 2000) {
      spdlog::warn(
          "Guard still active action:{} reason:{} guards:{} (suppressed {} repeats)",
          guardActionName(decision.action), decision.reason,
          guardFlagsToString(decision.flags), guard_repeat_suppressed_);
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
        cmdctrl_hz >= static_cast<int>(px4ctrl_params->statemachine_params.l2_cmd_ctrl_min_hz)) {
      next = MissionPhase::CMD_CTRL;
    }
    break;
  case MissionPhase::CMD_CTRL:
    if (!ctx.cmd_fresh ||
        cmdctrl_hz < static_cast<int>(px4ctrl_params->statemachine_params.l2_cmd_ctrl_min_hz)) {
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
      cmdctrl_hz >= static_cast<int>(px4ctrl_params->statemachine_params.l2_cmd_ctrl_min_hz)) {
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

  const double speed = px4ctrl_params->statemachine_params.l2_takeoff_landing_speed;
  const double des_z = landing_.start_pos.z() - speed * (elapsed_ms / 1000.0);

  const Eigen::Vector3d vel(ctx.odom_msg->twist.twist.linear.x,
                            ctx.odom_msg->twist.twist.linear.y,
                            ctx.odom_msg->twist.twist.linear.z);

  const double POSITION_DEVIATION_C =
      px4ctrl_params->statemachine_params.l2_land_position_deviation_c;
  const double VELOCITY_THR_C =
      px4ctrl_params->statemachine_params.l2_land_velocity_thr_c;
  const double TIME_KEEP_C =
      px4ctrl_params->statemachine_params.l2_land_time_keep_c;

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
    ctrl_cmd.type = params::ControlType::BODY_RATES;
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

    ctrl_cmd =
        se3_controller_->runControl(des, *ctx.odom_msg, *ctx.imu_msg);
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

    ctrl_cmd =
        se3_controller_->runControl(des, *ctx.odom_msg, *ctx.imu_msg);
    estimate_thrust_from_imu();
    return true;
  }

  case MissionPhase::LANDING:
  case MissionPhase::FAILSAFE: {
    if (phase == MissionPhase::FAILSAFE && active_guard_action_ == params::Guard::HOLD) {
      des.p = hover_.pos;
      des.q = hover_.q;
      des.yaw = controller::yawFromQuat(des.q);
      ctrl_cmd =
          se3_controller_->runControl(des, *ctx.odom_msg, *ctx.imu_msg);
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

    ctrl_cmd =
        se3_controller_->runControl(des, *ctx.odom_msg, *ctx.imu_msg);
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
    ctrl_cmd.type = params::ControlType::BODY_RATES;
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
    des.q =
        Eigen::Quaterniond(des_quat[0], des_quat[1], des_quat[2], des_quat[3]);
    des.yaw = ctx.cmd_msg->yaw;

    ctrl_cmd =
        se3_controller_->runControl(des, *ctx.odom_msg, *ctx.imu_msg);
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
  const double hover = std::clamp(se3_controller_->getHoverThrustEstimate(), min_thrust,
                                  max_thrust);

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

void Px4Ctrl::process() {
  const MissionContext ctx = build_context();
  GuardDecision decision;

  offboard_state_ = ctx.offboard;
  armed_state_ = ctx.armed;
  if (ctx.odom_msg != nullptr) {
    const Eigen::Quaterniond q(ctx.odom_msg->pose.pose.orientation.w,
                               ctx.odom_msg->pose.pose.orientation.x,
                               ctx.odom_msg->pose.pose.orientation.y,
                               ctx.odom_msg->pose.pose.orientation.z);
    const auto rpy_deg = quatToRpyDeg(q);
    roll_deg_ = rpy_deg[0];
    pitch_deg_ = rpy_deg[1];
    yaw_deg_ = rpy_deg[2];
  }

  const bool essential_ready =
      ctx.state_msg != nullptr && ctx.ext_state_msg != nullptr &&
      ctx.imu_msg != nullptr && ctx.battery_msg != nullptr;

  if (!essential_ready) {
    if (timePassedSeconds(last_log_state_time) > 2) {
      std::string missing;
      auto append_missing = [&](const char *name) {
        if (!missing.empty()) {
          missing += "/";
        }
        missing += name;
      };

      if (ctx.state_msg == nullptr) {
        append_missing("state");
      }
      if (ctx.ext_state_msg == nullptr) {
        append_missing("ext_state");
      }
      if (ctx.imu_msg == nullptr) {
        append_missing("imu");
      }
      if (ctx.battery_msg == nullptr) {
        append_missing("battery");
      }
      spdlog::info("Waiting for essential messages, missing: {}", missing);
      last_log_state_time = ctx.now;
    }

    controller::ControlCommand ctrl_cmd;
    build_proof_alive(ctx, ctrl_cmd);
    apply_control(ctrl_cmd);

    phase_ = MissionPhase::STANDBY;
    return;
  }

  guard(ctx, decision);
  MissionPhase next_phase = evaluate_phase(ctx, decision);

  if (next_phase != phase_) {
    last_phase_ = phase_;
    spdlog::info("Mission phase change:{}->{}", phaseName(phase_),
                 phaseName(next_phase));
    phase_ = next_phase;
    on_phase_enter(phase_, ctx, decision);
  }

  const bool allow_cmd_ctrl =
      (phase_ == MissionPhase::CMD_CTRL_READY || phase_ == MissionPhase::CMD_CTRL);
  if (allow_cmd_ctrl != allow_cmdctrl_pub_state_) {
    px4_bridge->pub_allow_cmdctrl(allow_cmd_ctrl);
    allow_cmdctrl_pub_state_ = allow_cmd_ctrl;
  }

  const auto source = select_control_source(ctx, phase_, decision);

  controller::ControlCommand ctrl_cmd;
  build_command(ctx, phase_, source, ctrl_cmd);
  apply_control(ctrl_cmd);

  if (timePassed(last_log_state_time) > 1000) {
    last_log_state_time = ctx.now;
    spdlog::debug("phase:{},source:{},offboard:{},armed:{}", phaseName(phase_),
                  sourceName(source), static_cast<int>(offboard_state_),
                  static_cast<int>(armed_state_));
  }
}

void Px4Ctrl::estimate_thrust_from_imu() {
  const auto imu = px4_state->imu->value();
  if (imu.first == nullptr) {
    return;
  }
  const Eigen::Vector3d est_a(imu.first->linear_acceleration.x,
                              imu.first->linear_acceleration.y,
                              imu.first->linear_acceleration.z);
  se3_controller_->estimateThrustModel(est_a, imu.second);
}

void Px4Ctrl::apply_control(const controller::ControlCommand &cmd) {
  double thrust =
      std::clamp(cmd.thrust, px4ctrl_params->quadrotor_params.min_thrust,
                 px4ctrl_params->quadrotor_params.max_thrust);

  switch (cmd.type) {
  case px4ctrl::params::ControlType::BODY_RATES: {
    Eigen::Vector3d bodyrates =
        cmd.bodyrates.cwiseMax(-px4ctrl_params->quadrotor_params.max_bodyrate)
            .cwiseMin(px4ctrl_params->quadrotor_params.max_bodyrate);

    last_ctrl_cmd = cmd;
    last_ctrl_cmd.type = params::ControlType::BODY_RATES;
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

void Px4Ctrl::client_command_callback(const ui::ClientPayload &payload) {
  const auto recv_time = clock::now();

  if (payload.command != ui::ClientCommand::HEARTBEAT) {
    spdlog::info("client command:{}",
                 ui::CommandStr[static_cast<int>(payload.command)]);
  }

  if (px4ctrl_params->guard_params.use_rc &&
      payload.command != ui::ClientCommand::HEARTBEAT &&
      payload.command != ui::ClientCommand::SET_SAFETY_LIMITS &&
      !has_valid_rc_signal()) {
    spdlog::warn("Reject client command: use_rc enabled but RC signal is invalid");
    last_client_cmd_time = recv_time;
    has_client_cmd = true;
    return;
  }

  switch (payload.command) {
  case ui::ClientCommand::HEARTBEAT:
    break;

  case ui::ClientCommand::ARM:
    if (px4ctrl_params->guard_params.use_rc) {
      spdlog::warn("Reject ARM from client: use_rc enabled, arm via RC/PX4 mode switch");
      break;
    }
    if (!px4_bridge->set_arm(true)) {
      spdlog::error("arm failed");
    }
    se3_controller_->resetThrustMapping();
    break;

  case ui::ClientCommand::FORCE_DISARM:
    if (px4ctrl_params->guard_params.use_rc) {
      spdlog::warn("Reject DISARM from client: use_rc enabled, disarm via RC/PX4 mode switch");
      break;
    }
    if (!px4_bridge->force_disarm()) {
      spdlog::error("force disarm failed");
    }
    se3_controller_->resetThrustMapping();
    requested_phase_ = MissionPhase::STANDBY;
    break;

  case ui::ClientCommand::ENTER_OFFBOARD:
    if (px4ctrl_params->guard_params.use_rc) {
      spdlog::warn("Reject ENTER_OFFBOARD from client: use_rc enabled, switch via RC/PX4");
      break;
    }
    if (!px4_bridge->enter_offboard()) {
      spdlog::error("enter offboard failed");
    }
    break;

  case ui::ClientCommand::EXIT_OFFBOARD:
    if (px4ctrl_params->guard_params.use_rc) {
      spdlog::warn("Reject EXIT_OFFBOARD from client: use_rc enabled, switch via RC/PX4");
      break;
    }
    if (!px4_bridge->exit_offboard()) {
      spdlog::error("exit offboard failed");
    }
    break;

  case ui::ClientCommand::TAKEOFF:
    if (!offboard_state_ || !armed_state_) {
      spdlog::error("Reject TAKEOFF: require OFFBOARD + ARMED");
      break;
    }
    if (phase_ == MissionPhase::STANDBY) {
      requested_phase_ = MissionPhase::TAKEOFF;
    } else {
      spdlog::error("Reject TAKEOFF: invalid phase {}", phaseName(phase_));
    }
    break;

  case ui::ClientCommand::LAND:
    if (!offboard_state_ || !armed_state_) {
      spdlog::error("Reject LAND: require OFFBOARD + ARMED");
      break;
    }
    if (phase_ == MissionPhase::HOVER || phase_ == MissionPhase::CMD_CTRL_READY ||
        phase_ == MissionPhase::CMD_CTRL) {
      requested_phase_ = MissionPhase::LANDING;
    } else {
      spdlog::error("Reject LAND: invalid phase {}", phaseName(phase_));
    }
    break;

  case ui::ClientCommand::FORCE_HOVER:
    if (!offboard_state_ || !armed_state_) {
      spdlog::error("Reject FORCE_HOVER: require OFFBOARD + ARMED");
      break;
    }
    if (phase_ != MissionPhase::STANDBY && phase_ != MissionPhase::WAIT_FOR_RC) {
      requested_phase_ = MissionPhase::HOVER;
    } else {
      spdlog::error("Reject FORCE_HOVER: invalid phase {}", phaseName(phase_));
    }
    break;

  case ui::ClientCommand::ALLOW_CMD_CTRL:
    if (!offboard_state_ || !armed_state_) {
      spdlog::error("Reject ALLOW_CMD_CTRL: require OFFBOARD + ARMED");
      break;
    }
    if (phase_ == MissionPhase::HOVER) {
      requested_phase_ = MissionPhase::CMD_CTRL_READY;
    } else {
      spdlog::error("Reject ALLOW_CMD_CTRL: invalid phase {}", phaseName(phase_));
    }
    break;

  case ui::ClientCommand::CHANGE_HOVER_POS: {
    if (!offboard_state_ || !armed_state_) {
      spdlog::error("Reject CHANGE_HOVER_POS: require OFFBOARD + ARMED");
      break;
    }
    if (phase_ != MissionPhase::HOVER) {
      spdlog::error("Reject CHANGE_HOVER_POS: require HOVER phase");
      break;
    }

    double data[7];
    std::memcpy(data, payload.data, sizeof(data));
    bool reject = false;
    for (auto &d : data) {
      if (std::isnan(d)) {
        reject = true;
        spdlog::error("Reject CHANGE_HOVER_POS: data contains nan");
        break;
      }
    }
    if (reject) {
      break;
    }
    Eigen::Vector3d pos(data[0], data[1], data[2]);
    Eigen::Quaterniond q(data[3], data[4], data[5], data[6]);
    set_hovering_pos(pos, q);
    break;
  }

  case ui::ClientCommand::SET_SAFETY_LIMITS: {
    ui::SafetyLimitsPayload limits{};
    unpack_raw(payload.data, sizeof(limits), limits);
    update_safety_limits(limits);
    break;
  }
  }

  last_client_cmd_time = recv_time;
  has_client_cmd = true;
}

ui::ServerPayload Px4Ctrl::fill_server_payload() {
  ui::ServerPayload payload{};
  payload.id = 0;
  const auto now = clock::now();
  payload.timestamp = to_uint64(now);

  auto battery = px4_state->battery->value().first;
  if (battery == nullptr) {
    payload.battery_voltage = 0;
    payload.battery_remaining = -1;
  } else {
    payload.battery_voltage = battery->voltage;
    payload.battery_remaining = battery->percentage;
  }

  payload.mission_phase = static_cast<int32_t>(phase_);
  payload.offboard_state = offboard_state_ ? 1 : 0;
  payload.armed_state = armed_state_ ? 1 : 0;

  payload.hover_pos[0] = hover_.pos.x();
  payload.hover_pos[1] = hover_.pos.y();
  payload.hover_pos[2] = hover_.pos.z();
  payload.hover_quat[0] = hover_.q.w();
  payload.hover_quat[1] = hover_.q.x();
  payload.hover_quat[2] = hover_.q.y();
  payload.hover_quat[3] = hover_.q.z();

  const auto odom_state = px4_state->odom->value();
  auto odom = odom_state.first;
  if (odom == nullptr) {
    payload.pos[0] = 0;
    payload.pos[1] = 0;
    payload.pos[2] = 0;
    payload.vel[0] = 0;
    payload.vel[1] = 0;
    payload.vel[2] = 0;
    payload.omega[0] = 0;
    payload.omega[1] = 0;
    payload.omega[2] = 0;
    payload.quat[0] = 1;
    payload.quat[1] = 0;
    payload.quat[2] = 0;
    payload.quat[3] = 0;
  } else {
    payload.pos[0] = odom->pose.pose.position.x;
    payload.pos[1] = odom->pose.pose.position.y;
    payload.pos[2] = odom->pose.pose.position.z;
    payload.vel[0] = odom->twist.twist.linear.x;
    payload.vel[1] = odom->twist.twist.linear.y;
    payload.vel[2] = odom->twist.twist.linear.z;
    payload.omega[0] = odom->twist.twist.angular.x;
    payload.omega[1] = odom->twist.twist.angular.y;
    payload.omega[2] = odom->twist.twist.angular.z;
    payload.quat[0] = odom->pose.pose.orientation.w;
    payload.quat[1] = odom->pose.pose.orientation.x;
    payload.quat[2] = odom->pose.pose.orientation.y;
    payload.quat[3] = odom->pose.pose.orientation.z;
  }

  payload.odom_hz = odom_hz;
  payload.cmdctrl_hz = cmdctrl_hz;

  payload.thrust_setpoint = static_cast<float>(last_ctrl_cmd.thrust);
  if (last_ctrl_cmd.type == params::ControlType::BODY_RATES) {
    payload.omega_setpoint[0] = static_cast<float>(last_ctrl_cmd.bodyrates.x());
    payload.omega_setpoint[1] = static_cast<float>(last_ctrl_cmd.bodyrates.y());
    payload.omega_setpoint[2] = static_cast<float>(last_ctrl_cmd.bodyrates.z());
  } else {
    payload.omega_setpoint[0] = 0;
    payload.omega_setpoint[1] = 0;
    payload.omega_setpoint[2] = 0;
  }

  payload.thrust_map[0] = static_cast<float>(se3_controller_->getThr2AccEstimate());
  payload.thrust_map[1] =
      static_cast<float>(se3_controller_->getHoverThrustEstimate());
  payload.thrust_map[2] = static_cast<float>(last_ctrl_cmd.thrust);

  payload.telemetry_seq = telemetry_seq++;
  payload.guard_flags = guard_flags;
  payload.odom_age_ms =
      (odom != nullptr)
          ? static_cast<float>(std::max(0.0, timeDuration(odom_state.second, now)))
          : -1.0F;
  payload.client_cmd_age_ms =
      has_client_cmd
          ? static_cast<float>(
                std::max(0.0, timeDuration(last_client_cmd_time, now)))
          : -1.0F;
  payload.speed_norm = std::sqrt(payload.vel[0] * payload.vel[0] +
                                 payload.vel[1] * payload.vel[1] +
                                 payload.vel[2] * payload.vel[2]);

  const Eigen::Quaterniond q(payload.quat[0], payload.quat[1], payload.quat[2],
                             payload.quat[3]);
  const Eigen::Vector3d body_z = q * Eigen::Vector3d::UnitZ();
  const double cos_tilt =
      std::clamp(body_z.dot(Eigen::Vector3d::UnitZ()), -1.0, 1.0);
  payload.tilt_deg = static_cast<float>(std::acos(cos_tilt) * kRadToDeg);

  payload.roll_deg = static_cast<float>(roll_deg_);
  payload.pitch_deg = static_cast<float>(pitch_deg_);
  payload.yaw_deg = static_cast<float>(yaw_deg_);

  payload.geofence_min[0] = static_cast<float>(px4ctrl_params->guard_params.geofence_min[0]);
  payload.geofence_min[1] = static_cast<float>(px4ctrl_params->guard_params.geofence_min[1]);
  payload.geofence_min[2] = static_cast<float>(px4ctrl_params->guard_params.geofence_min[2]);
  payload.geofence_max[0] = static_cast<float>(px4ctrl_params->guard_params.geofence_max[0]);
  payload.geofence_max[1] = static_cast<float>(px4ctrl_params->guard_params.geofence_max[1]);
  payload.geofence_max[2] = static_cast<float>(px4ctrl_params->guard_params.geofence_max[2]);
  payload.max_roll_deg = static_cast<float>(px4ctrl_params->guard_params.max_roll_deg);
  payload.max_pitch_deg = static_cast<float>(px4ctrl_params->guard_params.max_pitch_deg);
  payload.max_yaw_deg = static_cast<float>(px4ctrl_params->guard_params.max_yaw_deg);
  payload.enable_geofence =
      static_cast<uint8_t>(px4ctrl_params->guard_params.enable_geofence ? 1 : 0);
  payload.enable_attitude_fence =
      static_cast<uint8_t>(px4ctrl_params->guard_params.enable_attitude_fence ? 1 : 0);
  payload.use_rc = static_cast<uint8_t>(px4ctrl_params->guard_params.use_rc ? 1 : 0);

  return payload;
}

std::pair<const Eigen::Vector3d, const Eigen::Quaterniond>
Px4Ctrl::get_hovering_pos() const {
  return {hover_.pos, hover_.q};
}

bool Px4Ctrl::set_hovering_pos(const Eigen::Vector3d &pos,
                               const Eigen::Quaterniond &q) {
  hover_.pos = pos;
  hover_.q = q;
  hover_.initialized = true;
  return true;
}

} // namespace px4ctrl
