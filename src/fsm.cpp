#include "fsm.h"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <memory>
#include <thread>

#include <mavros_msgs/msg/state.hpp>

#include "fsm_internal.h"

namespace px4ctrl {

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
  int delta_t = static_cast<int>(
      1000 / std::max<uint32_t>(1, px4ctrl_params->statemachine_params.freq));

  odom_hold = px4_state->odom->observe([&](auto &) { odom_count++; });
  ctrl_hold = px4_state->ctrl_command->observe([&](auto &) { cmdctrl_count++; });
  client_hold =
      px4_server->client_data.observe([&](auto &cmd) { client_command_callback(cmd); });

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

  ctx.odom_fresh =
      ctx.odom_msg != nullptr &&
      ctx.odom_age_ms <= px4ctrl_params->guard_params.odom_timeout;

  const double cmd_fresh_ms =
      std::max(100.0, 2000.0 / std::max(1.0, px4ctrl_params->statemachine_params
                                                  .l2_cmd_ctrl_min_hz));
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
    const auto rpy_deg = fsm_internal::quatToRpyDeg(q);
    roll_deg_ = rpy_deg[0];
    pitch_deg_ = rpy_deg[1];
    yaw_deg_ = rpy_deg[2];
  }

  const bool essential_ready = ctx.state_msg != nullptr &&
                               ctx.ext_state_msg != nullptr &&
                               ctx.imu_msg != nullptr &&
                               ctx.battery_msg != nullptr;

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
    spdlog::info("Mission phase change:{}->{}", fsm_internal::phaseName(phase_),
                 fsm_internal::phaseName(next_phase));
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
    spdlog::debug("phase:{},source:{},offboard:{},armed:{}",
                  fsm_internal::phaseName(phase_), fsm_internal::sourceName(source),
                  static_cast<int>(offboard_state_),
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

} // namespace px4ctrl
