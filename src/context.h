#pragma once

#include <memory>
#include <mutex>
#include <optional>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "bridge.h"
#include "controller.h"
#include "params.h"
#include "types.h"

namespace px4ctrl {

enum class MissionPhase {
  STANDBY,
  TAKEOFF,
  HOVER,
  CMD_CTRL,
  LANDING,
  FAILSAFE,
};

enum class ControlSource {
  PROOF_ALIVE,
  SE3,
  SAFE_LANDING,
  EXTERNAL_CMD,
};

struct TakeoffProfile {
  Eigen::Vector3d start_pos = Eigen::Vector3d::Zero();
  Eigen::Quaterniond start_q = Eigen::Quaterniond::Identity();
  clock::time_point start_time;
  bool initialized = false;
};

struct HoverTarget {
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  bool initialized = false;
};

struct LandingProfile {
  Eigen::Vector3d start_pos = Eigen::Vector3d::Zero();
  Eigen::Quaterniond start_q = Eigen::Quaterniond::Identity();
  clock::time_point start_time;
  clock::time_point c12_reached_time;
  bool initialized = false;
  bool c12_started = false;
};

struct MissionContextSnapshot {
  clock::time_point now;

  mavros_msgs::msg::State::ConstSharedPtr state_msg;
  mavros_msgs::msg::ExtendedState::ConstSharedPtr ext_state_msg;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_msg;
  sensor_msgs::msg::Imu::ConstSharedPtr imu_msg;
  sensor_msgs::msg::BatteryState::ConstSharedPtr battery_msg;
  px4ctrl_msgs::msg::Command::ConstSharedPtr cmd_msg;

  double state_age_ms = 0.0;
  double odom_age_ms = 0.0;
  double battery_age_ms = 0.0;
  double imu_age_ms = 0.0;
  double cmd_age_ms = 0.0;

  bool offboard = false;
  bool armed = false;
  bool odom_fresh = false;
  bool cmd_fresh = false;
  bool rc_valid = false;
};

class Context {
public:
  Context(std::shared_ptr<Px4State> px4_state,
          std::shared_ptr<Px4CtrlParams> params)
      : px4_state_(std::move(px4_state)), params_(std::move(params)) {}

  const std::shared_ptr<Px4State> &px4_state() const { return px4_state_; }
  const std::shared_ptr<Px4CtrlParams> &params() const { return params_; }

  // --- Mission phase ---
  MissionPhase phase = MissionPhase::STANDBY;
  MissionPhase last_phase = MissionPhase::STANDBY;
  clock::time_point phase_enter_time;

  bool offboard_state = false;
  bool armed_state = false;

  // --- Flight profiles ---
  TakeoffProfile takeoff;
  HoverTarget hover;
  LandingProfile landing;

  // --- Guard state ---
  params::Guard active_guard_action = params::Guard::HOLD;
  uint32_t guard_flags = 0;

  // --- Client communication ---
  bool has_client_cmd = false;
  clock::time_point last_client_cmd_time;
  uint32_t telemetry_seq = 0;

  // --- Control ---
  bool allow_cmdctrl_pub_state = false;
  controller::ControlCommand last_ctrl_cmd{};

  // --- Attitude cache ---
  double roll_deg = 0.0;
  double pitch_deg = 0.0;
  double yaw_deg = 0.0;

  // --- Hz tracking ---
  int odom_count = 0;
  int cmdctrl_count = 0;
  int odom_hz = 0;
  int cmdctrl_hz = 0;
  clock::time_point odom_last_time;
  clock::time_point cmdctrl_last_time;

  // --- Thread-safe phase requests ---
  void request_phase(MissionPhase p) {
    std::lock_guard<std::mutex> lock(request_mutex_);
    requested_phase_ = p;
  }

  std::optional<MissionPhase> pop_request() {
    std::lock_guard<std::mutex> lock(request_mutex_);
    auto r = requested_phase_;
    requested_phase_.reset();
    return r;
  }

  // --- Snapshot builder ---
  MissionContextSnapshot build_snapshot() const {
    MissionContextSnapshot ctx;
    ctx.now = clock::now();

    const auto state = px4_state_->state->value();
    const auto ext_state = px4_state_->ext_state->value();
    const auto odom = px4_state_->odom->value();
    const auto imu = px4_state_->imu->value();
    const auto battery = px4_state_->battery->value();
    const auto cmd = px4_state_->ctrl_command->value();

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
        ctx.odom_age_ms <= params_->guard_params.odom_timeout;

    const double cmd_fresh_ms =
        std::max(100.0, 2000.0 / std::max(1.0, params_->statemachine_params
                                                    .l2_cmd_ctrl_min_hz));
    ctx.cmd_fresh = ctx.cmd_msg != nullptr && ctx.cmd_age_ms <= cmd_fresh_ms;

    ctx.rc_valid = has_valid_rc_signal();
    return ctx;
  }

  bool has_valid_rc_signal() const {
    const auto rcin = px4_state_->rcin->value();
    if (rcin.first == nullptr) return false;
    if (timeDuration(rcin.second, clock::now()) > params_->guard_params.rc_timeout)
      return false;
    if (rcin.first->channels.empty()) return false;
    return true;
  }

private:
  std::shared_ptr<Px4State> px4_state_;
  std::shared_ptr<Px4CtrlParams> params_;
  std::mutex request_mutex_;
  std::optional<MissionPhase> requested_phase_;
};

} // namespace px4ctrl
