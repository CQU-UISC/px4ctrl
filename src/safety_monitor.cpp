#include "safety_monitor.h"

#include <cmath>

#include <spdlog/spdlog.h>

namespace px4ctrl {

SafetyMonitor::SafetyMonitor(std::shared_ptr<Context> ctx)
    : ctx_(std::move(ctx)) {
  last_guard_repeat_log_time_ = clock::now();
  last_guard_log_time = clock::now();
}

SafetyMonitor::Result SafetyMonitor::evaluate(const MissionContextSnapshot &snap) {
  Result result;
  params::Guard selected = params::Guard::HOLD;
  std::string selected_reason;
  uint32_t selected_flag = 0;

  auto select_guard = [&](params::Guard action, const std::string &reason,
                          uint32_t flag) {
    if (guardSeverity(action) >= guardSeverity(selected)) {
      selected = action;
      selected_reason = reason;
    }
    selected_flag |= flag;
  };

  // 1. RC check
  if (ctx_->params()->guard_params.use_rc && !snap.rc_valid) {
    if (snap.armed || ctx_->armed_state) {
      select_guard(ctx_->params()->guard_params.rc_triggered, "rc signal lost",
                   kGuardRcLost);
    } else {
      result.rc_required_block = true;
      selected_flag |= kGuardRcRequired;
    }
  }

  // 2. MAVROS timeout
  if (snap.state_msg == nullptr ||
      snap.state_age_ms > ctx_->params()->guard_params.mavros_timeout) {
    select_guard(ctx_->params()->guard_params.mavros_triggered, "mavros timeout",
                 kGuardMavrosTimeout);
  }

  // 3. Odom checks
  if (snap.odom_msg == nullptr ||
      snap.odom_age_ms > ctx_->params()->guard_params.odom_timeout) {
    select_guard(ctx_->params()->guard_params.localization_loss_triggered,
                 "odom timeout", kGuardOdomTimeout);
  } else if (ctx_->odom_hz < static_cast<int>(ctx_->params()->guard_params.odom_min_hz)) {
    select_guard(ctx_->params()->guard_params.odom_triggered, "odom low hz",
                 kGuardOdomLowHz);
  }

  // 4. UI timeout
  const bool in_flight =
      (snap.offboard || ctx_->offboard_state) && (snap.armed || ctx_->armed_state);
  if (in_flight && ctx_->has_client_cmd &&
      timeDuration(ctx_->last_client_cmd_time, snap.now) >
          ctx_->params()->guard_params.ui_timeout) {
    select_guard(ctx_->params()->guard_params.ui_triggered, "ui timeout",
                 kGuardUiTimeout);
  }

  // 5. Low battery
  if (snap.battery_msg != nullptr &&
      snap.battery_msg->voltage < ctx_->params()->guard_params.low_battery_voltage) {
    select_guard(ctx_->params()->guard_params.lowvolt_triggered, "low battery",
                 kGuardLowBattery);
  }

  // 6-8. Position-dependent fences
  if (snap.odom_msg != nullptr) {
    const auto &pose = snap.odom_msg->pose.pose;
    const Eigen::Vector3d pos(pose.position.x, pose.position.y, pose.position.z);
    const Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x,
                               pose.orientation.y, pose.orientation.z);
    const Eigen::Vector3d vel(snap.odom_msg->twist.twist.linear.x,
                              snap.odom_msg->twist.twist.linear.y,
                              snap.odom_msg->twist.twist.linear.z);

    // 6. Geofence
    if (ctx_->params()->guard_params.enable_geofence) {
      const auto &mn = ctx_->params()->guard_params.geofence_min;
      const auto &mx = ctx_->params()->guard_params.geofence_max;
      if (pos.x() < mn[0] || pos.y() < mn[1] || pos.z() < mn[2] ||
          pos.x() > mx[0] || pos.y() > mx[1] || pos.z() > mx[2]) {
        select_guard(ctx_->params()->guard_params.geofence_triggered,
                     "geofence violated", kGuardGeofence);
      }
    }

    // 7. Attitude fence
    if (ctx_->params()->guard_params.enable_attitude_fence) {
      // inline quatToRpyDeg
      const Eigen::Quaterniond qn = q.normalized();
      const double sinr_cosp = 2.0 * (qn.w() * qn.x() + qn.y() * qn.z());
      const double cosr_cosp = 1.0 - 2.0 * (qn.x() * qn.x() + qn.y() * qn.y());
      const double roll = std::atan2(sinr_cosp, cosr_cosp);
      const double sinp = 2.0 * (qn.w() * qn.y() - qn.z() * qn.x());
      constexpr double kHalfPi = 1.5707963267948966;
      const double pitch =
          (std::abs(sinp) >= 1.0) ? std::copysign(kHalfPi, sinp) : std::asin(sinp);
      const double siny_cosp = 2.0 * (qn.w() * qn.z() + qn.x() * qn.y());
      const double cosy_cosp = 1.0 - 2.0 * (qn.y() * qn.y() + qn.z() * qn.z());
      const double yaw = std::atan2(siny_cosp, cosy_cosp);
      constexpr double kRadToDeg = 57.29577951308232;

      auto norm_deg = [](double rad) {
        return std::atan2(std::sin(rad), std::cos(rad)) * kRadToDeg;
      };
      const double roll_deg = std::abs(norm_deg(roll));
      const double pitch_deg = std::abs(norm_deg(pitch));
      const double yaw_deg = std::abs(norm_deg(yaw));

      auto over_limit = [](double angle_deg, double limit_deg) {
        return limit_deg != -1.0 && angle_deg > limit_deg;
      };

      if (over_limit(roll_deg, ctx_->params()->guard_params.max_roll_deg) ||
          over_limit(pitch_deg, ctx_->params()->guard_params.max_pitch_deg) ||
          over_limit(yaw_deg, ctx_->params()->guard_params.max_yaw_deg)) {
        select_guard(ctx_->params()->guard_params.attitude_triggered,
                     "attitude fence violated", kGuardAttitudeFence);
      }
    }

    // 8. Velocity fence
    if (ctx_->params()->guard_params.enable_velocity_fence &&
        vel.norm() > ctx_->params()->guard_params.max_velocity_norm) {
      select_guard(ctx_->params()->guard_params.velocity_triggered,
                   "velocity fence violated", kGuardVelocityFence);
    }
  }

  result.flags = selected_flag;
  result.action = selected;
  result.reason = selected_reason;
  result.triggered = !selected_reason.empty();

  ctx_->guard_flags = result.flags;

  // Logging
  if (result.triggered) {
    const bool same_guard =
        has_last_guard_ && result.action == last_guard_action_ &&
        result.reason == last_guard_reason_ &&
        result.flags == last_guard_flags_;

    if (!same_guard) {
      spdlog::warn("Guard triggered action:{} reason:{} guards:{}",
                   guardActionName(result.action), result.reason,
                   guardFlagsToString(result.flags));
      has_last_guard_ = true;
      last_guard_action_ = result.action;
      last_guard_reason_ = result.reason;
      last_guard_flags_ = result.flags;
      guard_repeat_suppressed_ = 0;
      last_guard_repeat_log_time_ = snap.now;
    } else if (timeDuration(last_guard_repeat_log_time_, snap.now) > 2000) {
      spdlog::warn(
          "Guard still active action:{} reason:{} guards:{} (suppressed {} repeats)",
          guardActionName(result.action), result.reason,
          guardFlagsToString(result.flags), guard_repeat_suppressed_);
      guard_repeat_suppressed_ = 0;
      last_guard_repeat_log_time_ = snap.now;
    } else {
      ++guard_repeat_suppressed_;
    }
  } else {
    has_last_guard_ = false;
    guard_repeat_suppressed_ = 0;
  }

  if (result.rc_required_block && timePassed(last_guard_log_time) > 1000) {
    spdlog::warn("use_rc enabled: waiting for valid RC signal before start");
    last_guard_log_time = snap.now;
  }

  was_triggered_ = result.triggered;
  return result;
}

} // namespace px4ctrl
