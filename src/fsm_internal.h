#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <string>

#include "fsm.h"

namespace px4ctrl::fsm_internal {

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

inline int guardSeverity(const params::Guard action) {
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

inline double normalize_deg(const double rad) {
  return std::atan2(std::sin(rad), std::cos(rad)) * kRadToDeg;
}

inline std::array<double, 3> quatToRpyDeg(const Eigen::Quaterniond &q_in) {
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

inline const char *phaseName(const Px4Ctrl::MissionPhase phase) {
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

inline const char *sourceName(const Px4Ctrl::ControlSource source) {
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

inline const char *guardActionName(const params::Guard action) {
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

inline std::string guardFlagsToString(const uint32_t flags) {
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

} // namespace px4ctrl::fsm_internal
