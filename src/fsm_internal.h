#pragma once

#include <array>
#include <cmath>

#include <Eigen/Dense>

#include "context.h"

namespace px4ctrl::fsm_internal {

constexpr double kRadToDeg = 57.29577951308232;

inline double normalize_deg(double rad) {
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

inline const char *phaseName(MissionPhase phase) {
  switch (phase) {
  case MissionPhase::STANDBY:  return "STANDBY";
  case MissionPhase::TAKEOFF:  return "TAKEOFF";
  case MissionPhase::HOVER:    return "HOVER";
  case MissionPhase::CMD_CTRL: return "CMD_CTRL";
  case MissionPhase::LANDING:  return "LANDING";
  case MissionPhase::FAILSAFE: return "FAILSAFE";
  }
  return "UNKNOWN";
}

inline const char *sourceName(ControlSource source) {
  switch (source) {
  case ControlSource::PROOF_ALIVE:  return "PROOF_ALIVE";
  case ControlSource::SE3:          return "SE3";
  case ControlSource::SAFE_LANDING: return "SAFE_LANDING";
  case ControlSource::EXTERNAL_CMD: return "EXTERNAL_CMD";
  }
  return "UNKNOWN";
}

} // namespace px4ctrl::fsm_internal
