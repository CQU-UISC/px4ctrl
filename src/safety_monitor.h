#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "context.h"
#include "params.h"
#include "types.h"

namespace px4ctrl {

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

inline int guardSeverity(params::Guard action) {
  switch (action) {
  case params::Guard::HOLD:   return 1;
  case params::Guard::LAND:   return 2;
  case params::Guard::DISARM: return 3;
  }
  return 0;
}

inline const char *guardActionName(params::Guard action) {
  switch (action) {
  case params::Guard::HOLD:   return "HOLD";
  case params::Guard::LAND:   return "LAND";
  case params::Guard::DISARM: return "DISARM";
  }
  return "UNKNOWN";
}

inline std::string guardFlagsToString(uint32_t flags) {
  if (flags == 0) return "none";
  std::string out;
  auto append = [&](const char *name) {
    if (!out.empty()) out += "|";
    out += name;
  };
  if (flags & kGuardMavrosTimeout)   append("mavros_timeout");
  if (flags & kGuardOdomTimeout)     append("odom_timeout");
  if (flags & kGuardUiTimeout)       append("ui_timeout");
  if (flags & kGuardLowBattery)      append("low_battery");
  if (flags & kGuardGeofence)        append("geofence");
  if (flags & kGuardAttitudeFence)   append("attitude_fence");
  if (flags & kGuardVelocityFence)   append("velocity_fence");
  if (flags & kGuardOdomLowHz)       append("odom_low_hz");
  if (flags & kGuardRcLost)          append("rc_lost");
  if (flags & kGuardRcRequired)      append("rc_required");
  return out;
}

class SafetyMonitor {
public:
  struct Result {
    bool triggered = false;
    bool rc_required_block = false;
    params::Guard action = params::Guard::HOLD;
    uint32_t flags = 0;
    std::string reason;
  };

  explicit SafetyMonitor(std::shared_ptr<Context> ctx);

  Result evaluate(const MissionContextSnapshot &snap);

  bool was_triggered() const { return was_triggered_; }

private:
  std::shared_ptr<Context> ctx_;

  // log suppression
  bool was_triggered_ = false;
  bool has_last_guard_ = false;
  params::Guard last_guard_action_ = params::Guard::HOLD;
  std::string last_guard_reason_;
  uint32_t last_guard_flags_ = 0;
  uint32_t guard_repeat_suppressed_ = 0;
  clock::time_point last_guard_repeat_log_time_;
  clock::time_point last_guard_log_time;
};

} // namespace px4ctrl
