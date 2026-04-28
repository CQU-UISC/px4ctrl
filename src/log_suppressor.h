#pragma once

#include <cstdint>

#include "types.h"

namespace px4ctrl {

/// Rate-limits repeated log messages: logs on first trigger, then at most every
/// `cooldown_ms`, counting suppressed repeats in between.
class LogSuppressor {
public:
  explicit LogSuppressor(int64_t cooldown_ms = 2000)
      : cooldown_ms_(cooldown_ms) {}

  struct TriggerResult {
    bool should_log;
    bool is_first;
    uint32_t suppressed;
  };

  /// Call when the monitored condition is active.
  TriggerResult trigger(clock::time_point now) {
    if (!active_) {
      active_ = true;
      last_log_time_ = now;
      return {true, true, 0};
    }
    if (timeDuration(last_log_time_, now) > cooldown_ms_) {
      auto s = suppressed_;
      suppressed_ = 0;
      last_log_time_ = now;
      return {true, false, s};
    }
    ++suppressed_;
    return {false, false, 0};
  }

  struct ClearResult {
    bool was_active;
    uint32_t suppressed;
  };

  /// Call when the condition clears. Returns previous state.
  ClearResult clear() {
    ClearResult r{active_, suppressed_};
    active_ = false;
    suppressed_ = 0;
    return r;
  }

private:
  int64_t cooldown_ms_;
  bool active_ = false;
  uint32_t suppressed_ = 0;
  clock::time_point last_log_time_;
};

} // namespace px4ctrl
