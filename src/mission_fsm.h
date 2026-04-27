#pragma once

#include <memory>

#include <spdlog/spdlog.h>

#include "context.h"
#include "controller.h"
#include "params.h"
#include "safety_monitor.h"
#include "types.h"
#include "tinyfsm.hpp"

namespace px4ctrl {

// --- Events ---
struct EventArm : tinyfsm::Event {};
struct EventDisarm : tinyfsm::Event {};
struct EventTakeoff : tinyfsm::Event {};
struct EventLand : tinyfsm::Event {};
struct EventForceHover : tinyfsm::Event {};
struct EventAllowCmdCtrl : tinyfsm::Event {};
struct EventFailsafe : tinyfsm::Event {
  SafetyMonitor::Result result;
};
struct EventFailsafeCleared : tinyfsm::Event {};
struct EventTick : tinyfsm::Event {};
struct EventOffboardLost : tinyfsm::Event {};

// Forward declarations
struct Standby;
struct Takeoff;
struct Hover_State;
struct CmdCtrl;
struct Landing;
struct Failsafe;

// --- FSM host ---
struct MissionFSM : tinyfsm::Fsm<MissionFSM> {
  static std::shared_ptr<Context> ctx;

  static void set_context(std::shared_ptr<Context> c) { ctx = std::move(c); }

  virtual void entry() {}
  virtual void exit() {}

  virtual void react(const EventTakeoff &) {}
  virtual void react(const EventLand &) {}
  virtual void react(const EventForceHover &) {}
  virtual void react(const EventAllowCmdCtrl &) {}
  virtual void react(const EventFailsafe &);
  virtual void react(const EventFailsafeCleared &) {}
  virtual void react(const EventTick &) {}
  virtual void react(const EventOffboardLost &);

  // Helper: set hover desired state (used by multiple states)
  static void ensure_hover_initialized(const MissionContextSnapshot &snap);
};

// --- Inline helpers ---
inline void MissionFSM::react(const EventFailsafe &) { transit<Failsafe>(); }
inline void MissionFSM::react(const EventOffboardLost &) { transit<Standby>(); }

inline void MissionFSM::ensure_hover_initialized(const MissionContextSnapshot &snap) {
  auto &c = *ctx;
  if (!c.hover.initialized && snap.odom_msg != nullptr) {
    c.hover.pos = Eigen::Vector3d(snap.odom_msg->pose.pose.position.x,
                                   snap.odom_msg->pose.pose.position.y,
                                   snap.odom_msg->pose.pose.position.z);
    const Eigen::Quaterniond q(snap.odom_msg->pose.pose.orientation.w,
                                snap.odom_msg->pose.pose.orientation.x,
                                snap.odom_msg->pose.pose.orientation.y,
                                snap.odom_msg->pose.pose.orientation.z);
    const auto yaw = controller::yawFromQuat(q);
    c.hover.q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    c.hover.initialized = true;
  }
}

// --- State: Standby ---
struct Standby : MissionFSM {
  void entry() override {
    ctx->takeoff.initialized = false;
    ctx->landing.initialized = false;
    ctx->phase = MissionPhase::STANDBY;
    spdlog::info("Enter Standby");
  }

  void react(const EventTakeoff &) override {
    if (ctx->offboard_state && ctx->armed_state) {
      transit<Takeoff>();
    }
  }
};

// --- State: Takeoff ---
struct Takeoff : MissionFSM {
  void entry() override {
    const auto snap = ctx->build_snapshot();

    if (snap.odom_msg != nullptr) {
      ctx->takeoff.start_pos =
          Eigen::Vector3d(snap.odom_msg->pose.pose.position.x,
                          snap.odom_msg->pose.pose.position.y,
                          snap.odom_msg->pose.pose.position.z);
      ctx->takeoff.start_q =
          Eigen::Quaterniond(snap.odom_msg->pose.pose.orientation.w,
                             snap.odom_msg->pose.pose.orientation.x,
                             snap.odom_msg->pose.pose.orientation.y,
                             snap.odom_msg->pose.pose.orientation.z);
    } else {
      ctx->takeoff.start_pos = ctx->hover.pos;
      if (snap.imu_msg != nullptr) {
        const auto &q = snap.imu_msg->orientation;
        ctx->takeoff.start_q = Eigen::Quaterniond(q.w, q.x, q.y, q.z);
      } else {
        ctx->takeoff.start_q = Eigen::Quaterniond::Identity();
      }
    }
    ctx->takeoff.start_time = snap.now;
    ctx->takeoff.initialized = true;
    ctx->phase = MissionPhase::TAKEOFF;

    spdlog::info("Taking off from:{} {} {}", ctx->takeoff.start_pos.x(),
                 ctx->takeoff.start_pos.y(), ctx->takeoff.start_pos.z());
  }

  void react(const EventForceHover &) override { transit<Hover_State>(); }

  void react(const EventTick &) override {
    const auto snap = ctx->build_snapshot();
    if (!ctx->takeoff.initialized || snap.odom_msg == nullptr) return;
    const Eigen::Vector3d cur_pos(snap.odom_msg->pose.pose.position.x,
                                   snap.odom_msg->pose.pose.position.y,
                                   snap.odom_msg->pose.pose.position.z);
    Eigen::Vector3d des_pos = ctx->takeoff.start_pos;
    des_pos.z() += ctx->params()->statemachine_params.l2_takeoff_height;
    if ((cur_pos - des_pos).norm() < 0.1) {
      transit<Hover_State>();
    }
  }
};

// --- State: Hover ---
struct Hover_State : MissionFSM {
  void entry() override {
    ctx->landing.initialized = false;
    ctx->landing.c12_started = false;

    const auto snap = ctx->build_snapshot();
    if (ctx->last_phase == MissionPhase::TAKEOFF && ctx->takeoff.initialized) {
      ctx->hover.pos = ctx->takeoff.start_pos;
      ctx->hover.pos.z() += ctx->params()->statemachine_params.l2_takeoff_height;
      ctx->hover.q = ctx->takeoff.start_q;
    } else if (snap.odom_msg != nullptr) {
      ctx->hover.pos = Eigen::Vector3d(snap.odom_msg->pose.pose.position.x,
                                       snap.odom_msg->pose.pose.position.y,
                                       snap.odom_msg->pose.pose.position.z);
      const Eigen::Quaterniond q(snap.odom_msg->pose.pose.orientation.w,
                                  snap.odom_msg->pose.pose.orientation.x,
                                  snap.odom_msg->pose.pose.orientation.y,
                                  snap.odom_msg->pose.pose.orientation.z);
      const auto yaw = controller::yawFromQuat(q);
      ctx->hover.q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    }
    ctx->hover.initialized = true;
    ctx->phase = MissionPhase::HOVER;

    spdlog::info("Hovering at->X:{} Y:{} Z:{}", ctx->hover.pos.x(),
                 ctx->hover.pos.y(), ctx->hover.pos.z());
  }

  void react(const EventLand &) override { transit<Landing>(); }
  void react(const EventAllowCmdCtrl &) override { transit<CmdCtrl>(); }

  void react(const EventTick &) override {
    ensure_hover_initialized(ctx->build_snapshot());
  }
};

// --- State: CmdCtrl ---
struct CmdCtrl : MissionFSM {
  void entry() override {
    ctx->landing.initialized = false;
    ctx->landing.c12_started = false;

    const auto snap = ctx->build_snapshot();
    if (snap.odom_msg != nullptr) {
      ctx->hover.pos = Eigen::Vector3d(snap.odom_msg->pose.pose.position.x,
                                       snap.odom_msg->pose.pose.position.y,
                                       snap.odom_msg->pose.pose.position.z);
      const Eigen::Quaterniond q(snap.odom_msg->pose.pose.orientation.w,
                                  snap.odom_msg->pose.pose.orientation.x,
                                  snap.odom_msg->pose.pose.orientation.y,
                                  snap.odom_msg->pose.pose.orientation.z);
      const auto yaw = controller::yawFromQuat(q);
      ctx->hover.q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
      ctx->hover.initialized = true;
    }
    ctx->phase = MissionPhase::CMD_CTRL;
  }

  void react(const EventForceHover &) override { transit<Hover_State>(); }
  void react(const EventLand &) override { transit<Landing>(); }

  void react(const EventTick &) override {
    const auto snap = ctx->build_snapshot();
    if (!snap.cmd_fresh ||
        ctx->cmdctrl_hz < static_cast<int>(
            ctx->params()->statemachine_params.l2_cmd_ctrl_min_hz)) {
      transit<Hover_State>();
      return;
    }
    ensure_hover_initialized(snap);
  }
};

// --- State: Landing ---
struct Landing : MissionFSM {
  void entry() override {
    const auto snap = ctx->build_snapshot();

    if (snap.odom_msg != nullptr) {
      ctx->landing.start_pos =
          Eigen::Vector3d(snap.odom_msg->pose.pose.position.x,
                          snap.odom_msg->pose.pose.position.y,
                          snap.odom_msg->pose.pose.position.z);
      ctx->landing.start_q =
          Eigen::Quaterniond(snap.odom_msg->pose.pose.orientation.w,
                             snap.odom_msg->pose.pose.orientation.x,
                             snap.odom_msg->pose.pose.orientation.y,
                             snap.odom_msg->pose.pose.orientation.z);
    } else {
      ctx->landing.start_pos = ctx->hover.pos;
      if (snap.imu_msg != nullptr) {
        const auto &q = snap.imu_msg->orientation;
        ctx->landing.start_q = Eigen::Quaterniond(q.w, q.x, q.y, q.z);
      } else {
        ctx->landing.start_q = Eigen::Quaterniond::Identity();
      }
    }
    ctx->landing.start_time = snap.now;
    ctx->landing.initialized = true;
    ctx->landing.c12_started = false;
    ctx->phase = MissionPhase::LANDING;
  }

  void react(const EventForceHover &) override { transit<Hover_State>(); }

  void react(const EventTick &) override {
    const auto snap = ctx->build_snapshot();
    if (!ctx->landing.initialized) return;

    const auto elapsed_ms = timeDuration(ctx->landing.start_time, snap.now);
    if (elapsed_ms > ctx->params()->guard_params.land_timeout) {
      spdlog::warn("Landing timeout reached, forcing disarm path");
      transit<Standby>();
      return;
    }

    if (snap.odom_msg == nullptr || !snap.odom_fresh) return;

    const double speed = ctx->params()->statemachine_params.l2_takeoff_landing_speed;
    const double des_z = ctx->landing.start_pos.z() - speed * (elapsed_ms / 1000.0);
    const Eigen::Vector3d vel(snap.odom_msg->twist.twist.linear.x,
                               snap.odom_msg->twist.twist.linear.y,
                               snap.odom_msg->twist.twist.linear.z);

    const double c = ctx->params()->statemachine_params.l2_land_position_deviation_c;
    const double vc = ctx->params()->statemachine_params.l2_land_velocity_thr_c;
    const double tc = ctx->params()->statemachine_params.l2_land_time_keep_c;

    const bool c12 = (des_z - snap.odom_msg->pose.pose.position.z) < c && vel.norm() < vc;

    if (c12) {
      if (!ctx->landing.c12_started) {
        ctx->landing.c12_reached_time = snap.now;
        ctx->landing.c12_started = true;
      }
      if (timeDuration(ctx->landing.c12_reached_time, snap.now) > tc) {
        spdlog::info("Successfully landed");
        transit<Standby>();
      }
    } else {
      ctx->landing.c12_started = false;
    }
  }
};

// --- State: Failsafe ---
struct Failsafe : MissionFSM {
  void entry() override {
    const auto snap = ctx->build_snapshot();
    ctx->phase = MissionPhase::FAILSAFE;

    if (active_guard_is_hold()) {
      if (snap.odom_msg != nullptr) {
        ctx->hover.pos = Eigen::Vector3d(snap.odom_msg->pose.pose.position.x,
                                         snap.odom_msg->pose.pose.position.y,
                                         snap.odom_msg->pose.pose.position.z);
        const Eigen::Quaterniond q(snap.odom_msg->pose.pose.orientation.w,
                                    snap.odom_msg->pose.pose.orientation.x,
                                    snap.odom_msg->pose.pose.orientation.y,
                                    snap.odom_msg->pose.pose.orientation.z);
        const auto yaw = controller::yawFromQuat(q);
        ctx->hover.q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
        ctx->hover.initialized = true;
      }
      return;
    }

    init_landing(snap);
  }

  void react(const EventFailsafeCleared &) override { transit<Hover_State>(); }

  void react(const EventTick &) override {
    const auto snap = ctx->build_snapshot();

    if (active_guard_is_hold()) {
      ensure_hover_initialized(snap);
      return;
    }

    if (ctx->landing.initialized &&
        timeDuration(ctx->landing.start_time, snap.now) >
            ctx->params()->guard_params.land_timeout) {
      spdlog::warn("Failsafe landing timeout reached, force disarm");
      transit<Standby>();
    }
  }

private:
  bool active_guard_is_hold() const {
    return ctx->active_guard_action == params::Guard::HOLD;
  }

  void init_landing(const MissionContextSnapshot &snap) {
    if (snap.odom_msg != nullptr) {
      ctx->landing.start_pos =
          Eigen::Vector3d(snap.odom_msg->pose.pose.position.x,
                          snap.odom_msg->pose.pose.position.y,
                          snap.odom_msg->pose.pose.position.z);
      ctx->landing.start_q =
          Eigen::Quaterniond(snap.odom_msg->pose.pose.orientation.w,
                             snap.odom_msg->pose.pose.orientation.x,
                             snap.odom_msg->pose.pose.orientation.y,
                             snap.odom_msg->pose.pose.orientation.z);
    } else {
      ctx->landing.start_pos = ctx->hover.pos;
      if (snap.imu_msg != nullptr) {
        const auto &q = snap.imu_msg->orientation;
        ctx->landing.start_q = Eigen::Quaterniond(q.w, q.x, q.y, q.z);
      } else {
        ctx->landing.start_q = Eigen::Quaterniond::Identity();
      }
    }
    ctx->landing.start_time = snap.now;
    ctx->landing.initialized = true;
    ctx->landing.c12_started = false;
  }
};

} // namespace px4ctrl
