#include "coordinator.h"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <thread>

#include <mavros_msgs/msg/state.hpp>
#include <spdlog/spdlog.h>

#include "fsm_internal.h"

namespace px4ctrl {

Coordinator::Coordinator(std::shared_ptr<Px4CtrlRosBridge> bridge,
                         std::shared_ptr<Px4State> px4_state,
                         std::shared_ptr<Px4CtrlParams> params,
                         std::shared_ptr<ui::Px4Server> server)
    : bridge_(std::move(bridge)), server_(std::move(server)),
      params_(std::move(params)) {
  assert(bridge_ != nullptr && px4_state != nullptr && params_ != nullptr);

  ctx_ = std::make_shared<Context>(std::move(px4_state), params_);
  MissionFSM::set_context(ctx_);
  safety_monitor_ = std::make_unique<SafetyMonitor>(ctx_);

  controller_ = std::make_shared<controller::Se3Control>(
      params_->control_params, params_->quadrotor_params);
  command_builder_ = std::make_unique<CommandBuilder>(controller_, ctx_);
  client_handler_ = std::make_unique<ClientCommandHandler>(bridge_, ctx_, params_,
                                                            controller_);
  telemetry_builder_ =
      std::make_unique<TelemetryBuilder>(ctx_, params_, controller_);

  const auto now = clock::now();
  ctx_->phase_enter_time = now;
  last_log_state_time_ = now;
  ctx_->odom_last_time = now;
  ctx_->cmdctrl_last_time = now;
  ctx_->last_client_cmd_time = now;
  ctx_->last_ctrl_cmd.type = params::ControlType::ATTITUDE;
  ctx_->last_ctrl_cmd.attitude = Eigen::Quaterniond::Identity();
  ctx_->last_ctrl_cmd.bodyrates = Eigen::Vector3d::Zero();
  ctx_->last_ctrl_cmd.thrust = 0.0;
}

void Coordinator::stop() { ok_ = false; }

void Coordinator::run() {
  int delta_t = static_cast<int>(
      1000 / std::max<uint32_t>(1, params_->statemachine_params.freq));

  odom_hold_ = ctx_->px4_state()->odom->observe(
      [this](auto &) { ctx_->odom_count++; });
  ctrl_hold_ = ctx_->px4_state()->ctrl_command->observe(
      [this](auto &) { ctx_->cmdctrl_count++; });
  client_hold_ = server_->client_data.observe(
      [this](const auto &cmd) { client_handler_->handle(cmd); });

  MissionFSM::start();

  while (ok_) {
    bridge_->spin_once();
    compute_hz();
    process();
    server_->pub(telemetry_builder_->build());
    std::this_thread::sleep_for(std::chrono::milliseconds(delta_t));
  }
}

// --- Hz monitoring ---

void Coordinator::compute_hz() {
  if (timePassed(ctx_->odom_last_time) > 100) {
    ctx_->odom_hz = ctx_->odom_count * 10;
    ctx_->odom_count = 0;
    ctx_->odom_last_time = clock::now();

    if (ctx_->odom_hz < 100) {
      auto r = odom_hz_suppressor_.trigger(ctx_->odom_last_time);
      if (r.should_log) {
        if (r.is_first) {
          spdlog::warn("odom hz low:{}", ctx_->odom_hz);
        } else {
          spdlog::warn("odom hz low:{} (suppressed {} repeated logs)",
                       ctx_->odom_hz, r.suppressed);
        }
      }
    } else if (auto r = odom_hz_suppressor_.clear(); r.was_active) {
      spdlog::info("odom hz recovered:{} (suppressed {} repeated low-hz logs)",
                   ctx_->odom_hz, r.suppressed);
    }
  }

  if (timePassed(ctx_->cmdctrl_last_time) > 100) {
    ctx_->cmdctrl_hz = ctx_->cmdctrl_count * 10;
    ctx_->cmdctrl_count = 0;
    ctx_->cmdctrl_last_time = clock::now();
  }
}

// --- Main processing ---

bool Coordinator::essential_ready(const MissionContextSnapshot &snap) {
  return snap.state_msg != nullptr && snap.ext_state_msg != nullptr &&
         snap.imu_msg != nullptr && snap.battery_msg != nullptr;
}

void Coordinator::update_cached_state(const MissionContextSnapshot &snap) {
  ctx_->offboard_state = snap.offboard;
  ctx_->armed_state = snap.armed;
  if (snap.odom_msg != nullptr) {
    const Eigen::Quaterniond q(snap.odom_msg->pose.pose.orientation.w,
                               snap.odom_msg->pose.pose.orientation.x,
                               snap.odom_msg->pose.pose.orientation.y,
                               snap.odom_msg->pose.pose.orientation.z);
    const auto rpy_deg = fsm_internal::quatToRpyDeg(q);
    ctx_->roll_deg = rpy_deg[0];
    ctx_->pitch_deg = rpy_deg[1];
    ctx_->yaw_deg = rpy_deg[2];
  }
}

void Coordinator::process() {
  const auto snap = ctx_->build_snapshot();
  update_cached_state(snap);

  // Wait for essential messages — send proof-of-life until ready
  if (!essential_ready(snap)) {
    if (timePassedSeconds(last_log_state_time_) > 2) {
      std::string missing;
      auto append = [&](const char *name) {
        if (!missing.empty()) missing += "/";
        missing += name;
      };
      if (snap.state_msg == nullptr) append("state");
      if (snap.ext_state_msg == nullptr) append("ext_state");
      if (snap.imu_msg == nullptr) append("imu");
      if (snap.battery_msg == nullptr) append("battery");
      spdlog::info("Waiting for essential messages, missing: {}", missing);
      last_log_state_time_ = snap.now;
    }

    controller::ControlCommand ctrl_cmd;
    command_builder_->build(snap, ctrl_cmd);
    apply_control(ctrl_cmd);
    ctx_->phase = MissionPhase::STANDBY;
    return;
  }

  // Offboard or armed lost → standby
  if (!snap.offboard || !snap.armed) {
    if (ctx_->phase != MissionPhase::STANDBY) {
      spdlog::info("Offboard or armed lost, returning to Standby");
      MissionFSM::dispatch(EventOffboardLost{});
    }
    ctx_->phase = MissionPhase::STANDBY;

    controller::ControlCommand ctrl_cmd;
    command_builder_->build(snap, ctrl_cmd);
    apply_control(ctrl_cmd);
    return;
  }

  // Evaluate safety guards
  auto decision = safety_monitor_->evaluate(snap);
  ctx_->active_guard_action = decision.action;
  ctx_->guard_flags = decision.flags;

  // Dispatch guard events — FSM is the single source of truth
  bool in_failsafe = MissionFSM::is_in_state<Failsafe>();
  if (decision.triggered && !ctx_->disarmed_by_timeout) {
    if (!in_failsafe) {
      spdlog::warn("Entering failsafe: {} ({})", decision.reason,
                   guardFlagsToString(decision.flags));
      EventFailsafe fs_event;
      fs_event.result = decision;
      MissionFSM::dispatch(fs_event);
    }
  } else if (in_failsafe && ctx_->active_guard_action == params::Guard::HOLD) {
    // Only auto-clear for HOLD.  LAND actions must complete (C1/C2 or
    // timeout) before exiting failsafe, even if the triggering condition
    // has cleared.
    spdlog::info("Failsafe cleared, returning to Hover");
    MissionFSM::dispatch(EventFailsafeCleared{});

    if (auto req = ctx_->pop_request()) {
      handle_phase_request(*req);
    }
  }

  // Emergency disarm (bypasses FSM)
  if (decision.triggered && decision.action == params::Guard::DISARM) {
    if (!bridge_->force_disarm()) {
      spdlog::error("Guard force disarm failed");
    }
    controller_->resetThrustMapping();
    ctx_->phase = MissionPhase::STANDBY;
    MissionFSM::dispatch(EventOffboardLost{});

    controller::ControlCommand ctrl_cmd;
    command_builder_->build(snap, ctrl_cmd);
    apply_control(ctrl_cmd);
    return;
  }

  // Process phase requests from client
  if (auto req = ctx_->pop_request()) {
    if (!MissionFSM::is_in_state<Failsafe>()) {
      handle_phase_request(*req);
    }
  }

  // Tick FSM for automatic transitions
  MissionFSM::dispatch(EventTick{});

  // Handle pending disarm from FSM timeout (landing/failsafe)
  if (ctx_->pending_disarm) {
    ctx_->pending_disarm = false;
    ctx_->disarmed_by_timeout = true;
    spdlog::info("FSM landing timeout — force disarming");
    if (!bridge_->force_disarm()) {
      spdlog::error("force disarm failed");
    }
    controller_->resetThrustMapping();
  }

  // Publish cmdctrl state
  bool allow_cmd_ctrl = MissionFSM::is_in_state<CmdCtrl>();
  if (allow_cmd_ctrl != ctx_->allow_cmdctrl_pub_state) {
    bridge_->pub_allow_cmdctrl(allow_cmd_ctrl);
    ctx_->allow_cmdctrl_pub_state = allow_cmd_ctrl;
  }

  // Build and apply control command
  controller::ControlCommand ctrl_cmd;
  auto source = command_builder_->build(snap, ctrl_cmd);
  apply_control(ctrl_cmd);

  if (source != ControlSource::PROOF_ALIVE) {
    estimate_thrust_from_imu();
  }

  // Periodic status log
  if (timePassed(last_log_state_time_) > 1000) {
    last_log_state_time_ = snap.now;
    spdlog::debug("phase:{},source:{},offboard:{},armed:{}",
                  fsm_internal::phaseName(ctx_->phase),
                  fsm_internal::sourceName(source),
                  static_cast<int>(ctx_->offboard_state),
                  static_cast<int>(ctx_->armed_state));
  }
}

// --- Phase request → FSM event mapping ---

void Coordinator::handle_phase_request(MissionPhase requested) {
  switch (requested) {
  case MissionPhase::TAKEOFF:
    MissionFSM::dispatch(EventTakeoff{});
    break;
  case MissionPhase::HOVER:
    MissionFSM::dispatch(EventForceHover{});
    break;
  case MissionPhase::CMD_CTRL:
    MissionFSM::dispatch(EventAllowCmdCtrl{});
    break;
  case MissionPhase::LANDING:
    MissionFSM::dispatch(EventLand{});
    break;
  default:
    break;
  }
}

// --- Control output ---

void Coordinator::apply_control(const controller::ControlCommand &cmd) {
  double thrust = std::clamp(cmd.thrust, params_->quadrotor_params.min_thrust,
                             params_->quadrotor_params.max_thrust);

  switch (cmd.type) {
  case params::ControlType::BODYRATES: {
    Eigen::Vector3d bodyrates =
        cmd.bodyrates.cwiseMax(-params_->quadrotor_params.max_bodyrate)
            .cwiseMin(params_->quadrotor_params.max_bodyrate);

    ctx_->last_ctrl_cmd = cmd;
    ctx_->last_ctrl_cmd.type = params::ControlType::BODYRATES;
    ctx_->last_ctrl_cmd.thrust = thrust;
    ctx_->last_ctrl_cmd.bodyrates = bodyrates;

    bridge_->pub_bodyrates_target(
        thrust, {bodyrates.x(), bodyrates.y(), bodyrates.z()});
    break;
  }
  case params::ControlType::ATTITUDE: {
    ctx_->last_ctrl_cmd = cmd;
    ctx_->last_ctrl_cmd.type = params::ControlType::ATTITUDE;
    ctx_->last_ctrl_cmd.thrust = thrust;

    bridge_->pub_attitude_target(
        thrust,
        {cmd.attitude.w(), cmd.attitude.x(), cmd.attitude.y(), cmd.attitude.z()});
    break;
  }
  }
}

void Coordinator::estimate_thrust_from_imu() {
  const auto imu = ctx_->px4_state()->imu->value();
  if (imu.first == nullptr) return;
  const Eigen::Vector3d est_a(imu.first->linear_acceleration.x,
                               imu.first->linear_acceleration.y,
                               imu.first->linear_acceleration.z);
  controller_->estimateThrustModel(est_a, imu.second);
}

// --- Public helpers ---

std::pair<const Eigen::Vector3d, const Eigen::Quaterniond>
Coordinator::get_hovering_pos() const {
  return {ctx_->hover.pos, ctx_->hover.q};
}

bool Coordinator::set_hovering_pos(const Eigen::Vector3d &pos,
                                   const Eigen::Quaterniond &q) {
  ctx_->hover.pos = pos;
  ctx_->hover.q = q;
  ctx_->hover.initialized = true;
  return true;
}

} // namespace px4ctrl
