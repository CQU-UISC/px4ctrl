#include "coordinator.h"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstring>
#include <thread>

#include <mavros_msgs/msg/state.hpp>
#include <px4ctrl_msgs/msg/command.hpp>

#include "datas.h"
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

  const auto now = clock::now();
  ctx_->phase_enter_time = now;
  last_log_state_time_ = now;
  odom_low_log_time_ = now;
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
      [this](const auto &cmd) { on_client_command(cmd); });

  MissionFSM::start();

  while (ok_) {
    bridge_->spin_once();
    compute_hz();
    process();
    server_->pub(build_telemetry());
    std::this_thread::sleep_for(std::chrono::milliseconds(delta_t));
  }
}

void Coordinator::compute_hz() {
  if (timePassed(ctx_->odom_last_time) > 100) {
    ctx_->odom_hz = ctx_->odom_count * 10;
    ctx_->odom_count = 0;
    ctx_->odom_last_time = clock::now();
    if (ctx_->odom_hz < 100) {
      if (!odom_low_active_) {
        spdlog::warn("odom hz low:{}", ctx_->odom_hz);
        odom_low_active_ = true;
        odom_low_suppressed_ = 0;
        odom_low_log_time_ = ctx_->odom_last_time;
      } else if (timeDuration(odom_low_log_time_, ctx_->odom_last_time) > 2000) {
        spdlog::warn("odom hz low:{} (suppressed {} repeated logs)",
                     ctx_->odom_hz, odom_low_suppressed_);
        odom_low_suppressed_ = 0;
        odom_low_log_time_ = ctx_->odom_last_time;
      } else {
        ++odom_low_suppressed_;
      }
    } else if (odom_low_active_) {
      spdlog::info("odom hz recovered:{} (suppressed {} repeated low-hz logs)",
                   ctx_->odom_hz, odom_low_suppressed_);
      odom_low_active_ = false;
      odom_low_suppressed_ = 0;
      odom_low_log_time_ = ctx_->odom_last_time;
    }
  }
  if (timePassed(ctx_->cmdctrl_last_time) > 100) {
    ctx_->cmdctrl_hz = ctx_->cmdctrl_count * 10;
    ctx_->cmdctrl_count = 0;
    ctx_->cmdctrl_last_time = clock::now();
  }
}

bool Coordinator::essential_ready(const MissionContextSnapshot &snap) const {
  return snap.state_msg != nullptr && snap.ext_state_msg != nullptr &&
         snap.imu_msg != nullptr && snap.battery_msg != nullptr;
}

void Coordinator::process() {
  const auto snap = ctx_->build_snapshot();

  // Update cached state
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

  // Wait for essential messages
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
    build_proof_alive(snap, ctrl_cmd);
    apply_control(ctrl_cmd);

    ctx_->phase = MissionPhase::STANDBY;
    return;
  }

  // Check for offboard loss
  if (!snap.offboard || !snap.armed) {
    if (ctx_->phase != MissionPhase::STANDBY) {
      spdlog::info("Offboard or armed lost, returning to Standby");
      MissionFSM::dispatch(EventOffboardLost{});
    }
    ctx_->phase = MissionPhase::STANDBY;

    controller::ControlCommand ctrl_cmd;
    build_proof_alive(snap, ctrl_cmd);
    apply_control(ctrl_cmd);
    return;
  }

  // Evaluate guards
  auto decision = safety_monitor_->evaluate(snap);
  ctx_->active_guard_action = decision.action;

  // Dispatch guard events
  if (decision.triggered) {
    if (!was_failsafe_) {
      spdlog::warn("Entering failsafe: {} ({})", decision.reason,
                   guardFlagsToString(decision.flags));
      EventFailsafe fs_event;
      fs_event.result = decision;
      MissionFSM::dispatch(fs_event);
      was_failsafe_ = true;
    }
  } else if (was_failsafe_) {
    spdlog::info("Failsafe cleared, returning to Hover");
    MissionFSM::dispatch(EventFailsafeCleared{});
    was_failsafe_ = false;

    // Also process any pending phase request
    if (auto req = ctx_->pop_request()) {
      handle_phase_request(*req);
    }
  }

  // Handle DISARM action immediately (bypass FSM for emergency disarm)
  if (decision.triggered && decision.action == params::Guard::DISARM) {
    if (!bridge_->force_disarm()) {
      spdlog::error("Guard force disarm failed");
    }
    controller_->resetThrustMapping();
    ctx_->phase = MissionPhase::STANDBY;
    MissionFSM::dispatch(EventOffboardLost{});

    controller::ControlCommand ctrl_cmd;
    build_proof_alive(snap, ctrl_cmd);
    apply_control(ctrl_cmd);
    return;
  }

  // Process phase requests from client
  auto requested = ctx_->pop_request();
  if (requested.has_value() && !was_failsafe_) {
    handle_phase_request(*requested);
  }

  // Tick FSM for automatic transitions
  MissionFSM::dispatch(EventTick{});

  // Publish cmdctrl state
  bool allow_cmd_ctrl = MissionFSM::is_in_state<CmdCtrl>();
  if (allow_cmd_ctrl != ctx_->allow_cmdctrl_pub_state) {
    bridge_->pub_allow_cmdctrl(allow_cmd_ctrl);
    ctx_->allow_cmdctrl_pub_state = allow_cmd_ctrl;
  }

  // Select control source and build command
  auto source = select_control_source(snap);
  controller::ControlCommand ctrl_cmd;
  build_command(snap, source, ctrl_cmd);
  apply_control(ctrl_cmd);

  if (timePassed(last_log_state_time_) > 1000) {
    last_log_state_time_ = snap.now;
    spdlog::debug("phase:{},source:{},offboard:{},armed:{}",
                  fsm_internal::phaseName(ctx_->phase),
                  fsm_internal::sourceName(source),
                  static_cast<int>(ctx_->offboard_state),
                  static_cast<int>(ctx_->armed_state));
  }
}

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

ControlSource
Coordinator::select_control_source(const MissionContextSnapshot &snap) const {
  if (!snap.offboard || !snap.armed) {
    return ControlSource::PROOF_ALIVE;
  }

  if (ctx_->phase == MissionPhase::FAILSAFE) {
    if (ctx_->active_guard_action == params::Guard::HOLD && snap.odom_fresh) {
      return ControlSource::SE3;
    }
    return ControlSource::SAFE_LANDING;
  }

  if (ctx_->phase == MissionPhase::CMD_CTRL && snap.cmd_fresh &&
      ctx_->cmdctrl_hz >= static_cast<int>(
          params_->statemachine_params.l2_cmd_ctrl_min_hz)) {
    return ControlSource::EXTERNAL_CMD;
  }

  if (!snap.odom_fresh &&
      (ctx_->phase == MissionPhase::TAKEOFF ||
       ctx_->phase == MissionPhase::HOVER ||
       ctx_->phase == MissionPhase::CMD_CTRL ||
       ctx_->phase == MissionPhase::LANDING)) {
    return ControlSource::SAFE_LANDING;
  }

  return ControlSource::SE3;
}

// --- Command builders ---

void Coordinator::build_proof_alive(const MissionContextSnapshot &snap,
                                    controller::ControlCommand &ctrl_cmd) const {
  ctrl_cmd.type = params::ControlType::ATTITUDE;
  if (snap.imu_msg != nullptr) {
    const auto &quat = snap.imu_msg->orientation;
    ctrl_cmd.attitude = Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z);
  } else {
    ctrl_cmd.attitude = Eigen::Quaterniond::Identity();
  }
  ctrl_cmd.thrust = 0.01;
}

bool Coordinator::build_se3_command(const MissionContextSnapshot &snap,
                                    controller::ControlCommand &ctrl_cmd) {
  if (snap.odom_msg == nullptr || snap.imu_msg == nullptr) return false;

  const double speed = params_->statemachine_params.l2_takeoff_landing_speed;
  controller::DesiredState des;

  switch (ctx_->phase) {
  case MissionPhase::STANDBY:
    ctrl_cmd.type = params::ControlType::BODYRATES;
    ctrl_cmd.thrust = 0.1;
    ctrl_cmd.bodyrates = Eigen::Vector3d::Zero();
    return true;

  case MissionPhase::TAKEOFF: {
    if (!ctx_->takeoff.initialized) return false;
    des.p = ctx_->takeoff.start_pos;
    des.q = ctx_->takeoff.start_q;
    des.yaw = controller::yawFromQuat(des.q);
    des.v = Eigen::Vector3d(0, 0, speed);

    const double elapsed_s = timeDuration(ctx_->takeoff.start_time, snap.now) / 1000.0;
    const double target_z =
        ctx_->takeoff.start_pos.z() + params_->statemachine_params.l2_takeoff_height;
    des.p.z() = ctx_->takeoff.start_pos.z() + speed * elapsed_s;
    if (des.p.z() >= target_z) {
      des.p.z() = target_z;
      des.v = Eigen::Vector3d::Zero();
    }

    ctrl_cmd = controller_->runControl(des, *snap.odom_msg, *snap.imu_msg);
    estimate_thrust_from_imu();
    return true;
  }

  case MissionPhase::HOVER:
  case MissionPhase::CMD_CTRL: {
    if (!ctx_->hover.initialized) {
      ctx_->hover.pos = Eigen::Vector3d(snap.odom_msg->pose.pose.position.x,
                                         snap.odom_msg->pose.pose.position.y,
                                         snap.odom_msg->pose.pose.position.z);
      const Eigen::Quaterniond q(snap.odom_msg->pose.pose.orientation.w,
                                  snap.odom_msg->pose.pose.orientation.x,
                                  snap.odom_msg->pose.pose.orientation.y,
                                  snap.odom_msg->pose.pose.orientation.z);
      const auto yaw = controller::yawFromQuat(q);
      ctx_->hover.q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
      ctx_->hover.initialized = true;
    }
    des.p = ctx_->hover.pos;
    des.q = ctx_->hover.q;
    des.yaw = controller::yawFromQuat(des.q);

    ctrl_cmd = controller_->runControl(des, *snap.odom_msg, *snap.imu_msg);
    estimate_thrust_from_imu();
    return true;
  }

  case MissionPhase::LANDING:
  case MissionPhase::FAILSAFE: {
    if (ctx_->phase == MissionPhase::FAILSAFE &&
        ctx_->active_guard_action == params::Guard::HOLD) {
      des.p = ctx_->hover.pos;
      des.q = ctx_->hover.q;
      des.yaw = controller::yawFromQuat(des.q);
      ctrl_cmd = controller_->runControl(des, *snap.odom_msg, *snap.imu_msg);
      estimate_thrust_from_imu();
      return true;
    }

    if (!ctx_->landing.initialized) {
      ctx_->landing.start_pos =
          Eigen::Vector3d(snap.odom_msg->pose.pose.position.x,
                          snap.odom_msg->pose.pose.position.y,
                          snap.odom_msg->pose.pose.position.z);
      ctx_->landing.start_q =
          Eigen::Quaterniond(snap.odom_msg->pose.pose.orientation.w,
                             snap.odom_msg->pose.pose.orientation.x,
                             snap.odom_msg->pose.pose.orientation.y,
                             snap.odom_msg->pose.pose.orientation.z);
      ctx_->landing.start_time = snap.now;
      ctx_->landing.initialized = true;
      ctx_->landing.c12_started = false;
    }

    des.p = ctx_->landing.start_pos;
    des.q = ctx_->landing.start_q;
    des.yaw = controller::yawFromQuat(des.q);
    des.v = Eigen::Vector3d(0, 0, -speed);
    des.p.z() -= speed * (timeDuration(ctx_->landing.start_time, snap.now) / 1000.0);

    ctrl_cmd = controller_->runControl(des, *snap.odom_msg, *snap.imu_msg);
    estimate_thrust_from_imu();
    return true;
  }
  }

  return false;
}

bool Coordinator::build_external_command(const MissionContextSnapshot &snap,
                                         controller::ControlCommand &ctrl_cmd) {
  if (snap.cmd_msg == nullptr) return false;

  switch (snap.cmd_msg->type) {
  case px4ctrl_msgs::msg::Command::ROTORS_FORCE:
    spdlog::error("not supported type:ROTORS_FORCE");
    return false;

  case px4ctrl_msgs::msg::Command::THRUST_BODYRATE:
    ctrl_cmd.type = params::ControlType::BODYRATES;
    ctrl_cmd.thrust = controller_->thrustMap(snap.cmd_msg->u[0]);
    ctrl_cmd.bodyrates =
        Eigen::Vector3d(snap.cmd_msg->u[1], snap.cmd_msg->u[2], snap.cmd_msg->u[3]);
    return true;

  case px4ctrl_msgs::msg::Command::THRUST_TORQUE:
    spdlog::error("not supported type:THRUST_TORQUE");
    return false;

  case px4ctrl_msgs::msg::Command::DESIRED_POS: {
    if (snap.odom_msg == nullptr || snap.imu_msg == nullptr) return false;
    controller::DesiredState des;
    const auto &des_pos = snap.cmd_msg->pos;
    const auto &des_vel = snap.cmd_msg->vel;
    const auto &des_acc = snap.cmd_msg->acc;
    const auto &des_jerk = snap.cmd_msg->jerk;
    const auto &des_quat = snap.cmd_msg->quat;

    des.p = Eigen::Vector3d(des_pos[0], des_pos[1], des_pos[2]);
    des.v = Eigen::Vector3d(des_vel[0], des_vel[1], des_vel[2]);
    des.a = Eigen::Vector3d(des_acc[0], des_acc[1], des_acc[2]);
    des.j = Eigen::Vector3d(des_jerk[0], des_jerk[1], des_jerk[2]);
    des.q = Eigen::Quaterniond(des_quat[0], des_quat[1], des_quat[2], des_quat[3]);
    des.yaw = snap.cmd_msg->yaw;

    ctrl_cmd = controller_->runControl(des, *snap.odom_msg, *snap.imu_msg);
    estimate_thrust_from_imu();
    return true;
  }

  case px4ctrl_msgs::msg::Command::THRUST_QUAT: {
    ctrl_cmd.type = params::ControlType::ATTITUDE;
    const auto &des_quat = snap.cmd_msg->quat;
    ctrl_cmd.thrust = controller_->thrustMap(snap.cmd_msg->u[0]);
    ctrl_cmd.attitude =
        Eigen::Quaterniond(des_quat[0], des_quat[1], des_quat[2], des_quat[3]);
    return true;
  }
  }

  return false;
}

bool Coordinator::build_safe_landing_command(const MissionContextSnapshot &snap,
                                             controller::ControlCommand &ctrl_cmd) {
  if (snap.imu_msg == nullptr) return false;

  if (!ctx_->landing.initialized) {
    ctx_->landing.start_time = snap.now;
    ctx_->landing.initialized = true;
    ctx_->landing.c12_started = false;
  }

  const auto &imu_q = snap.imu_msg->orientation;
  const Eigen::Quaterniond q_imu(imu_q.w, imu_q.x, imu_q.y, imu_q.z);
  const double yaw = controller::yawFromQuat(q_imu);
  const Eigen::Quaterniond level_q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));

  const auto min_thrust = params_->quadrotor_params.min_thrust;
  const auto max_thrust = params_->quadrotor_params.max_thrust;
  const double hover =
      std::clamp(controller_->getHoverThrustEstimate(), min_thrust, max_thrust);

  const double descent_speed = params_->statemachine_params.l2_takeoff_landing_speed;
  const double descent_ratio =
      std::clamp(descent_speed / std::max(1.0, params_->quadrotor_params.g), 0.05, 0.25);
  const double safe_thrust =
      std::clamp(hover * (1.0 - descent_ratio), min_thrust, max_thrust);

  ctrl_cmd.type = params::ControlType::ATTITUDE;
  ctrl_cmd.attitude = level_q;
  ctrl_cmd.thrust = safe_thrust;

  estimate_thrust_from_imu();
  return true;
}

void Coordinator::build_command(const MissionContextSnapshot &snap,
                                ControlSource source,
                                controller::ControlCommand &ctrl_cmd) {
  bool ok_build = false;
  switch (source) {
  case ControlSource::PROOF_ALIVE:
    build_proof_alive(snap, ctrl_cmd);
    return;
  case ControlSource::SE3:
    ok_build = build_se3_command(snap, ctrl_cmd);
    break;
  case ControlSource::SAFE_LANDING:
    ok_build = build_safe_landing_command(snap, ctrl_cmd);
    break;
  case ControlSource::EXTERNAL_CMD:
    ok_build = build_external_command(snap, ctrl_cmd);
    if (!ok_build) {
      spdlog::warn("External cmd invalid, fallback to hover controller");
      ok_build = build_se3_command(snap, ctrl_cmd);
    }
    break;
  }

  if (!ok_build) {
    if (!build_safe_landing_command(snap, ctrl_cmd)) {
      build_proof_alive(snap, ctrl_cmd);
    }
  }
}

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
        thrust, {cmd.attitude.w(), cmd.attitude.x(), cmd.attitude.y(), cmd.attitude.z()});
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

// --- Client command handling ---

void Coordinator::on_client_command(const ui::ClientPayload &payload) {
  const auto recv_time = clock::now();

  if (payload.command != ui::ClientCommand::HEARTBEAT) {
    spdlog::info("client command:{}",
                 ui::CommandStr[static_cast<int>(payload.command)]);
  }

  if (params_->guard_params.use_rc &&
      payload.command != ui::ClientCommand::HEARTBEAT &&
      payload.command != ui::ClientCommand::SET_SAFETY_LIMITS &&
      !ctx_->has_valid_rc_signal()) {
    spdlog::warn("Reject client command: use_rc enabled but RC signal is invalid");
    ctx_->last_client_cmd_time = recv_time;
    ctx_->has_client_cmd = true;
    return;
  }

  switch (payload.command) {
  case ui::ClientCommand::HEARTBEAT:
    break;

  case ui::ClientCommand::ARM:
    if (params_->guard_params.use_rc) {
      spdlog::warn("Reject ARM: use_rc enabled");
      break;
    }
    if (!bridge_->set_arm(true)) spdlog::error("arm failed");
    controller_->resetThrustMapping();
    break;

  case ui::ClientCommand::FORCE_DISARM:
    if (params_->guard_params.use_rc) {
      spdlog::warn("Reject DISARM: use_rc enabled");
      break;
    }
    if (!bridge_->force_disarm()) spdlog::error("force disarm failed");
    controller_->resetThrustMapping();
    ctx_->request_phase(MissionPhase::STANDBY);
    break;

  case ui::ClientCommand::ENTER_OFFBOARD:
    if (params_->guard_params.use_rc) {
      spdlog::warn("Reject ENTER_OFFBOARD: use_rc enabled");
      break;
    }
    if (!bridge_->enter_offboard()) spdlog::error("enter offboard failed");
    break;

  case ui::ClientCommand::EXIT_OFFBOARD:
    if (params_->guard_params.use_rc) {
      spdlog::warn("Reject EXIT_OFFBOARD: use_rc enabled");
      break;
    }
    if (!bridge_->exit_offboard()) spdlog::error("exit offboard failed");
    break;

  case ui::ClientCommand::TAKEOFF:
    if (!ctx_->offboard_state || !ctx_->armed_state) {
      spdlog::error("Reject TAKEOFF: require OFFBOARD + ARMED");
      break;
    }
    if (MissionFSM::is_in_state<Standby>()) {
      ctx_->request_phase(MissionPhase::TAKEOFF);
    } else {
      spdlog::error("Reject TAKEOFF: invalid phase {}",
                    fsm_internal::phaseName(ctx_->phase));
    }
    break;

  case ui::ClientCommand::LAND:
    if (!ctx_->offboard_state || !ctx_->armed_state) {
      spdlog::error("Reject LAND: require OFFBOARD + ARMED");
      break;
    }
    if (MissionFSM::is_in_state<Hover_State>() ||
        MissionFSM::is_in_state<CmdCtrl>()) {
      ctx_->request_phase(MissionPhase::LANDING);
    } else {
      spdlog::error("Reject LAND: invalid phase {}",
                    fsm_internal::phaseName(ctx_->phase));
    }
    break;

  case ui::ClientCommand::FORCE_HOVER:
    if (!ctx_->offboard_state || !ctx_->armed_state) {
      spdlog::error("Reject FORCE_HOVER: require OFFBOARD + ARMED");
      break;
    }
    if (!MissionFSM::is_in_state<Standby>()) {
      ctx_->request_phase(MissionPhase::HOVER);
    } else {
      spdlog::error("Reject FORCE_HOVER: invalid phase {}",
                    fsm_internal::phaseName(ctx_->phase));
    }
    break;

  case ui::ClientCommand::ALLOW_CMD_CTRL:
    if (!ctx_->offboard_state || !ctx_->armed_state) {
      spdlog::error("Reject ALLOW_CMD_CTRL: require OFFBOARD + ARMED");
      break;
    }
    if (MissionFSM::is_in_state<Hover_State>()) {
      ctx_->request_phase(MissionPhase::CMD_CTRL);
    } else {
      spdlog::error("Reject ALLOW_CMD_CTRL: invalid phase {}",
                    fsm_internal::phaseName(ctx_->phase));
    }
    break;

  case ui::ClientCommand::CHANGE_HOVER_POS: {
    if (!ctx_->offboard_state || !ctx_->armed_state) {
      spdlog::error("Reject CHANGE_HOVER_POS: require OFFBOARD + ARMED");
      break;
    }
    if (!MissionFSM::is_in_state<Hover_State>()) {
      spdlog::error("Reject CHANGE_HOVER_POS: require HOVER phase");
      break;
    }

    double data[7];
    std::memcpy(data, payload.data, sizeof(data));
    bool reject = false;
    for (const auto &d : data) {
      if (std::isnan(d)) {
        reject = true;
        spdlog::error("Reject CHANGE_HOVER_POS: data contains nan");
        break;
      }
    }
    if (reject) break;

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

  ctx_->last_client_cmd_time = recv_time;
  ctx_->has_client_cmd = true;
}

bool Coordinator::validate_safety_limit(double v) const {
  return v == -1.0 || (v > 0.0 && v <= 180.0);
}

void Coordinator::update_safety_limits(const ui::SafetyLimitsPayload &limits) {
  if (!validate_safety_limit(limits.max_roll_deg) ||
      !validate_safety_limit(limits.max_pitch_deg) ||
      !validate_safety_limit(limits.max_yaw_deg)) {
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

  params_->guard_params.geofence_min = {limits.geofence_min[0], limits.geofence_min[1],
                                        limits.geofence_min[2]};
  params_->guard_params.geofence_max = {limits.geofence_max[0], limits.geofence_max[1],
                                        limits.geofence_max[2]};
  params_->guard_params.max_roll_deg = limits.max_roll_deg;
  params_->guard_params.max_pitch_deg = limits.max_pitch_deg;
  params_->guard_params.max_yaw_deg = limits.max_yaw_deg;
  params_->guard_params.enable_geofence = limits.enable_geofence != 0;
  params_->guard_params.enable_attitude_fence = limits.enable_attitude_fence != 0;

  spdlog::info("Applied SET_SAFETY_LIMITS: geofence[{},{},{}]-[{},{},{}], "
               "max_rpy_deg[{},{},{}], en_geo={}, en_att={}",
               limits.geofence_min[0], limits.geofence_min[1], limits.geofence_min[2],
               limits.geofence_max[0], limits.geofence_max[1], limits.geofence_max[2],
               limits.max_roll_deg, limits.max_pitch_deg, limits.max_yaw_deg,
               static_cast<int>(limits.enable_geofence),
               static_cast<int>(limits.enable_attitude_fence));
}

// --- Telemetry ---

ui::ServerPayload Coordinator::build_telemetry() {
  ui::ServerPayload payload{};
  const auto now = clock::now();
  payload.timestamp = to_uint64(now);

  auto battery = ctx_->px4_state()->battery->value().first;
  if (battery == nullptr) {
    payload.battery_voltage = 0;
    payload.battery_remaining = -1;
  } else {
    payload.battery_voltage = battery->voltage;
    payload.battery_remaining = battery->percentage;
  }

  payload.mission_phase = static_cast<int32_t>(ctx_->phase);
  payload.offboard_state = ctx_->offboard_state ? 1 : 0;
  payload.armed_state = ctx_->armed_state ? 1 : 0;

  payload.hover_pos[0] = ctx_->hover.pos.x();
  payload.hover_pos[1] = ctx_->hover.pos.y();
  payload.hover_pos[2] = ctx_->hover.pos.z();
  payload.hover_quat[0] = ctx_->hover.q.w();
  payload.hover_quat[1] = ctx_->hover.q.x();
  payload.hover_quat[2] = ctx_->hover.q.y();
  payload.hover_quat[3] = ctx_->hover.q.z();

  const auto odom_state = ctx_->px4_state()->odom->value();
  auto odom = odom_state.first;
  if (odom == nullptr) {
    payload.pos[0] = payload.pos[1] = payload.pos[2] = 0;
    payload.vel[0] = payload.vel[1] = payload.vel[2] = 0;
    payload.omega[0] = payload.omega[1] = payload.omega[2] = 0;
    payload.quat[0] = 1;
    payload.quat[1] = payload.quat[2] = payload.quat[3] = 0;
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

  payload.odom_hz = ctx_->odom_hz;
  payload.cmdctrl_hz = ctx_->cmdctrl_hz;

  payload.thrust_setpoint = static_cast<float>(ctx_->last_ctrl_cmd.thrust);
  if (ctx_->last_ctrl_cmd.type == params::ControlType::BODYRATES) {
    payload.omega_setpoint[0] = static_cast<float>(ctx_->last_ctrl_cmd.bodyrates.x());
    payload.omega_setpoint[1] = static_cast<float>(ctx_->last_ctrl_cmd.bodyrates.y());
    payload.omega_setpoint[2] = static_cast<float>(ctx_->last_ctrl_cmd.bodyrates.z());
  } else {
    payload.omega_setpoint[0] = payload.omega_setpoint[1] = payload.omega_setpoint[2] = 0;
  }

  payload.thrust_map[0] = static_cast<float>(controller_->getThr2AccEstimate());
  payload.thrust_map[1] = static_cast<float>(controller_->getHoverThrustEstimate());
  payload.thrust_map[2] = static_cast<float>(ctx_->last_ctrl_cmd.thrust);

  payload.telemetry_seq = ctx_->telemetry_seq++;
  payload.guard_flags = ctx_->guard_flags;
  payload.odom_age_ms =
      (odom != nullptr)
          ? static_cast<float>(std::max(0.0, timeDuration(odom_state.second, now)))
          : -1.0F;
  payload.client_cmd_age_ms =
      ctx_->has_client_cmd
          ? static_cast<float>(std::max(0.0, timeDuration(ctx_->last_client_cmd_time, now)))
          : -1.0F;
  payload.speed_norm = std::sqrt(payload.vel[0] * payload.vel[0] +
                                 payload.vel[1] * payload.vel[1] +
                                 payload.vel[2] * payload.vel[2]);

  const Eigen::Quaterniond q(payload.quat[0], payload.quat[1], payload.quat[2],
                             payload.quat[3]);
  const Eigen::Vector3d body_z = q * Eigen::Vector3d::UnitZ();
  const double cos_tilt = std::clamp(body_z.dot(Eigen::Vector3d::UnitZ()), -1.0, 1.0);
  payload.tilt_deg = static_cast<float>(std::acos(cos_tilt) * fsm_internal::kRadToDeg);

  payload.roll_deg = static_cast<float>(ctx_->roll_deg);
  payload.pitch_deg = static_cast<float>(ctx_->pitch_deg);
  payload.yaw_deg = static_cast<float>(ctx_->yaw_deg);

  payload.geofence_min[0] = static_cast<float>(params_->guard_params.geofence_min[0]);
  payload.geofence_min[1] = static_cast<float>(params_->guard_params.geofence_min[1]);
  payload.geofence_min[2] = static_cast<float>(params_->guard_params.geofence_min[2]);
  payload.geofence_max[0] = static_cast<float>(params_->guard_params.geofence_max[0]);
  payload.geofence_max[1] = static_cast<float>(params_->guard_params.geofence_max[1]);
  payload.geofence_max[2] = static_cast<float>(params_->guard_params.geofence_max[2]);
  payload.max_roll_deg = static_cast<float>(params_->guard_params.max_roll_deg);
  payload.max_pitch_deg = static_cast<float>(params_->guard_params.max_pitch_deg);
  payload.max_yaw_deg = static_cast<float>(params_->guard_params.max_yaw_deg);
  payload.enable_geofence =
      static_cast<uint8_t>(params_->guard_params.enable_geofence ? 1 : 0);
  payload.enable_attitude_fence =
      static_cast<uint8_t>(params_->guard_params.enable_attitude_fence ? 1 : 0);
  payload.use_rc = static_cast<uint8_t>(params_->guard_params.use_rc ? 1 : 0);

  return payload;
}

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
