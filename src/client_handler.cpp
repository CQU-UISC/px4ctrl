#include "client_handler.h"

#include <cmath>
#include <cstring>

#include <spdlog/spdlog.h>

#include "datas.h"
#include "fsm_internal.h"
#include "mission_fsm.h"

namespace px4ctrl {

ClientCommandHandler::ClientCommandHandler(
    std::shared_ptr<Px4CtrlRosBridge> bridge, std::shared_ptr<Context> ctx,
    std::shared_ptr<Px4CtrlParams> params,
    std::shared_ptr<controller::Se3Control> ctrl)
    : bridge_(std::move(bridge)), ctx_(std::move(ctx)),
      params_(std::move(params)), controller_(std::move(ctrl)) {}

void ClientCommandHandler::handle(const ui::ClientPayload &payload) {
  const auto recv_time = clock::now();

  if (payload.command != ui::ClientCommand::HEARTBEAT) {
    spdlog::info("client command:{}",
                 ui::CommandStr[static_cast<int>(payload.command)]);
  }

  if (params_->drone_id != 0 && payload.id != params_->drone_id) {
    spdlog::warn("Reject client command: drone_id mismatch (expected {}, got {})",
                 params_->drone_id, payload.id);
    return;
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
    handle_arm();
    break;

  case ui::ClientCommand::FORCE_DISARM:
    handle_force_disarm();
    break;

  case ui::ClientCommand::ENTER_OFFBOARD:
    handle_enter_offboard();
    break;

  case ui::ClientCommand::EXIT_OFFBOARD:
    handle_exit_offboard();
    break;

  case ui::ClientCommand::TAKEOFF:
    handle_takeoff();
    break;

  case ui::ClientCommand::LAND:
    handle_land();
    break;

  case ui::ClientCommand::FORCE_HOVER:
    handle_force_hover();
    break;

  case ui::ClientCommand::ALLOW_CMD_CTRL:
    handle_allow_cmd_ctrl();
    break;

  case ui::ClientCommand::CHANGE_HOVER_POS:
    handle_change_hover_pos(payload);
    break;

  case ui::ClientCommand::SET_SAFETY_LIMITS:
    handle_set_safety_limits(payload);
    break;
  }

  ctx_->last_client_cmd_time = recv_time;
  ctx_->has_client_cmd = true;
}

// --- Individual command handlers ---

void ClientCommandHandler::handle_arm() {
  if (params_->guard_params.use_rc) {
    spdlog::warn("Reject ARM: use_rc enabled");
    return;
  }
  if (!bridge_->set_arm(true)) spdlog::error("arm failed");
  controller_->resetThrustMapping();
  ctx_->disarmed_by_timeout = false;
}

void ClientCommandHandler::handle_force_disarm() {
  if (params_->guard_params.use_rc) {
    spdlog::warn("Reject DISARM: use_rc enabled");
    return;
  }
  if (!bridge_->force_disarm()) spdlog::error("force disarm failed");
  controller_->resetThrustMapping();
  ctx_->request_phase(MissionPhase::STANDBY);
}

void ClientCommandHandler::handle_enter_offboard() {
  if (params_->guard_params.use_rc) {
    spdlog::warn("Reject ENTER_OFFBOARD: use_rc enabled");
    return;
  }
  if (!bridge_->enter_offboard()) spdlog::error("enter offboard failed");
}

void ClientCommandHandler::handle_exit_offboard() {
  if (params_->guard_params.use_rc) {
    spdlog::warn("Reject EXIT_OFFBOARD: use_rc enabled");
    return;
  }
  if (!bridge_->exit_offboard()) spdlog::error("exit offboard failed");
}

void ClientCommandHandler::handle_takeoff() {
  if (!ctx_->offboard_state || !ctx_->armed_state) {
    spdlog::error("Reject TAKEOFF: require OFFBOARD + ARMED");
    return;
  }
  if (MissionFSM::is_in_state<Standby>()) {
    ctx_->request_phase(MissionPhase::TAKEOFF);
  } else {
    spdlog::error("Reject TAKEOFF: invalid phase {}",
                  fsm_internal::phaseName(ctx_->phase));
  }
}

void ClientCommandHandler::handle_land() {
  if (!ctx_->offboard_state || !ctx_->armed_state) {
    spdlog::error("Reject LAND: require OFFBOARD + ARMED");
    return;
  }
  if (MissionFSM::is_in_state<Hover_State>() ||
      MissionFSM::is_in_state<CmdCtrl>()) {
    ctx_->request_phase(MissionPhase::LANDING);
  } else {
    spdlog::error("Reject LAND: invalid phase {}",
                  fsm_internal::phaseName(ctx_->phase));
  }
}

void ClientCommandHandler::handle_force_hover() {
  if (!ctx_->offboard_state || !ctx_->armed_state) {
    spdlog::error("Reject FORCE_HOVER: require OFFBOARD + ARMED");
    return;
  }
  if (!MissionFSM::is_in_state<Standby>()) {
    ctx_->request_phase(MissionPhase::HOVER);
  } else {
    spdlog::error("Reject FORCE_HOVER: invalid phase {}",
                  fsm_internal::phaseName(ctx_->phase));
  }
}

void ClientCommandHandler::handle_allow_cmd_ctrl() {
  if (!ctx_->offboard_state || !ctx_->armed_state) {
    spdlog::error("Reject ALLOW_CMD_CTRL: require OFFBOARD + ARMED");
    return;
  }
  if (MissionFSM::is_in_state<Hover_State>()) {
    ctx_->request_phase(MissionPhase::CMD_CTRL);
  } else {
    spdlog::error("Reject ALLOW_CMD_CTRL: invalid phase {}",
                  fsm_internal::phaseName(ctx_->phase));
  }
}

void ClientCommandHandler::handle_change_hover_pos(
    const ui::ClientPayload &payload) {
  if (!ctx_->offboard_state || !ctx_->armed_state) {
    spdlog::error("Reject CHANGE_HOVER_POS: require OFFBOARD + ARMED");
    return;
  }
  if (!MissionFSM::is_in_state<Hover_State>()) {
    spdlog::error("Reject CHANGE_HOVER_POS: require HOVER phase");
    return;
  }

  double data[7];
  std::memcpy(data, payload.data, sizeof(data));
  for (const auto &d : data) {
    if (std::isnan(d)) {
      spdlog::error("Reject CHANGE_HOVER_POS: data contains nan");
      return;
    }
  }

  Eigen::Vector3d pos(data[0], data[1], data[2]);
  Eigen::Quaterniond q(data[3], data[4], data[5], data[6]);
  ctx_->hover.pos = pos;
  ctx_->hover.q = q;
  ctx_->hover.initialized = true;
}

void ClientCommandHandler::handle_set_safety_limits(
    const ui::ClientPayload &payload) {
  ui::SafetyLimitsPayload limits{};
  unpack_raw(payload.data, sizeof(limits), limits);
  update_safety_limits(limits);
}

// --- Safety limits ---

bool ClientCommandHandler::validate_safety_limit(double v) const {
  return v == -1.0 || (v > 0.0 && v <= 180.0);
}

void ClientCommandHandler::update_safety_limits(
    const ui::SafetyLimitsPayload &limits) {
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

  params_->guard_params.geofence_min = {limits.geofence_min[0],
                                        limits.geofence_min[1],
                                        limits.geofence_min[2]};
  params_->guard_params.geofence_max = {limits.geofence_max[0],
                                        limits.geofence_max[1],
                                        limits.geofence_max[2]};
  params_->guard_params.max_roll_deg = limits.max_roll_deg;
  params_->guard_params.max_pitch_deg = limits.max_pitch_deg;
  params_->guard_params.max_yaw_deg = limits.max_yaw_deg;
  params_->guard_params.enable_geofence = limits.enable_geofence != 0;
  params_->guard_params.enable_attitude_fence =
      limits.enable_attitude_fence != 0;

  spdlog::info(
      "Applied SET_SAFETY_LIMITS: geofence[{},{},{}]-[{},{},{}], "
      "max_rpy_deg[{},{},{}], en_geo={}, en_att={}",
      limits.geofence_min[0], limits.geofence_min[1], limits.geofence_min[2],
      limits.geofence_max[0], limits.geofence_max[1], limits.geofence_max[2],
      limits.max_roll_deg, limits.max_pitch_deg, limits.max_yaw_deg,
      static_cast<int>(limits.enable_geofence),
      static_cast<int>(limits.enable_attitude_fence));
}

} // namespace px4ctrl
