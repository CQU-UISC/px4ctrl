#include "fsm.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "fsm_internal.h"

namespace px4ctrl {

void Px4Ctrl::client_command_callback(const ui::ClientPayload &payload) {
  const auto recv_time = clock::now();

  if (payload.command != ui::ClientCommand::HEARTBEAT) {
    spdlog::info("client command:{}",
                 ui::CommandStr[static_cast<int>(payload.command)]);
  }

  if (px4ctrl_params->guard_params.use_rc &&
      payload.command != ui::ClientCommand::HEARTBEAT &&
      payload.command != ui::ClientCommand::SET_SAFETY_LIMITS &&
      !has_valid_rc_signal()) {
    spdlog::warn("Reject client command: use_rc enabled but RC signal is invalid");
    last_client_cmd_time = recv_time;
    has_client_cmd = true;
    return;
  }

  switch (payload.command) {
  case ui::ClientCommand::HEARTBEAT:
    break;

  case ui::ClientCommand::ARM:
    if (px4ctrl_params->guard_params.use_rc) {
      spdlog::warn(
          "Reject ARM from client: use_rc enabled, arm via RC/PX4 mode switch");
      break;
    }
    if (!px4_bridge->set_arm(true)) {
      spdlog::error("arm failed");
    }
    se3_controller_->resetThrustMapping();
    break;

  case ui::ClientCommand::FORCE_DISARM:
    if (px4ctrl_params->guard_params.use_rc) {
      spdlog::warn("Reject DISARM from client: use_rc enabled, disarm via RC/PX4 "
                   "mode switch");
      break;
    }
    if (!px4_bridge->force_disarm()) {
      spdlog::error("force disarm failed");
    }
    se3_controller_->resetThrustMapping();
    requested_phase_ = MissionPhase::STANDBY;
    break;

  case ui::ClientCommand::ENTER_OFFBOARD:
    if (px4ctrl_params->guard_params.use_rc) {
      spdlog::warn(
          "Reject ENTER_OFFBOARD from client: use_rc enabled, switch via RC/PX4");
      break;
    }
    if (!px4_bridge->enter_offboard()) {
      spdlog::error("enter offboard failed");
    }
    break;

  case ui::ClientCommand::EXIT_OFFBOARD:
    if (px4ctrl_params->guard_params.use_rc) {
      spdlog::warn(
          "Reject EXIT_OFFBOARD from client: use_rc enabled, switch via RC/PX4");
      break;
    }
    if (!px4_bridge->exit_offboard()) {
      spdlog::error("exit offboard failed");
    }
    break;

  case ui::ClientCommand::TAKEOFF:
    if (!offboard_state_ || !armed_state_) {
      spdlog::error("Reject TAKEOFF: require OFFBOARD + ARMED");
      break;
    }
    if (phase_ == MissionPhase::STANDBY) {
      requested_phase_ = MissionPhase::TAKEOFF;
    } else {
      spdlog::error("Reject TAKEOFF: invalid phase {}",
                    fsm_internal::phaseName(phase_));
    }
    break;

  case ui::ClientCommand::LAND:
    if (!offboard_state_ || !armed_state_) {
      spdlog::error("Reject LAND: require OFFBOARD + ARMED");
      break;
    }
    if (phase_ == MissionPhase::HOVER || phase_ == MissionPhase::CMD_CTRL_READY ||
        phase_ == MissionPhase::CMD_CTRL) {
      requested_phase_ = MissionPhase::LANDING;
    } else {
      spdlog::error("Reject LAND: invalid phase {}",
                    fsm_internal::phaseName(phase_));
    }
    break;

  case ui::ClientCommand::FORCE_HOVER:
    if (!offboard_state_ || !armed_state_) {
      spdlog::error("Reject FORCE_HOVER: require OFFBOARD + ARMED");
      break;
    }
    if (phase_ != MissionPhase::STANDBY && phase_ != MissionPhase::WAIT_FOR_RC) {
      requested_phase_ = MissionPhase::HOVER;
    } else {
      spdlog::error("Reject FORCE_HOVER: invalid phase {}",
                    fsm_internal::phaseName(phase_));
    }
    break;

  case ui::ClientCommand::ALLOW_CMD_CTRL:
    if (!offboard_state_ || !armed_state_) {
      spdlog::error("Reject ALLOW_CMD_CTRL: require OFFBOARD + ARMED");
      break;
    }
    if (phase_ == MissionPhase::HOVER) {
      requested_phase_ = MissionPhase::CMD_CTRL_READY;
    } else {
      spdlog::error("Reject ALLOW_CMD_CTRL: invalid phase {}",
                    fsm_internal::phaseName(phase_));
    }
    break;

  case ui::ClientCommand::CHANGE_HOVER_POS: {
    if (!offboard_state_ || !armed_state_) {
      spdlog::error("Reject CHANGE_HOVER_POS: require OFFBOARD + ARMED");
      break;
    }
    if (phase_ != MissionPhase::HOVER) {
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
    if (reject) {
      break;
    }
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

  last_client_cmd_time = recv_time;
  has_client_cmd = true;
}

ui::ServerPayload Px4Ctrl::fill_server_payload() {
  ui::ServerPayload payload{};
  payload.id = 0;
  const auto now = clock::now();
  payload.timestamp = to_uint64(now);

  auto battery = px4_state->battery->value().first;
  if (battery == nullptr) {
    payload.battery_voltage = 0;
    payload.battery_remaining = -1;
  } else {
    payload.battery_voltage = battery->voltage;
    payload.battery_remaining = battery->percentage;
  }

  payload.mission_phase = static_cast<int32_t>(phase_);
  payload.offboard_state = offboard_state_ ? 1 : 0;
  payload.armed_state = armed_state_ ? 1 : 0;

  payload.hover_pos[0] = hover_.pos.x();
  payload.hover_pos[1] = hover_.pos.y();
  payload.hover_pos[2] = hover_.pos.z();
  payload.hover_quat[0] = hover_.q.w();
  payload.hover_quat[1] = hover_.q.x();
  payload.hover_quat[2] = hover_.q.y();
  payload.hover_quat[3] = hover_.q.z();

  const auto odom_state = px4_state->odom->value();
  auto odom = odom_state.first;
  if (odom == nullptr) {
    payload.pos[0] = 0;
    payload.pos[1] = 0;
    payload.pos[2] = 0;
    payload.vel[0] = 0;
    payload.vel[1] = 0;
    payload.vel[2] = 0;
    payload.omega[0] = 0;
    payload.omega[1] = 0;
    payload.omega[2] = 0;
    payload.quat[0] = 1;
    payload.quat[1] = 0;
    payload.quat[2] = 0;
    payload.quat[3] = 0;
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

  payload.odom_hz = odom_hz;
  payload.cmdctrl_hz = cmdctrl_hz;

  payload.thrust_setpoint = static_cast<float>(last_ctrl_cmd.thrust);
  if (last_ctrl_cmd.type == params::ControlType::BODYRATES) {
    payload.omega_setpoint[0] = static_cast<float>(last_ctrl_cmd.bodyrates.x());
    payload.omega_setpoint[1] = static_cast<float>(last_ctrl_cmd.bodyrates.y());
    payload.omega_setpoint[2] = static_cast<float>(last_ctrl_cmd.bodyrates.z());
  } else {
    payload.omega_setpoint[0] = 0;
    payload.omega_setpoint[1] = 0;
    payload.omega_setpoint[2] = 0;
  }

  payload.thrust_map[0] = static_cast<float>(se3_controller_->getThr2AccEstimate());
  payload.thrust_map[1] =
      static_cast<float>(se3_controller_->getHoverThrustEstimate());
  payload.thrust_map[2] = static_cast<float>(last_ctrl_cmd.thrust);

  payload.telemetry_seq = telemetry_seq++;
  payload.guard_flags = guard_flags;
  payload.odom_age_ms =
      (odom != nullptr)
          ? static_cast<float>(std::max(0.0, timeDuration(odom_state.second, now)))
          : -1.0F;
  payload.client_cmd_age_ms =
      has_client_cmd
          ? static_cast<float>(std::max(0.0, timeDuration(last_client_cmd_time, now)))
          : -1.0F;
  payload.speed_norm = std::sqrt(payload.vel[0] * payload.vel[0] +
                                 payload.vel[1] * payload.vel[1] +
                                 payload.vel[2] * payload.vel[2]);

  const Eigen::Quaterniond q(payload.quat[0], payload.quat[1], payload.quat[2],
                             payload.quat[3]);
  const Eigen::Vector3d body_z = q * Eigen::Vector3d::UnitZ();
  const double cos_tilt =
      std::clamp(body_z.dot(Eigen::Vector3d::UnitZ()), -1.0, 1.0);
  payload.tilt_deg = static_cast<float>(std::acos(cos_tilt) * fsm_internal::kRadToDeg);

  payload.roll_deg = static_cast<float>(roll_deg_);
  payload.pitch_deg = static_cast<float>(pitch_deg_);
  payload.yaw_deg = static_cast<float>(yaw_deg_);

  payload.geofence_min[0] =
      static_cast<float>(px4ctrl_params->guard_params.geofence_min[0]);
  payload.geofence_min[1] =
      static_cast<float>(px4ctrl_params->guard_params.geofence_min[1]);
  payload.geofence_min[2] =
      static_cast<float>(px4ctrl_params->guard_params.geofence_min[2]);
  payload.geofence_max[0] =
      static_cast<float>(px4ctrl_params->guard_params.geofence_max[0]);
  payload.geofence_max[1] =
      static_cast<float>(px4ctrl_params->guard_params.geofence_max[1]);
  payload.geofence_max[2] =
      static_cast<float>(px4ctrl_params->guard_params.geofence_max[2]);
  payload.max_roll_deg = static_cast<float>(px4ctrl_params->guard_params.max_roll_deg);
  payload.max_pitch_deg =
      static_cast<float>(px4ctrl_params->guard_params.max_pitch_deg);
  payload.max_yaw_deg = static_cast<float>(px4ctrl_params->guard_params.max_yaw_deg);
  payload.enable_geofence =
      static_cast<uint8_t>(px4ctrl_params->guard_params.enable_geofence ? 1 : 0);
  payload.enable_attitude_fence = static_cast<uint8_t>(
      px4ctrl_params->guard_params.enable_attitude_fence ? 1 : 0);
  payload.use_rc = static_cast<uint8_t>(px4ctrl_params->guard_params.use_rc ? 1 : 0);

  return payload;
}

std::pair<const Eigen::Vector3d, const Eigen::Quaterniond>
Px4Ctrl::get_hovering_pos() const {
  return {hover_.pos, hover_.q};
}

bool Px4Ctrl::set_hovering_pos(const Eigen::Vector3d &pos,
                               const Eigen::Quaterniond &q) {
  hover_.pos = pos;
  hover_.q = q;
  hover_.initialized = true;
  return true;
}

} // namespace px4ctrl
