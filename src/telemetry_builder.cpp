#include "telemetry_builder.h"

#include <algorithm>
#include <cmath>

#include "fsm_internal.h"

namespace px4ctrl {

TelemetryBuilder::TelemetryBuilder(
    std::shared_ptr<Context> ctx, std::shared_ptr<Px4CtrlParams> params,
    std::shared_ptr<controller::Se3Control> ctrl)
    : ctx_(std::move(ctx)), params_(std::move(params)),
      controller_(std::move(ctrl)) {}

ui::ServerPayload TelemetryBuilder::build() {
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

  const auto cmd_state = ctx_->px4_state()->ctrl_command->value();
  if (cmd_state.first != nullptr) {
    payload.cmd_age_ms =
        static_cast<float>(std::max(0.0, timeDuration(cmd_state.second, now)));
  } else {
    payload.cmd_age_ms = -1.0F;
  }

  const float max_wb = static_cast<float>(params_->quadrotor_params.max_bodyrate);
  payload.omega_min = -max_wb;
  payload.omega_max = max_wb;
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

} // namespace px4ctrl
