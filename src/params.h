#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <ostream>
#include <spdlog/spdlog.h>
#include <sstream>
#include <stdexcept>
#include <string>

#include "json.hpp"

namespace px4ctrl {

namespace params {

enum class ThrustMod { ESTIMATE, THRUSTMAP };

inline ThrustMod thrustModFromString(const std::string &str) {
  if (str == "ESTIMATE") {
    return ThrustMod::ESTIMATE;
  }
  if (str == "THRUSTMAP") {
    return ThrustMod::THRUSTMAP;
  }
  spdlog::error("Invalid ThrustMod type:{}", str);
  throw std::runtime_error(
      "Invalid ThrustMod type, must be ESTIMATE or THRUSTMAP, but got " + str);
}

struct QuadrotorParams {
  double mass;                     // kg
  std::array<double, 3> inertia;   // kg*m^2
  double g;                        // m/s^2
  ThrustMod thrustmod;             // 估计推力 or 推力映射
  std::array<double, 3> thrustmap; // 推力映射 c2,c1,c0
  double init_hover_thrust;        // 初始悬停推力
  double max_thrust;               // 最大推力 无单位 <= 1
  double min_thrust;               // 最小推力 无单位 >= 0
  double max_bodyrate;             // 最大角速度 rad/s
};

enum class Guard { HOLD, LAND, DISARM };

inline Guard guardFromString(const std::string &str) {
  if (str == "HOLD") {
    return Guard::HOLD;
  }
  if (str == "LAND") {
    return Guard::LAND;
  }
  if (str == "DISARM") {
    return Guard::DISARM;
  }
  spdlog::error("Invalid Guard type:{}", str);
  throw std::runtime_error(
      "Invalid Guard type, must be HOLD, LAND or DISARM, but got " + str);
}

struct GuardParams {
  uint32_t freq;
  uint32_t land_timeout; // ms after this time, disarm

  uint32_t mavros_timeout; // ms
  Guard mavros_triggered;

  uint32_t odom_timeout; // ms
  uint32_t odom_min_hz;  // Hz
  Guard odom_triggered;

  uint32_t ui_timeout; // ms
  Guard ui_triggered;

  bool use_rc = false;
  uint32_t rc_timeout = 500; // ms
  Guard rc_triggered = Guard::LAND;

  double low_battery_voltage; // V
  Guard lowvolt_triggered;

  // extended safety controls
  Guard localization_loss_triggered = Guard::LAND;

  bool enable_geofence = false;
  std::array<double, 3> geofence_min = {-1000.0, -1000.0, -1000.0};
  std::array<double, 3> geofence_max = {1000.0, 1000.0, 1000.0};
  Guard geofence_triggered = Guard::HOLD;

  bool enable_attitude_fence = false;
  double max_roll_deg = -1.0;  // -1 means unlimited, otherwise (0, 180]
  double max_pitch_deg = -1.0; // -1 means unlimited, otherwise (0, 180]
  double max_yaw_deg = -1.0;   // -1 means unlimited, otherwise (0, 180]
  Guard attitude_triggered = Guard::HOLD;

  bool enable_velocity_fence = false;
  double max_velocity_norm = 10.0;
  Guard velocity_triggered = Guard::HOLD;
};

struct StateMachineParams {
  uint32_t freq;
  double l2_takeoff_height;            // m
  double l2_idle_disarm_time;          // ms
  double l2_cmd_ctrl_min_hz;           // Hz
  double l2_takeoff_landing_speed;     // m/s
  double l2_land_position_deviation_c; // m
  double l2_land_velocity_thr_c;       // m/s
  double l2_land_time_keep_c;          // ms
};

enum class ControlType {
  BODYRATES,
  ATTITUDE,
  // ROTOR_THRUST not implemented in ROS1
};

inline ControlType controlTypeFromString(const std::string &str) {
  if (str == "BODYRATES") {
    return ControlType::BODYRATES;
  }
  if (str == "ATTITUDE") {
    return ControlType::ATTITUDE;
  }
  spdlog::error("Invalid ControlType type:{}", str);
  throw std::runtime_error(
      "Invalid ControlType type, must be BODYRATES or ATTITUDE, but got " + str);
}

struct ControlParams {
  uint32_t freq;
  double Kp_pos;
  double Kd_pos;
  double Ki_pos;
  double max_pos_error;
  double max_vel_error;
  double max_vel_int;

  ControlType type;
  double Kw_rp;
  double Kw_yaw;
  double max_bodyrate_error;
};

} // namespace params

struct Px4CtrlParams {
  params::QuadrotorParams quadrotor_params;
  params::GuardParams guard_params;
  params::StateMachineParams statemachine_params;
  params::ControlParams control_params;
  uint8_t drone_id = 0;  // 0 = accept any, non-zero = reject mismatched client commands

private:
  template <typename T>
  static T json_required(const nlohmann::json &node, const char *key,
                         const char *section) {
    if (!node.contains(key)) {
      throw std::runtime_error(std::string(section) + " missing key: " + key);
    }
    return node.at(key).get<T>();
  }

  static Px4CtrlParams load_json(const std::string &file) {
    Px4CtrlParams params;

    std::ifstream ifs(file);
    if (!ifs.is_open()) {
      throw std::runtime_error("Failed to open config file: " + file);
    }

    nlohmann::json config;
    ifs >> config;

    if (!config.contains("quadrotor") || !config.contains("guard") ||
        !config.contains("px4ctrl") || !config.contains("controller")) {
      throw std::runtime_error(
          "Config must contain quadrotor/guard/px4ctrl/controller sections");
    }

    const auto &quadrotor = config.at("quadrotor");
    params.quadrotor_params.mass =
        json_required<double>(quadrotor, "mass", "quadrotor");
    params.quadrotor_params.inertia =
        json_required<std::array<double, 3>>(quadrotor, "inertia", "quadrotor");
    params.quadrotor_params.g = json_required<double>(quadrotor, "g", "quadrotor");
    params.quadrotor_params.thrustmod = params::thrustModFromString(
        json_required<std::string>(quadrotor, "thrustmod", "quadrotor"));
    params.quadrotor_params.thrustmap = json_required<std::array<double, 3>>(
        quadrotor, "thrustmap", "quadrotor");
    params.quadrotor_params.init_hover_thrust =
        json_required<double>(quadrotor, "init_hover_thrust", "quadrotor");
    params.quadrotor_params.max_thrust =
        json_required<double>(quadrotor, "max_thrust", "quadrotor");
    params.quadrotor_params.min_thrust =
        json_required<double>(quadrotor, "min_thrust", "quadrotor");
    params.quadrotor_params.max_bodyrate =
        json_required<double>(quadrotor, "max_bodyrate", "quadrotor");

    const auto &guard = config.at("guard");
    params.guard_params.freq = json_required<uint32_t>(guard, "freq", "guard");
    params.guard_params.land_timeout =
        json_required<uint32_t>(guard, "land_timeout", "guard");
    params.guard_params.mavros_timeout =
        json_required<uint32_t>(guard, "mavros_timeout", "guard");
    params.guard_params.mavros_triggered = params::guardFromString(
        json_required<std::string>(guard, "mavros_triggered", "guard"));
    params.guard_params.odom_timeout =
        json_required<uint32_t>(guard, "odom_timeout", "guard");
    params.guard_params.odom_min_hz =
        json_required<uint32_t>(guard, "odom_min_hz", "guard");
    params.guard_params.odom_triggered = params::guardFromString(
        json_required<std::string>(guard, "odom_triggered", "guard"));
    params.guard_params.ui_timeout =
        json_required<uint32_t>(guard, "ui_timeout", "guard");
    params.guard_params.ui_triggered = params::guardFromString(
        json_required<std::string>(guard, "ui_triggered", "guard"));
    params.guard_params.use_rc =
        guard.value("use_rc", params.guard_params.use_rc);
    params.guard_params.rc_timeout =
        guard.value("rc_timeout", params.guard_params.rc_timeout);
    if (guard.contains("rc_triggered")) {
      params.guard_params.rc_triggered =
          params::guardFromString(guard.at("rc_triggered").get<std::string>());
    }
    params.guard_params.low_battery_voltage =
        json_required<double>(guard, "low_battery_voltage", "guard");
    params.guard_params.lowvolt_triggered = params::guardFromString(
        json_required<std::string>(guard, "lowvolt_triggered", "guard"));

    if (guard.contains("localization_loss_triggered")) {
      params.guard_params.localization_loss_triggered = params::guardFromString(
          guard.at("localization_loss_triggered").get<std::string>());
    } else {
      params.guard_params.localization_loss_triggered =
          params.guard_params.odom_triggered;
    }

    params.guard_params.enable_geofence =
        guard.value("enable_geofence", params.guard_params.enable_geofence);
    if (guard.contains("geofence_min")) {
      params.guard_params.geofence_min =
          guard.at("geofence_min").get<std::array<double, 3>>();
    }
    if (guard.contains("geofence_max")) {
      params.guard_params.geofence_max =
          guard.at("geofence_max").get<std::array<double, 3>>();
    }
    if (guard.contains("geofence_triggered")) {
      params.guard_params.geofence_triggered = params::guardFromString(
          guard.at("geofence_triggered").get<std::string>());
    }

    params.guard_params.enable_attitude_fence =
        guard.value("enable_attitude_fence",
                    params.guard_params.enable_attitude_fence);
    if (guard.contains("max_roll_deg")) {
      params.guard_params.max_roll_deg =
          guard.at("max_roll_deg").get<double>();
    }
    if (guard.contains("max_pitch_deg")) {
      params.guard_params.max_pitch_deg =
          guard.at("max_pitch_deg").get<double>();
    }
    if (guard.contains("max_yaw_deg")) {
      params.guard_params.max_yaw_deg = guard.at("max_yaw_deg").get<double>();
    } else if (guard.contains("max_tilt_deg")) {
      // Backward compatibility for old config key.
      const double max_tilt = guard.at("max_tilt_deg").get<double>();
      params.guard_params.max_roll_deg = max_tilt;
      params.guard_params.max_pitch_deg = max_tilt;
      params.guard_params.max_yaw_deg = -1.0;
    }
    if (guard.contains("attitude_triggered")) {
      params.guard_params.attitude_triggered = params::guardFromString(
          guard.at("attitude_triggered").get<std::string>());
    }

    params.guard_params.enable_velocity_fence =
        guard.value("enable_velocity_fence",
                    params.guard_params.enable_velocity_fence);
    params.guard_params.max_velocity_norm =
        guard.value("max_velocity_norm", params.guard_params.max_velocity_norm);
    if (guard.contains("velocity_triggered")) {
      params.guard_params.velocity_triggered = params::guardFromString(
          guard.at("velocity_triggered").get<std::string>());
    }

    const auto &px4ctrl = config.at("px4ctrl");
    params.statemachine_params.freq =
        json_required<uint32_t>(px4ctrl, "freq", "px4ctrl");
    params.statemachine_params.l2_takeoff_height =
        json_required<double>(px4ctrl, "l2_takeoff_height", "px4ctrl");
    params.statemachine_params.l2_idle_disarm_time =
        json_required<double>(px4ctrl, "l2_idle_disarm_time", "px4ctrl");
    params.statemachine_params.l2_cmd_ctrl_min_hz =
        json_required<double>(px4ctrl, "l2_cmd_ctrl_min_hz", "px4ctrl");
    params.statemachine_params.l2_takeoff_landing_speed =
        json_required<double>(px4ctrl, "l2_takeoff_landing_speed", "px4ctrl");
    params.statemachine_params.l2_land_position_deviation_c =
        json_required<double>(px4ctrl, "l2_land_position_deviation_c", "px4ctrl");
    params.statemachine_params.l2_land_velocity_thr_c =
        json_required<double>(px4ctrl, "l2_land_velocity_thr_c", "px4ctrl");
    params.statemachine_params.l2_land_time_keep_c =
        json_required<double>(px4ctrl, "l2_land_time_keep_c", "px4ctrl");
    params.drone_id = px4ctrl.value("drone_id", params.drone_id);

    const auto &control = config.at("controller");
    params.control_params.freq =
        json_required<uint32_t>(control, "freq", "controller");
    params.control_params.Kp_pos =
        json_required<double>(control, "Kp_pos", "controller");
    params.control_params.Kd_pos =
        json_required<double>(control, "Kd_pos", "controller");
    params.control_params.Ki_pos =
        json_required<double>(control, "Ki_pos", "controller");
    params.control_params.max_pos_error =
        json_required<double>(control, "max_pos_error", "controller");
    params.control_params.max_vel_error =
        json_required<double>(control, "max_vel_error", "controller");
    params.control_params.max_vel_int =
        json_required<double>(control, "max_vel_int", "controller");
    params.control_params.type = params::controlTypeFromString(
        json_required<std::string>(control, "type", "controller"));
    params.control_params.Kw_rp =
        json_required<double>(control, "Kw_rp", "controller");
    params.control_params.Kw_yaw =
        json_required<double>(control, "Kw_yaw", "controller");
    params.control_params.max_bodyrate_error =
        json_required<double>(control, "max_bodyrate_error", "controller");

    return params;
  }

public:
  inline static Px4CtrlParams load(const std::string &file) {
    try {
      const auto ext = std::filesystem::path(file).extension().string();
      if (ext == ".json") {
        return load_json(file);
      }
      throw std::runtime_error("Unsupported config format: " + file +
                               " (only .json is supported)");
    } catch (const std::exception &e) {
      spdlog::error("error:{}", e.what());
      throw;
    }
  }

  friend inline std::ostream &operator<<(std::ostream &os,
                                         const Px4CtrlParams &px4paras) {
    os << "QuadrotorParams:" << std::endl;
    os << "mass:" << px4paras.quadrotor_params.mass << std::endl;
    os << "inertia:" << px4paras.quadrotor_params.inertia[0] << " "
       << px4paras.quadrotor_params.inertia[1] << " "
       << px4paras.quadrotor_params.inertia[2] << std::endl;
    os << "g:" << px4paras.quadrotor_params.g << std::endl;
    os << "thrustmod:" << static_cast<int>(px4paras.quadrotor_params.thrustmod)
       << std::endl;
    os << "thrustmap:" << px4paras.quadrotor_params.thrustmap[0] << " "
       << px4paras.quadrotor_params.thrustmap[1] << " "
       << px4paras.quadrotor_params.thrustmap[2] << std::endl;
    os << "init_hover_thrust:" << px4paras.quadrotor_params.init_hover_thrust
       << std::endl;
    os << "max_thrust:" << px4paras.quadrotor_params.max_thrust << std::endl;
    os << "min_thrust:" << px4paras.quadrotor_params.min_thrust << std::endl;
    os << "max_bodyrate:" << px4paras.quadrotor_params.max_bodyrate << std::endl;

    os << "GuardParams:" << std::endl;
    os << "freq:" << px4paras.guard_params.freq << std::endl;
    os << "land_timeout:" << px4paras.guard_params.land_timeout << std::endl;
    os << "mavros_timeout:" << px4paras.guard_params.mavros_timeout << std::endl;
    os << "mavros_triggered:"
       << static_cast<int>(px4paras.guard_params.mavros_triggered) << std::endl;
    os << "odom_timeout:" << px4paras.guard_params.odom_timeout << std::endl;
    os << "odom_triggered:"
       << static_cast<int>(px4paras.guard_params.odom_triggered) << std::endl;
    os << "ui_timeout:" << px4paras.guard_params.ui_timeout << std::endl;
    os << "ui_triggered:" << static_cast<int>(px4paras.guard_params.ui_triggered)
       << std::endl;
    os << "use_rc:" << px4paras.guard_params.use_rc << std::endl;
    os << "rc_timeout:" << px4paras.guard_params.rc_timeout << std::endl;
    os << "rc_triggered:" << static_cast<int>(px4paras.guard_params.rc_triggered)
       << std::endl;
    os << "low_battery_voltage:" << px4paras.guard_params.low_battery_voltage
       << std::endl;
    os << "lowvolt_triggered:"
       << static_cast<int>(px4paras.guard_params.lowvolt_triggered) << std::endl;
    os << "enable_geofence:" << px4paras.guard_params.enable_geofence << std::endl;
    os << "enable_attitude_fence:" << px4paras.guard_params.enable_attitude_fence
       << std::endl;
    os << "max_roll_deg:" << px4paras.guard_params.max_roll_deg << std::endl;
    os << "max_pitch_deg:" << px4paras.guard_params.max_pitch_deg << std::endl;
    os << "max_yaw_deg:" << px4paras.guard_params.max_yaw_deg << std::endl;
    os << "enable_velocity_fence:" << px4paras.guard_params.enable_velocity_fence
       << std::endl;

    os << "Px4CtrlParams:" << std::endl;
    os << "freq:" << px4paras.statemachine_params.freq << std::endl;
    os << "l2_takeoff_height:" << px4paras.statemachine_params.l2_takeoff_height
       << std::endl;
    os << "l2_idle_disarm_time:"
       << px4paras.statemachine_params.l2_idle_disarm_time << std::endl;
    os << "l2_cmd_ctrl_min_hz:"
       << px4paras.statemachine_params.l2_cmd_ctrl_min_hz << std::endl;
    os << "l2_takeoff_landing_speed:"
       << px4paras.statemachine_params.l2_takeoff_landing_speed << std::endl;
    os << "drone_id:" << static_cast<int>(px4paras.drone_id) << std::endl;

    os << "ControlParams:" << std::endl;
    os << "freq:" << px4paras.control_params.freq << std::endl;
    os << "Kp_pos:" << px4paras.control_params.Kp_pos << std::endl;
    os << "Kd_pos:" << px4paras.control_params.Kd_pos << std::endl;
    os << "Ki_pos:" << px4paras.control_params.Ki_pos << std::endl;
    os << "max_pos_error:" << px4paras.control_params.max_pos_error << std::endl;
    os << "max_vel_error:" << px4paras.control_params.max_vel_error << std::endl;
    os << "max_vel_int:" << px4paras.control_params.max_vel_int << std::endl;
    os << "type:" << static_cast<int>(px4paras.control_params.type) << std::endl;
    os << "Kw_rp:" << px4paras.control_params.Kw_rp << std::endl;
    os << "Kw_yaw:" << px4paras.control_params.Kw_yaw << std::endl;
    os << "max_bodyrate_error:" << px4paras.control_params.max_bodyrate_error
       << std::endl;
    return os;
  }

  operator std::string() const {
    std::stringstream ss;
    ss << *this;
    return ss.str();
  }
};

} // namespace px4ctrl
