#pragma once

#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <spdlog/spdlog.h>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

#include "json.hpp"

namespace px4ctrl {
namespace ui {

struct ServerPayload {
  uint8_t id;
  uint64_t timestamp;
  // rigid body
  float pos[3];
  float vel[3];
  float omega[3];
  float quat[4];
  // controller debug msg
  float thrust_setpoint;
  float omega_setpoint[3];
  // quad state
  float battery_voltage;
  // runtime states
  int32_t mission_phase;
  int32_t offboard_state;
  int32_t armed_state;
  float thrust_map[3]; // [thr2acc, hover_thrust_est, applied_thrust]
  float hover_pos[3];
  float hover_quat[4]; // actually only have yaw
  float odom_hz;
  float cmdctrl_hz;

  // extended telemetry (kept at end for compatibility evolution)
  uint32_t telemetry_seq;
  uint32_t guard_flags;
  float odom_age_ms;
  float client_cmd_age_ms;
  float battery_remaining;
  float speed_norm;
  float tilt_deg;
  float roll_deg;
  float pitch_deg;
  float yaw_deg;
  float cmd_age_ms;     // age of external ctrl_command stream
  float omega_min;      // min bodyrate for plot scaling
  float omega_max;      // max bodyrate for plot scaling

  float geofence_min[3];
  float geofence_max[3];
  float max_roll_deg;
  float max_pitch_deg;
  float max_yaw_deg;
  uint8_t enable_geofence;
  uint8_t enable_attitude_fence;
  uint8_t use_rc;
  uint8_t reserved0;
};

enum class ClientCommand : uint32_t {
  HEARTBEAT,
  ARM,
  ENTER_OFFBOARD,
  EXIT_OFFBOARD,
  TAKEOFF,
  LAND,
  FORCE_HOVER,
  ALLOW_CMD_CTRL,
  FORCE_DISARM,
  CHANGE_HOVER_POS,
  SET_SAFETY_LIMITS,
};

const char *const CommandStr[] = {
    "HEARTBEAT",      "ARM",
    "ENTER_OFFBOARD", "EXIT_OFFBOARD",
    "TAKEOFF",        "LAND",
    "FORCE_HOVER",    "ALLOW_CMD_CTRL",
    "FORCE_DISARM",   "CHANGE_HOVER_POS",
    "SET_SAFETY_LIMITS",
};

struct SafetyLimitsPayload {
  float geofence_min[3];
  float geofence_max[3];
  float max_roll_deg;
  float max_pitch_deg;
  float max_yaw_deg;
  uint8_t enable_geofence;
  uint8_t enable_attitude_fence;
  uint8_t reserved[2];
};

struct ClientPayload {
  uint8_t id;
  uint64_t timestamp;
  // command
  ClientCommand command;
  // data
  uint8_t data[64]; // for future use
};

static_assert(std::is_trivially_copyable_v<ServerPayload>,
              "ServerPayload must be trivially copyable for wire transport");
static_assert(std::is_trivially_copyable_v<ClientPayload>,
              "ClientPayload must be trivially copyable for wire transport");
static_assert(sizeof(SafetyLimitsPayload) <= sizeof(ClientPayload::data),
              "SafetyLimitsPayload exceeds client payload data area");
static_assert(sizeof(ClientCommand) == sizeof(uint32_t),
              "ClientCommand wire size must stay 4 bytes");
static_assert(sizeof(ServerPayload) == 240,
              "ServerPayload wire size changed; update client/server together");
static_assert(sizeof(ClientPayload) == 88,
              "ClientPayload wire size changed; update client/server together");

template <typename T> inline void unpack_raw(const uint8_t *data, const size_t size, T &out) {
  if (size != sizeof(T)) {
    throw std::runtime_error("Message size does not match");
  }
  std::memcpy(&out, data, sizeof(T));
}

enum class CommBackend {
  ZENOH,
};

inline CommBackend backendFromString(const std::string &backend) {
  if (backend == "zenoh" || backend == "ZENOH") {
    return CommBackend::ZENOH;
  }
  throw std::runtime_error("Invalid backend: " + backend + " (expected zenoh)");
}

struct TransportParas {
  CommBackend backend = CommBackend::ZENOH;

  // keyexpr/topic names: kept unchanged by default
  std::string server_topic = "px4s";
  std::string client_topic = "px4c";
  std::string log_topic = "px4log";

  uint32_t telemetry_hz = 200;

  // zenoh section
  std::string zenoh_mode = "peer"; // client | peer
  std::string zenoh_connect;        // optional, e.g. tcp/127.0.0.1:7447
  std::string zenoh_listen;         // optional
  bool zenoh_multicast_scouting = true;
  uint32_t zenoh_scouting_timeout_ms = 1000;

  inline static TransportParas load_from_json(const std::string &file) {
    TransportParas paras;
    std::ifstream ifs(file);
    if (!ifs.is_open()) {
      throw std::runtime_error("Failed to open transport config: " + file);
    }

    nlohmann::json cfg;
    ifs >> cfg;

    if (cfg.contains("backend")) {
      paras.backend = backendFromString(cfg.value("backend", std::string("zenoh")));
    }

    paras.server_topic = cfg.value("server_topic", paras.server_topic);
    paras.client_topic = cfg.value("client_topic", paras.client_topic);
    paras.log_topic = cfg.value("log_topic", paras.log_topic);
    paras.telemetry_hz = cfg.value("telemetry_hz", paras.telemetry_hz);

    if (cfg.contains("zenoh")) {
      const auto &z = cfg["zenoh"];
      paras.zenoh_mode = z.value("mode", paras.zenoh_mode);
      paras.zenoh_connect = z.value("connect", paras.zenoh_connect);
      paras.zenoh_listen = z.value("listen", paras.zenoh_listen);
      paras.zenoh_multicast_scouting =
          z.value("multicast_scouting", paras.zenoh_multicast_scouting);
      paras.zenoh_scouting_timeout_ms =
          z.value("scouting_timeout_ms", paras.zenoh_scouting_timeout_ms);
    }

    return paras;
  }

  inline static TransportParas load(const std::string &file) {
    try {
      auto ext = std::filesystem::path(file).extension().string();
      if (ext == ".json") {
        return load_from_json(file);
      }
      throw std::runtime_error("Unsupported transport config format: " + file +
                               " (only .json is supported)");
    } catch (const std::exception &e) {
      spdlog::error("error:{}", e.what());
      throw;
    }
  }
};

} // namespace ui
} // namespace px4ctrl
