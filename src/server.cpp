#include "server.h"

#include <chrono>
#include <cstdio>
#include <mutex>
#include <string>

namespace px4ctrl {
namespace ui {

namespace {
std::string json5_string(const std::string &value) { return "'" + value + "'"; }

std::string json5_singleton_list(const std::string &value) {
  return "['" + value + "']";
}

bool configure_zenoh(const TransportParas &paras, z_owned_config_t &config) {
  if (z_config_default(&config) < 0) {
    return false;
  }

  const std::string mode = (paras.zenoh_mode == "client") ? "client" : "peer";
  if (zc_config_insert_json5(z_loan_mut(config), Z_CONFIG_MODE_KEY,
                             json5_string(mode).c_str()) < 0) {
    return false;
  }

  if (zc_config_insert_json5(z_loan_mut(config), Z_CONFIG_MULTICAST_SCOUTING_KEY,
                             paras.zenoh_multicast_scouting ? "true" : "false") <
      0) {
    return false;
  }

  const auto scouting_timeout = std::to_string(paras.zenoh_scouting_timeout_ms);
  if (zc_config_insert_json5(z_loan_mut(config), Z_CONFIG_SCOUTING_TIMEOUT_KEY,
                             scouting_timeout.c_str()) < 0) {
    return false;
  }

  if (!paras.zenoh_connect.empty()) {
    if (zc_config_insert_json5(z_loan_mut(config), Z_CONFIG_CONNECT_KEY,
                               json5_singleton_list(paras.zenoh_connect).c_str()) <
        0) {
      return false;
    }
  }

  if (!paras.zenoh_listen.empty()) {
    if (zc_config_insert_json5(z_loan_mut(config), Z_CONFIG_LISTEN_KEY,
                               json5_singleton_list(paras.zenoh_listen).c_str()) <
        0) {
      return false;
    }
  }

  return true;
}

void bytes_to_client_payload(const z_loaned_bytes_t *bytes, ClientPayload &payload) {
  const auto size = z_bytes_len(bytes);
  if (size != sizeof(ClientPayload)) {
    throw std::runtime_error("ClientPayload size mismatch");
  }
  z_bytes_reader_t reader = z_bytes_get_reader(bytes);
  const auto copied = z_bytes_reader_read(
      &reader, reinterpret_cast<uint8_t *>(&payload), sizeof(ClientPayload));
  if (copied != sizeof(ClientPayload)) {
    throw std::runtime_error("ClientPayload short read");
  }
}

bool payload_to_bytes(const void *data, const size_t size, z_owned_bytes_t &bytes) {
  return z_bytes_copy_from_buf(&bytes, reinterpret_cast<const uint8_t *>(data),
                               size) >= 0;
}

} // namespace

Px4Server::Px4Server(const TransportParas &paras) : paras_(paras) {
  if (paras_.backend != CommBackend::ZENOH) {
    throw std::runtime_error(
        "Only zenoh backend is supported in this build. Set backend=zenoh.");
  }
  z_internal_null(&session_);
  z_internal_null(&server_pub_);
  z_internal_null(&log_pub_);
  z_internal_null(&client_sub_);

  if (!init_zenoh()) {
    throw std::runtime_error("Px4Server init failed");
  }
}

bool Px4Server::init_zenoh() {
  static std::once_flag zenoh_log_once;
  std::call_once(zenoh_log_once, []() { zc_init_log_from_env_or("error"); });

  z_owned_config_t config;
  z_internal_null(&config);
  if (!configure_zenoh(paras_, config)) {
    spdlog::error("Failed to configure zenoh");
    z_drop(z_move(config));
    return false;
  }

  if (z_open(&session_, z_move(config), nullptr) < 0) {
    spdlog::error("Failed to open zenoh session");
    return false;
  }

  z_view_keyexpr_t server_key;
  if (z_view_keyexpr_from_str(&server_key, paras_.server_topic.c_str()) < 0) {
    spdlog::error("Invalid server topic keyexpr: {}", paras_.server_topic);
    close_zenoh();
    return false;
  }
  if (z_declare_publisher(z_loan(session_), &server_pub_, z_loan(server_key),
                          nullptr) < 0) {
    spdlog::error("Failed to declare server publisher on {}", paras_.server_topic);
    close_zenoh();
    return false;
  }

  z_view_keyexpr_t log_key;
  if (z_view_keyexpr_from_str(&log_key, paras_.log_topic.c_str()) < 0) {
    spdlog::error("Invalid log topic keyexpr: {}", paras_.log_topic);
    close_zenoh();
    return false;
  }
  if (z_declare_publisher(z_loan(session_), &log_pub_, z_loan(log_key), nullptr) <
      0) {
    spdlog::error("Failed to declare log publisher on {}", paras_.log_topic);
    close_zenoh();
    return false;
  }

  z_owned_closure_sample_t closure;
  z_internal_null(&closure);
  z_closure_sample(&closure, Px4Server::client_sample_callback, nullptr, this);

  z_view_keyexpr_t client_key;
  if (z_view_keyexpr_from_str(&client_key, paras_.client_topic.c_str()) < 0) {
    spdlog::error("Invalid client topic keyexpr: {}", paras_.client_topic);
    close_zenoh();
    return false;
  }
  if (z_declare_subscriber(z_loan(session_), &client_sub_, z_loan(client_key),
                           z_move(closure), nullptr) < 0) {
    spdlog::error("Failed to declare client subscriber on {}", paras_.client_topic);
    close_zenoh();
    return false;
  }

  ok_.store(true);
  spdlog::info("Zenoh server ready, pub:{}, sub:{}, log:{}", paras_.server_topic,
               paras_.client_topic, paras_.log_topic);
  return true;
}

void Px4Server::close_zenoh() {
  if (z_internal_check(client_sub_)) {
    (void)z_undeclare_subscriber(z_move(client_sub_));
  }
  if (z_internal_check(log_pub_)) {
    (void)z_undeclare_publisher(z_move(log_pub_));
  }
  if (z_internal_check(server_pub_)) {
    (void)z_undeclare_publisher(z_move(server_pub_));
  }
  if (z_internal_check(session_)) {
    z_drop(z_move(session_));
  }
}

void Px4Server::pub(const ServerPayload &payload) {
  if (!ok_.load()) {
    return;
  }

  z_owned_bytes_t bytes;
  z_internal_null(&bytes);
  if (!payload_to_bytes(&payload, sizeof(ServerPayload), bytes)) {
    spdlog::warn("Failed to serialize server payload");
    return;
  }

  if (z_publisher_put(z_loan(server_pub_), z_move(bytes), nullptr) < 0) {
    spdlog::warn("Failed to publish telemetry");
  }
}

void Px4Server::pub_log(spdlog::level::level_enum level, std::string_view log) {
  if (!ok_.load() || !z_internal_check(log_pub_)) {
    return;
  }

  nlohmann::json log_payload;
  log_payload["level"] = static_cast<int>(level);
  log_payload["text"] = std::string(log);
  const std::string serialized = log_payload.dump();

  z_owned_bytes_t bytes;
  z_internal_null(&bytes);
  if (!payload_to_bytes(serialized.data(), serialized.size(), bytes)) {
    return;
  }
  (void)z_publisher_put(z_loan(log_pub_), z_move(bytes), nullptr);
}

void Px4Server::client_sample_callback(z_loaned_sample_t *sample, void *context) {
  auto *self = reinterpret_cast<Px4Server *>(context);
  if (self == nullptr) {
    return;
  }

  try {
    const auto *payload_bytes = z_sample_payload(sample);
    if (payload_bytes == nullptr) {
      return;
    }
    ClientPayload payload;
    bytes_to_client_payload(payload_bytes, payload);
    self->client_data.post(payload);
  } catch (const std::exception &e) {
    spdlog::warn("Failed to decode client payload: {}", e.what());
  }
}

Px4Server::~Px4Server() {
  ok_.store(false);
  close_zenoh();
  spdlog::info("server closed");
}

template <typename Mutex>
zenoh_sink<Mutex>::zenoh_sink(std::shared_ptr<Px4Server> server)
    : server_(std::move(server)) {}

template <typename Mutex>
void zenoh_sink<Mutex>::sink_it_(const spdlog::details::log_msg &msg) {
  const auto server = server_.lock();
  if (!server) {
    return;
  }

  spdlog::memory_buf_t formatted;
  spdlog::sinks::base_sink<Mutex>::formatter_->format(msg, formatted);

  server->pub_log(msg.level,
                  std::string_view(formatted.data(), formatted.size()));
}

template <typename Mutex> zenoh_sink<Mutex>::~zenoh_sink() = default;

// template instantiations
template class zenoh_sink<std::mutex>;
template class zenoh_sink<spdlog::details::null_mutex>;

} // namespace ui

} // namespace px4ctrl
