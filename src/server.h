#pragma once

#if defined(__linux__) && !defined(ZENOH_LINUX)
#define ZENOH_LINUX 1
#endif

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <spdlog/common.h>
#include <spdlog/sinks/base_sink.h>
#include <sstream>
#include <string>
#include <string_view>
#include <zenoh.h>

#include "datas.h"
#include "types.h"

namespace px4ctrl {
namespace ui {

class Px4Server {
public:
  explicit Px4Server(const TransportParas &paras);
  ~Px4Server();

  void pub(const ServerPayload &payload);
  void pub_log(spdlog::level::level_enum level, std::string_view log);
  Px4Data<ClientPayload> client_data;

private:
  TransportParas paras_;
  z_owned_session_t session_{};
  z_owned_publisher_t server_pub_{};
  z_owned_publisher_t log_pub_{};
  z_owned_subscriber_t client_sub_{};

  std::atomic<bool> ok_{false};

  bool init_zenoh();
  void close_zenoh();
  static void client_sample_callback(z_loaned_sample_t *sample, void *context);
};

template <typename Mutex>
class zenoh_sink : public spdlog::sinks::base_sink<Mutex> {
public:
  explicit zenoh_sink(std::shared_ptr<Px4Server> server);
  ~zenoh_sink() override;

protected:
  void sink_it_(const spdlog::details::log_msg &msg) override;
  void flush_() override {}

private:
  std::weak_ptr<Px4Server> server_;
};

using zenoh_sink_mt = zenoh_sink<std::mutex>;
using zenoh_sink_st = zenoh_sink<spdlog::details::null_mutex>;

} // namespace ui

} // namespace px4ctrl
