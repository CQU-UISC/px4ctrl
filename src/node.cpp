#include <filesystem>
#include <memory>
#include <signal.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <string>

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "bridge.h"
#include "fsm.h"
#include "server.h"

std::shared_ptr<px4ctrl::Px4Ctrl> px4ctrl_fsm;

void sigintHandler(int sig) {
  spdlog::info("[PX4Ctrl] exit...");
  px4ctrl_fsm->stop();
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("px4ctrl");
  std::string base_dir, cfg_name, transport_cfg_name;
  node->declare_parameter("px4ctrl_base_dir", "");
  node->declare_parameter("px4ctrl_cfg_name", "gz500.json");
  node->declare_parameter("px4ctrl_transport_cfg_name", "transport.json");
  node->get_parameter("px4ctrl_base_dir", base_dir);
  node->get_parameter("px4ctrl_cfg_name", cfg_name);
  node->get_parameter("px4ctrl_transport_cfg_name", transport_cfg_name);
  if (transport_cfg_name.empty()) {
    transport_cfg_name = "transport.json";
  }

  // check if cfg exists
  std::string cfg_file = base_dir + "/config/" + cfg_name;
  std::string transport_cfg_file = base_dir + "/config/" + transport_cfg_name;
  if (!std::filesystem::exists(cfg_file)) {
    spdlog::error("px4ctrl config file not found: {}", cfg_file);
    return -1;
  }
  if (!std::filesystem::exists(transport_cfg_file)) {
    spdlog::error("transport config file not found: {}", transport_cfg_file);
    return -1;
  }
  // log filenamez
  std::time_t now =
      std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::string date(30, '\0');
  std::strftime(&date[0], date.size(), "%Y-%m-%d-%H:%M:%S",
                std::localtime(&now));
  std::string log_file = base_dir + "/log/px4ctrl_" + date + ".log";

  // load params
  auto cfg = px4ctrl::Px4CtrlParams::load(cfg_file);
  auto transport_cfg = px4ctrl::ui::TransportParas::load(transport_cfg_file);
  auto px4ctrl_server = std::make_shared<px4ctrl::ui::Px4Server>(transport_cfg);
  auto zenoh_sink =
      std::make_shared<px4ctrl::ui::zenoh_sink_mt>(px4ctrl_server);

  // init logging
  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  auto file_sink =
      std::make_shared<spdlog::sinks::basic_file_sink_mt>(log_file, true);

  auto logger_ptr = std::make_shared<spdlog::logger>(
      "px4ctrl", spdlog::sinks_init_list{console_sink, file_sink, zenoh_sink});
  spdlog::set_default_logger(logger_ptr);

  auto px4ctrl_params = std::make_shared<px4ctrl::Px4CtrlParams>(cfg);
  auto px4_state = std::make_shared<px4ctrl::Px4State>();
  auto px4_bridge =
      std::make_shared<px4ctrl::Px4CtrlRosBridge>(node, px4_state);
  px4ctrl_fsm = std::make_shared<px4ctrl::Px4Ctrl>(
      px4_bridge, px4_state, px4ctrl_params, px4ctrl_server);
  signal(SIGINT, sigintHandler);
  px4ctrl_fsm->run();
  rclcpp::shutdown();
  spdlog::info("[PX4Ctrl] exited");
  return 0;
}
