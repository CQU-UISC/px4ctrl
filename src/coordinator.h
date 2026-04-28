#pragma once

#include <memory>

#include "client_handler.h"
#include "command_builder.h"
#include "context.h"
#include "controller.h"
#include "log_suppressor.h"
#include "mission_fsm.h"
#include "safety_monitor.h"
#include "server.h"
#include "telemetry_builder.h"
#include "types.h"

namespace px4ctrl {

class Coordinator {
public:
  Coordinator(std::shared_ptr<Px4CtrlRosBridge> bridge,
              std::shared_ptr<Px4State> px4_state,
              std::shared_ptr<Px4CtrlParams> params,
              std::shared_ptr<ui::Px4Server> server);
  ~Coordinator() = default;

  void run();
  void stop();

  std::pair<const Eigen::Vector3d, const Eigen::Quaterniond>
  get_hovering_pos() const;
  bool set_hovering_pos(const Eigen::Vector3d &pos, const Eigen::Quaterniond &q);

private:
  void process();
  void compute_hz();
  static bool essential_ready(const MissionContextSnapshot &snap);
  void handle_phase_request(MissionPhase requested);
  void apply_control(const controller::ControlCommand &cmd);
  void estimate_thrust_from_imu();

  void update_cached_state(const MissionContextSnapshot &snap);

  // Components
  std::shared_ptr<Context> ctx_;
  std::shared_ptr<Px4CtrlRosBridge> bridge_;
  std::shared_ptr<ui::Px4Server> server_;
  std::shared_ptr<Px4CtrlParams> params_;
  std::shared_ptr<controller::Se3Control> controller_;
  std::unique_ptr<SafetyMonitor> safety_monitor_;
  std::unique_ptr<CommandBuilder> command_builder_;
  std::unique_ptr<ClientCommandHandler> client_handler_;
  std::unique_ptr<TelemetryBuilder> telemetry_builder_;

  // Observer holders
  Px4DataObserver odom_hold_;
  Px4DataObserver ctrl_hold_;
  Px4DataObserver client_hold_;

  // Log suppression
  clock::time_point last_log_state_time_;
  LogSuppressor odom_hz_suppressor_{2000};

  // State
  bool ok_ = true;
};

} // namespace px4ctrl
