#pragma once

#include <memory>
#include <string>

#include "context.h"
#include "controller.h"
#include "mission_fsm.h"
#include "safety_monitor.h"
#include "server.h"
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
  bool essential_ready(const MissionContextSnapshot &snap) const;

  ControlSource select_control_source(const MissionContextSnapshot &snap) const;

  void build_command(const MissionContextSnapshot &snap, ControlSource source,
                     controller::ControlCommand &ctrl_cmd);
  void build_proof_alive(const MissionContextSnapshot &snap,
                         controller::ControlCommand &ctrl_cmd) const;
  bool build_se3_command(const MissionContextSnapshot &snap,
                         controller::ControlCommand &ctrl_cmd);
  bool build_external_command(const MissionContextSnapshot &snap,
                              controller::ControlCommand &ctrl_cmd);
  bool build_safe_landing_command(const MissionContextSnapshot &snap,
                                  controller::ControlCommand &ctrl_cmd);

  void apply_control(const controller::ControlCommand &cmd);
  void estimate_thrust_from_imu();

  void on_client_command(const ui::ClientPayload &payload);
  bool validate_safety_limit(double v) const;
  void update_safety_limits(const ui::SafetyLimitsPayload &limits);
  void handle_phase_request(MissionPhase requested);

  ui::ServerPayload build_telemetry();

  // Components
  std::shared_ptr<Context> ctx_;
  std::shared_ptr<Px4CtrlRosBridge> bridge_;
  std::shared_ptr<ui::Px4Server> server_;
  std::shared_ptr<Px4CtrlParams> params_;
  std::shared_ptr<controller::Se3Control> controller_;
  std::unique_ptr<SafetyMonitor> safety_monitor_;

  // Observer holders (keep alive)
  Px4DataObserver odom_hold_;
  Px4DataObserver ctrl_hold_;
  Px4DataObserver client_hold_;

  // Log suppression
  clock::time_point last_log_state_time_;
  clock::time_point odom_low_log_time_;
  uint32_t odom_low_suppressed_ = 0;
  bool odom_low_active_ = false;

  // State
  bool ok_ = true;
  bool was_failsafe_ = false;
};

} // namespace px4ctrl
