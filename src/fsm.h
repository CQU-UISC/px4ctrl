#pragma once

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include <spdlog/logger.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include "bridge.h"
#include "controller.h"
#include "datas.h"
#include "server.h"
#include "types.h"

namespace px4ctrl {

class Px4Ctrl {
public:
  Px4Ctrl(std::shared_ptr<Px4CtrlRosBridge> px4_bridge,
          std::shared_ptr<Px4State> px4_state,
          std::shared_ptr<Px4CtrlParams> px4ctrl_params,
          std::shared_ptr<ui::Px4Server> px4_server);
  ~Px4Ctrl() = default;

  enum class MissionPhase {
    WAIT_FOR_RC,
    STANDBY,
    TAKEOFF,
    HOVER,
    CMD_CTRL_READY,
    CMD_CTRL,
    LANDING,
    FAILSAFE,
  };

  enum class ControlSource {
    PROOF_ALIVE,
    SE3,
    SAFE_LANDING,
    EXTERNAL_CMD,
  };

  void run();
  void stop();

  std::pair<const Eigen::Vector3d, const Eigen::Quaterniond>
  get_hovering_pos() const;
  bool set_hovering_pos(const Eigen::Vector3d &pos,
                        const Eigen::Quaterniond &q);

private:
  struct MissionContext {
    clock::time_point now;

    mavros_msgs::msg::State::ConstSharedPtr state_msg;
    mavros_msgs::msg::ExtendedState::ConstSharedPtr ext_state_msg;
    nav_msgs::msg::Odometry::ConstSharedPtr odom_msg;
    sensor_msgs::msg::Imu::ConstSharedPtr imu_msg;
    sensor_msgs::msg::BatteryState::ConstSharedPtr battery_msg;
    px4ctrl_msgs::msg::Command::ConstSharedPtr cmd_msg;

    double state_age_ms = 0.0;
    double odom_age_ms = 0.0;
    double battery_age_ms = 0.0;
    double imu_age_ms = 0.0;
    double cmd_age_ms = 0.0;

    bool offboard = false;
    bool armed = false;
    bool odom_fresh = false;
    bool cmd_fresh = false;
    bool rc_valid = false;
  };

  struct GuardDecision {
    bool triggered = false;
    bool rc_required_block = false;
    params::Guard action = params::Guard::HOLD;
    uint32_t flags = 0;
    std::string reason;
  };

  struct TakeoffProfile {
    Eigen::Vector3d start_pos = Eigen::Vector3d::Zero();
    Eigen::Quaterniond start_q = Eigen::Quaterniond::Identity();
    clock::time_point start_time;
    bool initialized = false;
  };

  struct HoverTarget {
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    bool initialized = false;
  };

  struct LandingProfile {
    Eigen::Vector3d start_pos = Eigen::Vector3d::Zero();
    Eigen::Quaterniond start_q = Eigen::Quaterniond::Identity();
    clock::time_point start_time;
    clock::time_point c12_reached_time;
    bool initialized = false;
    bool c12_started = false;
  };

  bool ok = true;
  bool init();

  void process();
  void compute_hz();
  MissionContext build_context() const;

  void guard(const MissionContext &ctx, GuardDecision &decision);
  MissionPhase evaluate_phase(const MissionContext &ctx,
                              const GuardDecision &decision);
  ControlSource select_control_source(const MissionContext &ctx,
                                      MissionPhase phase,
                                      const GuardDecision &decision) const;

  void build_command(const MissionContext &ctx, MissionPhase phase,
                     ControlSource source,
                     controller::ControlCommand &ctrl_cmd);
  void build_proof_alive(const MissionContext &ctx,
                         controller::ControlCommand &ctrl_cmd) const;
  bool build_se3_command(const MissionContext &ctx, MissionPhase phase,
                         controller::ControlCommand &ctrl_cmd);
  bool build_external_command(const MissionContext &ctx,
                              controller::ControlCommand &ctrl_cmd);
  bool build_safe_landing_command(const MissionContext &ctx,
                                  controller::ControlCommand &ctrl_cmd);

  void on_phase_enter(MissionPhase phase, const MissionContext &ctx,
                      const GuardDecision &decision);
  bool check_takeoff_finished(const MissionContext &ctx) const;
  bool check_landing_finished(const MissionContext &ctx);

  void estimate_thrust_from_imu();
  bool has_valid_rc_signal() const;
  bool validate_safety_limit(double v) const;
  void update_safety_limits(const ui::SafetyLimitsPayload &limits);
  void apply_control(const controller::ControlCommand &cmd);

  std::shared_ptr<Px4CtrlRosBridge> px4_bridge;
  std::shared_ptr<Px4State> px4_state;
  std::shared_ptr<ui::Px4Server> px4_server;
  std::shared_ptr<Px4CtrlParams> px4ctrl_params;

  std::shared_ptr<controller::Se3Control> se3_controller_;

  MissionPhase phase_ = MissionPhase::STANDBY;
  MissionPhase last_phase_ = MissionPhase::STANDBY;
  clock::time_point phase_enter_time_;
  std::optional<MissionPhase> requested_phase_;

  bool offboard_state_ = false;
  bool armed_state_ = false;
  double roll_deg_ = 0.0;
  double pitch_deg_ = 0.0;
  double yaw_deg_ = 0.0;

  TakeoffProfile takeoff_;
  HoverTarget hover_;
  LandingProfile landing_;

  params::Guard active_guard_action_ = params::Guard::HOLD;

  bool allow_cmdctrl_pub_state_ = false;
  bool has_client_cmd = false;
  uint32_t guard_flags = 0;
  uint32_t telemetry_seq = 0;

  clock::time_point last_client_cmd_time;
  clock::time_point last_log_state_time;
  clock::time_point last_guard_log_time;
  clock::time_point last_guard_repeat_log_time_;
  params::Guard last_guard_action_ = params::Guard::HOLD;
  std::string last_guard_reason_;
  uint32_t last_guard_flags_ = 0;
  uint32_t guard_repeat_suppressed_ = 0;
  bool has_last_guard_ = false;

  ui::ServerPayload fill_server_payload();
  void client_command_callback(const ui::ClientPayload &payload);
  controller::ControlCommand last_ctrl_cmd{};

  int odom_count = 0;
  int cmdctrl_count = 0;
  int odom_hz = 0;
  int cmdctrl_hz = 0;
  clock::time_point odom_last_time;
  clock::time_point cmdctrl_last_time;
  clock::time_point odom_low_log_time_;
  uint32_t odom_low_suppressed_ = 0;
  bool odom_low_active_ = false;
  Px4DataObserver odom_hold;
  Px4DataObserver ctrl_hold;
  Px4DataObserver client_hold;
};

} // namespace px4ctrl
