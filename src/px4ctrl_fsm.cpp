#include "px4ctrl_fsm.h"

#include "json.hpp"
#include "mavros_msgs/ExtendedState.h"
#include "mavros_msgs/State.h"
#include "px4ctrl_gcs.h"
#include "px4ctrl_se3_controller.h"
#include "px4ctrl_state.h"
#include "quadrotor_msgs/CtrlCommand.h"
#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <spdlog/spdlog.h>
#include <thread>

using json = nlohmann::json;

namespace px4ctrl {
Px4Ctrl::Px4Ctrl(std::shared_ptr<PX4CTRL_ROS_BRIDGE> px4_mavros,
                 std::shared_ptr<PX4_STATE> px4_state,
                 std::shared_ptr<gcs::DroneCom> drone_com)
    : Px4Ctrl(std::filesystem::current_path().string(), px4_mavros, px4_state,
              drone_com) {}

Px4Ctrl::Px4Ctrl(std::string base_dir,
                 std::shared_ptr<PX4CTRL_ROS_BRIDGE> px4_mavros,
                 std::shared_ptr<PX4_STATE> px4_state,
                 std::shared_ptr<gcs::DroneCom> drone_com)
    : px4_mavros(px4_mavros), px4_state(px4_state), base_dir(base_dir),
      drone_com(drone_com) {
  spdlog::info("px4ctrl base dir:{}", base_dir);
  assert(px4_mavros != nullptr && px4_state != nullptr);
  if (!init()) {
    spdlog::error("Px4Ctrl init failed");
    exit(EXIT_FAILURE);
  }
}

bool Px4Ctrl::init() {
  // init state
  L0.reset(L0_NON_OFFBOARD);
  L1.reset(L1_UNARMED);
  L2.reset(L2_IDLE);

  // init logging
  // check if log dir exists
  if (!std::filesystem::exists(base_dir + "/log")) {
    std::filesystem::create_directory(base_dir + "/log");
  }

  // load config
  if (!load_config()) {
    return false;
  }

  // init logging
  std::time_t now =
      std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::string date(30, '\0');
  std::strftime(&date[0], date.size(), "%Y-%m-%d-%H:%M:%S",
                std::localtime(&now));

  console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
      base_dir + "/log/px4ctrl_" + date + ".log", true);
  logger_ptr = std::make_shared<spdlog::logger>(
      "px4ctrl", spdlog::sinks_init_list{console_sink, file_sink});
  spdlog::set_default_logger(logger_ptr);

  // init controller
  controller =
      std::make_shared<controller::LinearControl>(params.control_params);

  gcs::reset(gcs_message);
  gcs::reset(drone_message);
  return true;
}

bool Px4Ctrl::load_config() {
  std::ifstream config_file(base_dir + "/px4ctrl.json");
  nlohmann::json json_file;
  if (!config_file.is_open()) {
    spdlog::error("Failed to open config file");
    return false;
  }
  config_file >> json_file;
  spdlog::info("Config file loaded");
  config_file.close();
  /*
      struct Px4CtrlParams{
        uint freq;
        double l2_takeoff_height;
        double l2_idle_disarm_time;//在这个时间内没有收到用户命令将会disarm,
    seconds double
    l2_allow_cmd_ctrl_allow_time;//cmd在这个时间内到达将会进入cmd_ctrl,milliseconds
        double
    l2_cmd_ctrl_cmd_timeout;//cmd_ctrl在这个时间内没有收到命令将会进入hovering,milliseconds
        double l2_takeoff_landing_speed;//m/s

        double max_thrust;
        double min_thrust;
        double max_bodyrate;
        controller::ControlParams control_params;
        uint8_t drone_id,gcs_id,version;//load from config
    };
  */
  if (!(json_file.contains("freq") && json_file.contains("l2_takeoff_height") &&
        json_file.contains("l2_idle_disarm_time") &&
        json_file.contains("l2_allow_cmd_ctrl_allow_time") &&
        json_file.contains("l2_cmd_ctrl_cmd_timeout") &&
        json_file.contains("l2_takeoff_landing_speed") &&
        json_file.contains("max_thrust") && json_file.contains("min_thrust") &&
        json_file.contains("max_bodyrate") &&
        json_file.contains("control_params") &&
        json_file.contains("drone_id") && json_file.contains("gcs_id") &&
        json_file.contains("version") && json_file.contains("guard_params"))) {
    spdlog::error("Config file missing key");
    return false;
  }
  params.freq = json_file["freq"].template get<uint>();
  params.l2_takeoff_height =
      json_file["l2_takeoff_height"].template get<double>();
  params.l2_idle_disarm_time =
      json_file["l2_idle_disarm_time"].template get<double>();
  params.l2_allow_cmd_ctrl_allow_time =
      json_file["l2_allow_cmd_ctrl_allow_time"].template get<double>();
  params.l2_cmd_ctrl_cmd_timeout =
      json_file["l2_cmd_ctrl_cmd_timeout"].template get<double>();
  params.l2_takeoff_landing_speed =
      json_file["l2_takeoff_landing_speed"].template get<double>();
  params.max_thrust = json_file["max_thrust"].template get<double>();
  params.min_thrust = json_file["min_thrust"].template get<double>();
  params.max_bodyrate = json_file["max_bodyrate"].template get<double>();
  params.drone_id = json_file["drone_id"].template get<uint8_t>();
  params.gcs_id = json_file["gcs_id"].template get<uint8_t>();
  params.version = json_file["version"].template get<uint8_t>();
  // control_params
  /*
  struct ControlParams{
    double mass;
    double g;
    double Kp,Kv,Ka,Kw;
    double hover_percentage;
};
 */
  if (!(json_file["control_params"].contains("g") &&
        json_file["control_params"].contains("hover_percentage") &&
        json_file["control_params"].contains("mass") &&
        json_file["control_params"].contains("Kp") &&
        json_file["control_params"].contains("Kv") &&
        json_file["control_params"].contains("Ka") &&
        json_file["control_params"].contains("Kw") &&
        json_file["control_params"].contains("bodyrates_control"))) {
    spdlog::error("Config file missing key:control_params");
    return false;
  }
  params.control_params.g =
      json_file["control_params"]["g"].template get<double>();
  params.control_params.hover_percentage =
      json_file["control_params"]["hover_percentage"].template get<double>();
  params.control_params.mass =
      json_file["control_params"]["mass"].template get<double>();
  params.control_params.Kp =
      json_file["control_params"]["Kp"].template get<double>();
  params.control_params.Kv =
      json_file["control_params"]["Kv"].template get<double>();
  params.control_params.Ka =
      json_file["control_params"]["Ka"].template get<double>();
  params.control_params.Kw =
      json_file["control_params"]["Kw"].template get<double>();
  params.control_params.bodyrates_control =
      json_file["control_params"]["bodyrates_control"].template get<bool>();
  if (!(json_file["guard_params"].contains("mavros_timeout") &&
        json_file["guard_params"].contains("odom_timeout") &&
        json_file["guard_params"].contains("gcs_timeout") &&
        json_file["guard_params"].contains("low_battery_voltage"))) {
    spdlog::error("Config file missing key:guard_params");
    return false;
  }
  params.guard_params.mavros_timeout =
      json_file["guard_params"]["mavros_timeout"].template get<double>();
  params.guard_params.odom_timeout =
      json_file["guard_params"]["odom_timeout"].template get<double>();
  params.guard_params.gcs_timeout =
      json_file["guard_params"]["gcs_timeout"].template get<double>();
  params.guard_params.low_battery_voltage =
      json_file["guard_params"]["low_battery_voltage"].template get<double>();

  // print config
  spdlog::info("Config loaded:{}", json_file.dump());
  return true;
}

GuardStatus Px4Ctrl::guard() {
  clock::time_point now = clock::now();
  std::chrono::milliseconds mavros_state =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          now - px4_state->state.second);
  std::chrono::milliseconds odom_state =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          now - px4_state->odom.second);
  std::chrono::milliseconds gcs_state =
      std::chrono::duration_cast<std::chrono::milliseconds>(now -
                                                            last_gcs_time);
  // check gcs
  if (gcs_state.count() > params.guard_params.gcs_timeout) {
    return GuardStatus::GCS_TIMEOUT;
  }
  // check mavros
  if (mavros_state.count() > params.guard_params.mavros_timeout) {
    return GuardStatus::MAVROS_TIMEOUT;
  }
  // check odom
  if (odom_state.count() > params.guard_params.odom_timeout) {
    return GuardStatus::ODOM_TIMEOUT;
  }
  if (px4_state->battery.first->voltage <
      params.guard_params.low_battery_voltage) {
    return GuardStatus::LOW_VOLTAGE;
  }
  return GuardStatus::OK;
}

void Px4Ctrl::stop() { ok = false; }

void Px4Ctrl::run() {
  int delta_t = int(1000 / params.freq);
  while (ok) {
    process();
    std::this_thread::sleep_for(std::chrono::milliseconds(delta_t));
  }
}

void Px4Ctrl::apply_control(const controller::ControlCommand &cmd) {
  double thrust = std::clamp(cmd.thrust, params.min_thrust, params.max_thrust);
  switch (cmd.type) {
  case controller::ControlType::BODY_RATES: {
    Eigen::Vector3d bodyrates = cmd.bodyrates.cwiseMax(-params.max_bodyrate)
                                    .cwiseMin(params.max_bodyrate);
    std::array<double, 3> bodyrates_arr = {bodyrates.x(), bodyrates.y(),
                                           bodyrates.z()};
    px4_mavros->pub_bodyrates_target(thrust, bodyrates_arr);
    break;
  }
  case controller::ControlType::ATTITUDE: {
    const std::array<double, 4> quat = {cmd.attitude.w(), cmd.attitude.x(),
                                        cmd.attitude.y(), cmd.attitude.z()};
    px4_mavros->pub_attitude_target(thrust, quat);
    break;
  }
  }
  return;
}

void Px4Ctrl::process() {
  // process ros message
  px4_mavros->spin_once();

  // if not recv px4, return
  if (clock::now() - px4_state->state.second > std::chrono::seconds(1)) {
    logger_ptr->info("Waiting for Px4 message");
    return;
  }

  // check for other message except cmd_ctrl
  if (px4_state->state.first == nullptr ||
      px4_state->ext_state.first == nullptr ||
      px4_state->odom.first == nullptr || px4_state->imu.first == nullptr ||
      px4_state->battery.first == nullptr) {
    logger_ptr->info("Waiting for All message");
    if (px4_state->state.first == nullptr) {
      logger_ptr->info("state message is nullptr");
    }
    if (px4_state->ext_state.first == nullptr) {
      logger_ptr->info("ext_state message is nullptr");
    }
    if (px4_state->odom.first == nullptr) {
      logger_ptr->info("odom message is nullptr");
    }
    if (px4_state->imu.first == nullptr) {
      logger_ptr->info("imu message is nullptr");
    }
    if (px4_state->battery.first == nullptr) {
      logger_ptr->info("battery message is nullptr");
    }
    return;
  }

  // recv
  gcs::reset(gcs_message);
  bool recv_gcs = drone_com->receive(gcs_message);
  if (recv_gcs) {
    last_gcs_time = clock::now();
  }

  if (init_com) {
    // 如果没有消息到达，直接return
    if (!recv_gcs) {
      logger_ptr->info("Waiting for GCS message");
      return;
    } else {
      logger_ptr->info("GCS message arrive, begin...");
      init_com = false;
    }
  }

  /*
  Description
  Offboard mode is used for controlling vehicle movement and attitude,
  by setting position, velocity, acceleration, attitude, attitude rates or
  thrust/torque setpoints.

  PX4 must receive a stream of MAVLink setpoint messages or the ROS 2
  OffboardControlMode at 2 Hz as proof that the external controller is healthy.
  The stream must be sent for at least a second before PX4 will arm in offboard
  mode, or switch to offboard mode when flying. If the rate falls below 2Hz
  while under external control PX4 will switch out of offboard mode after a
  timeout (COM_OF_LOSS_T), and attempt to land or perform some other failsafe
  action. The action depends on whether or not RC control is available, and is
  defined in the parameter COM_OBL_RC_ACT.

  When using MAVLink the setpoint messages convey both the signal to indicate
  that the external source is "alive", and the setpoint value itself. In order
  to hold position in this case the vehicle must receive a stream of setpoints
  for the current position.

  When using ROS 2 the proof that the external source is alive is provided by a
  stream of OffboardControlMode messages, while the actual setpoint is provided
  by publishing to one of the setpoint uORB topics, such as TrajectorySetpoint.
  In order to hold position in this case the vehicle must receive a stream of
  OffboardControlMode but would only need the TrajectorySetpoint once.

  Note that offboard mode only supports a very limited set of MAVLink commands
  and messages. Operations, like taking off, landing, return to launch, may be
  best handled using the appropriate modes. Operations like uploading,
  downloading missions can be performed in any mode.
*/

  controller::ControlCommand ctrl_cmd;
  switch (guard()) {
  // 什么都不做1s后就会被接管
  case GuardStatus::GCS_TIMEOUT:
    // Try land
    logger_ptr->error("GCS timeout, try land");
    px4_mavros->set_mode(mavros_msgs::State::MODE_PX4_LAND);
    break;
  case GuardStatus::MAVROS_TIMEOUT:
    // Try land&&kill
    logger_ptr->error("Mavros timeout, try land");
    px4_mavros->set_mode(mavros_msgs::State::MODE_PX4_LAND);
    // px4_mavros->set_arm(false);
    break;
  case GuardStatus::ODOM_TIMEOUT:
    logger_ptr->error("Odom timeout, try land");
    px4_mavros->set_mode(mavros_msgs::State::MODE_PX4_LAND);
    break;
  case GuardStatus::LOW_VOLTAGE:
    logger_ptr->error("Low voltage, try land");
    px4_mavros->set_mode(mavros_msgs::State::MODE_PX4_LAND);
    break;
  case GuardStatus::OK:
    // process state and apply control
    process_l0(ctrl_cmd);
    // control
    apply_control(ctrl_cmd);
    break;
  }

  //
  if (last_log_state_time.time_since_epoch().count() == 0) {
    last_log_state_time = clock::now();
  }
  if (std::chrono::duration_cast<std::chrono::seconds>(clock::now() -
                                                       last_log_state_time)
          .count() > 1) {
    last_log_state_time = clock::now();
    logger_ptr->info("L0:{},L1:{},L2:{}", state_map(L0.state),
                     state_map(L1.state), state_map(L2.state));
  }
  // send
  fill_drone_message();
  drone_com->send(drone_message);
}

void Px4Ctrl::fill_drone_message() {
  // uint32_t id;//mesage id
  // uint8_t version;
  // uint8_t gcs_id;
  // uint8_t drone_id;
  // uint8_t px4ctrl_status[3];
  // float pos[3];
  // float vel[3];
  // float acc[3];
  // float omega[3];
  // float quat[4];
  // float battery;

  ++drone_message.id;
  drone_message.version = params.version;
  drone_message.drone_id = params.drone_id;
  drone_message.gcs_id = params.gcs_id;
  drone_message.px4ctrl_status[0] = L0.state;
  drone_message.px4ctrl_status[1] = L1.state;
  drone_message.px4ctrl_status[2] = L2.state;
  const auto &position = px4_state->odom.first->pose.pose.position;
  const auto &vel = px4_state->odom.first->twist.twist.linear;
  drone_message.pos[0] = position.x;
  drone_message.pos[1] = position.y;
  drone_message.pos[2] = position.z;

  drone_message.vel[0] = vel.x;
  drone_message.vel[1] = vel.y;
  drone_message.vel[2] = vel.z;

  const auto &acc = px4_state->imu.first->linear_acceleration;
  drone_message.acc[0] = acc.x;
  drone_message.acc[1] = acc.y;
  drone_message.acc[2] = acc.z;

  const auto &omega = px4_state->odom.first->twist.twist.angular;
  drone_message.omega[0] = omega.x;
  drone_message.omega[1] = omega.y;
  drone_message.omega[2] = omega.z;

  const auto &quat = px4_state->odom.first->pose.pose.orientation;
  drone_message.quat[0] = quat.w;
  drone_message.quat[1] = quat.x;
  drone_message.quat[2] = quat.y;
  drone_message.quat[3] = quat.z;

  drone_message.battery = px4_state->battery.first->voltage;
}

void Px4Ctrl::proof_alive(controller::ControlCommand &ctrl_cmd) {
  // proof alive
  const auto &quat = px4_state->imu.first->orientation;
  ctrl_cmd.type = controller::ControlType::ATTITUDE;
  ctrl_cmd.attitude = Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z);
  ctrl_cmd.thrust = 0.01;
  return;
}

void Px4Ctrl::process_l0(controller::ControlCommand &ctrl_cmd) {
  // communication level

  // check if connected
  if (L0.state < L0_NON_OFFBOARD || L0.state >= L0_L1) {
    logger_ptr->error("Invalid L0 state, reset to NOT_CONNECTED");
    L0.reset(L0_NON_OFFBOARD);
    return;
  }

  switch (L0.state) {
  case L0_NON_OFFBOARD: {
    proof_alive(ctrl_cmd);
    break;
  }

  case L0_OFFBOARD: {
    process_l1(ctrl_cmd);
    break;
  }

  default: {
    logger_ptr->error("Unexcepted L0 state: {}", state_map(L0.state));
    break;
  }
  }

  // user input
  if (gcs_message.id != 0 && gcs_message.drone_id == params.drone_id) {
    if (gcs_message.drone_cmd == gcs::command::ENTER_OFFBOARD) {
      px4_mavros->set_mode(mavros_msgs::State::MODE_PX4_OFFBOARD);
    }
    if (gcs_message.drone_cmd == gcs::command::EXIT_OFFBOARD) {
      px4_mavros->set_mode(mavros_msgs::State::MODE_PX4_LAND);
    }
  }

  // update state
  if (px4_state->state.first->mode == mavros_msgs::State::MODE_PX4_OFFBOARD) {
    L0 = L0_OFFBOARD;
  } else {
    L0 = L0_NON_OFFBOARD;
  }

  if (L0.next_state != L0.state) {
    logger_ptr->info("L0 state change:{}->{}", state_map(L0.state),
                     state_map(L0.next_state));
  }
  L0.step(); // defer assignment
  return;
}

void Px4Ctrl::process_l1(controller::ControlCommand &ctrl_cmd) {
  // motor level
  if (L1.state <= L0_L1 || L1.state >= L1_L2) {
    logger_ptr->error("Invalid L1 state, reset to UNARMED");
    L1.reset(L1_UNARMED);
    return;
  }

  switch (L1.state) {
  case L1_UNARMED: {
    proof_alive(ctrl_cmd);
    break;
  }
  case L1_ARMED: {
    process_l2(ctrl_cmd);
    break;
  }
  default: {
    logger_ptr->error("Unexcepted L1 state:{}", state_map(L1.state));
    break;
  }
  }
  // 处理用户的输入
  if (gcs_message.id != 0 && gcs_message.drone_id == params.drone_id) {
    if (gcs_message.drone_cmd == gcs::command::ARM) {
      if (!px4_mavros->set_arm(true)) {
        logger_ptr->error("Failed to arm");
      }
    }
    if (gcs_message.drone_cmd == gcs::command::DISARM) {
      if (!px4_mavros->set_arm(false)) {
        logger_ptr->error("Failed to disarm");
      }
    }
    if( gcs_message.drone_cmd==gcs::command::FORCE_DISARM){
      if(!px4_mavros->force_disarm()){
        logger_ptr->error("Failed to force disarm");
      }
    }
  }

  // update state
  bool armed = px4_state->state.first->armed;
  if (armed) {
    L1 = L1_ARMED;
  } else {
    L1 = L1_UNARMED;
    // reset L2
    L2.reset(L2_IDLE);
    L2.idle.is_first_time = true;
  }

  if (L1.next_state != L1.state) {
    logger_ptr->info("L1 state change:{}->{}", state_map(L1.state),
                     state_map(L1.next_state));
  }

  L1.step();
}

void Px4Ctrl::process_l2(controller::ControlCommand &ctrl_cmd) {
  // task level, 输出控制命令
  if (L2.state <= L1_L2 || L2.state >= END) {
    logger_ptr->error("Invalid L2 state, reset to IDLE");
    L2 = L2_IDLE;
    return;
  }

  auto estimate_thrust = [this]() {
    Eigen::Vector3d est_a(px4_state->imu.first->linear_acceleration.x,
                          px4_state->imu.first->linear_acceleration.y,
                          px4_state->imu.first->linear_acceleration.z);
    controller->estimateThrustModel(est_a, px4_state->imu.second);
  };

  switch (L2.state) {
  case L2_IDLE: { // default
    if (L2.idle.is_first_time) {
      L2.idle.last_arm_time = clock::now();
      L2.idle.is_first_time = false;
    }
    if (std::chrono::duration_cast<std::chrono::seconds>(clock::now() -
                                                         L2.idle.last_arm_time)
            .count() > params.l2_idle_disarm_time) {
      if (!px4_mavros->set_arm(false)) {
        logger_ptr->error("Failed to disarm");
      }
      L2.idle.is_first_time = true;
    }
    ctrl_cmd.type = controller::ControlType::BODY_RATES;
    ctrl_cmd.thrust = 0.1;
    ctrl_cmd.bodyrates = Eigen::Vector3d(0, 0, 0);
    break;
  }

  case L2_TAKING_OFF: {
    if (L2.last_state != L2_TAKING_OFF) {
      const auto &pose = px4_state->odom.first->pose.pose;
      auto &pos = pose.position;
      auto &quat = pose.orientation;
      L2.takingoff.start_pos = Eigen::Vector3d(pos.x, pos.y, pos.z);
      L2.takingoff.start_q = Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z);
      logger_ptr->info("Taking off from:{} {} {}", L2.takingoff.start_pos.x(),
                       L2.takingoff.start_pos.y(), L2.takingoff.start_pos.z());
      logger_ptr->info("Taking off to:{} {} {}", L2.takingoff.start_pos.x(),
                       L2.takingoff.start_pos.y(),
                       L2.takingoff.start_pos.z() + params.l2_takeoff_height);
    } else {
      // check if taking off finished
      const auto &pose = px4_state->odom.first->pose.pose;
      const Eigen::Vector3d cur_pos(pose.position.x, pose.position.y,
                                    pose.position.z);

      const Eigen::Vector3d des_pos(
          L2.takingoff.start_pos.x(), L2.takingoff.start_pos.y(),
          L2.takingoff.start_pos.z() + params.l2_takeoff_height);

      if ((cur_pos - des_pos).norm() < 0.1) {
        L2 = L2_HOVERING;
        logger_ptr->info("Taking off finished");
        break;
      }
    }
    controller::DesiredState des;
    des.p = L2.takingoff.start_pos;
    des.q = L2.takingoff.start_q;
    des.p.z() += params.l2_takeoff_height;
    des.v = Eigen::Vector3d(0, 0, params.l2_takeoff_landing_speed);
    des.yaw = controller->fromQuaternion2yaw(des.q);

    ctrl_cmd = controller->calculateControl(des, *px4_state->odom.first,
                                            *px4_state->imu.first);
    break;
  }

  case L2_HOVERING: {
    // keep hovering
    // 如果在Hovering的情况下ForceHovering，状态不会发生改变
    // 如果在其他模式下ForceHovering，状态会发生改变
    if (L2.last_state != L2_HOVERING) {
      const auto &pose = px4_state->odom.first->pose.pose;
      auto &pos = pose.position;
      auto &quat = pose.orientation;
      L2.hovering.des_pos = Eigen::Vector3d(pos.x, pos.y, pos.z);
      L2.hovering.des_q = Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z);
      logger_ptr->info("Hovering at->X:{} Y:{} Z:{}", pos.x, pos.y, pos.z);
    }

    controller::DesiredState des;
    des.p = L2.hovering.des_pos;
    des.q = L2.hovering.des_q;
    des.yaw = controller->fromQuaternion2yaw(des.q);
    ctrl_cmd = controller->calculateControl(des, *px4_state->odom.first,
                                            *px4_state->imu.first);
    estimate_thrust();

    break;
  }

  case L2_LANDING: {
    if (L2.last_state != L2_LANDING) {
      const auto &pose = px4_state->odom.first->pose.pose;
      auto &pos = pose.position;
      auto &quat = pose.orientation;
      L2.landing.start_pos = Eigen::Vector3d(pos.x, pos.y, pos.z);
      L2.landing.start_q = Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z);
      L2.landing.start_time = clock::now();
      L2.landing.is_first_time = true;
    }
    Eigen::Vector3d vel =
        Eigen::Vector3d(px4_state->odom.first->twist.twist.linear.x,
                        px4_state->odom.first->twist.twist.linear.y,
                        px4_state->odom.first->twist.twist.linear.z);
    controller::DesiredState des;
    des.p = L2.landing.start_pos;
    des.q = L2.landing.start_q;
    des.v = Eigen::Vector3d(0, 0, -params.l2_takeoff_landing_speed);
    des.yaw = controller->fromQuaternion2yaw(des.q);
    des.p.z() -= params.l2_takeoff_landing_speed *
                 std::chrono::duration_cast<std::chrono::seconds>(
                     clock::now() - L2.landing.start_time)
                     .count();

    ctrl_cmd = controller->calculateControl(des, *px4_state->odom.first,
                                            *px4_state->imu.first);
    bool landed = false;

    // land_detector parameters
    const double POSITION_DEVIATION_C =
        -0.5; // Constraint 1: target position below real position for
              // POSITION_DEVIATION_C meters.
    const double VELOCITY_THR_C =
        0.3; // Constraint 2: velocity below VELOCITY_MIN_C m/s.
    const double TIME_KEEP_C =
        3.0; // Constraint 3: Time(s) the Constraint 1&2 need to keep.
    bool C12_satisfy =
        (des.p(2) - px4_state->odom.first->pose.pose.position.z) < POSITION_DEVIATION_C &&
        vel.norm() < VELOCITY_THR_C;
    if (C12_satisfy) {
      if (L2.landing.is_first_time) {
        L2.landing.time_C12_reached = clock::now();
        L2.landing.is_first_time = false;
      }
      if (std::chrono::duration_cast<std::chrono::seconds>(
              clock::now() - L2.landing.time_C12_reached)
              .count() > TIME_KEEP_C) // Constraint 3 reached
      {
        logger_ptr->info("Successfully landed");
        landed = true;
      }
    }

    logger_ptr->info("Desired position: {},{},{}", des.p(0), des.p(1), des.p(2));
    logger_ptr->info("Current position: {},{},{}", px4_state->odom.first->pose.pose.position.x,
                 px4_state->odom.first->pose.pose.position.y,
                 px4_state->odom.first->pose.pose.position.z);
    logger_ptr->info("Landing: C12_satisfy:{}, landed:{}, Landed_state:{}",
    C12_satisfy, landed,
    px4_state->ext_state.first->landed_state);

    if (px4_state->ext_state.first->landed_state ==mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND
    &&landed==true) {
      L2 = L2_IDLE;
    }

    break;
  }

  case L2_ALLOW_CMD_CTRL: {
    // keep hovering, until cmd received
    if (std::chrono::duration_cast<std::chrono::milliseconds>(
            clock::now() - px4_state->ctrl_command.second)
            .count() < params.l2_allow_cmd_ctrl_allow_time) {
      // 如果在20ms内收到了命令，进入CMD_CTRL
      L2 = L2_CMD_CTRL;
      break;
    }

    if (L2.last_state != L2_ALLOW_CMD_CTRL) {
      const auto &pose = px4_state->odom.first->pose.pose;
      auto &pos = pose.position;
      auto &quat = pose.orientation;
      L2.allow_cmd_ctrl.hovering_pos = Eigen::Vector3d(pos.x, pos.y, pos.z);
      L2.allow_cmd_ctrl.hovering_q =
          Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z);
    }

    controller::DesiredState des;
    des.p = L2.hovering.des_pos;
    des.q = L2.hovering.des_q;
    des.yaw = controller->fromQuaternion2yaw(des.q);
    ctrl_cmd = controller->calculateControl(des, *px4_state->odom.first,
                                            *px4_state->imu.first);
    estimate_thrust();
    break;
  }

  case L2_CMD_CTRL: {
    // check if cmd is timed out
    if (std::chrono::duration_cast<std::chrono::milliseconds>(
            clock::now() - px4_state->ctrl_command.second)
            .count() > params.l2_cmd_ctrl_cmd_timeout) {
      L2 = L2_HOVERING;
      break;
    }
    switch (px4_state->ctrl_command.first->type) {
    case quadrotor_msgs::CtrlCommand::ROTORS_FORCE: {
      logger_ptr->error("not supported type:ROTORS_FORCE");
      L2 = L2_HOVERING;
      break;
    }
    case quadrotor_msgs::CtrlCommand::THRUST_BODYRATE: {
      ctrl_cmd.type = controller::ControlType::BODY_RATES;
      ctrl_cmd.thrust =
          controller->thrustMap(px4_state->ctrl_command.first->u[0]);
      ctrl_cmd.bodyrates = Eigen::Vector3d(px4_state->ctrl_command.first->u[1],
                                           px4_state->ctrl_command.first->u[2],
                                           px4_state->ctrl_command.first->u[3]);
      break;
    }
    case quadrotor_msgs::CtrlCommand::THRUST_TORQUE: {
      logger_ptr->error("not supported type:THRUST_TORQUE");
      break;
    }
    }
    break;
  }

  default: {
    logger_ptr->error("Unexcepted L2 state:{}", state_map(L2.state));
    break;
  }
  }

  // 处理用户的输入
  if (gcs_message.id != 0 && gcs_message.drone_id == params.drone_id) {
    switch (gcs_message.drone_cmd) {

    case gcs::command::TAKEOFF: {
      if (L2.state == L2_IDLE) {
        L2 = L2_TAKING_OFF;
      } else {
        logger_ptr->error("Reject! beacuse can not transfer state from {}==>{}",
                          state_map(L2.state), state_map(L2_TAKING_OFF));
      }
      break;
    }

    case gcs::command::LAND: {
      if (L2.state == L2_HOVERING) {
        L2 = L2_LANDING;
      } else {
        logger_ptr->error("Reject! beacuse can not transfer state from {}==>{}",
                          state_map(L2.state), state_map(L2_LANDING));
      }
      break;
    }

    case gcs::command::FORCE_HOVER: {
      L2 = L2_HOVERING;
      break;
    }
    case gcs::command::ALLOW_CMD_CTRL: {
      if (L2.state == L2_HOVERING) {
        L2 = L2_ALLOW_CMD_CTRL;
      } else {
        logger_ptr->error("Reject! beacuse can not transfer state from {}==>{}",
                          state_map(L2.state), state_map(L2_ALLOW_CMD_CTRL));
      }
      break;
    }
    default: {
      break;
    }
    }
  }
  if (L2.next_state != L2.state) {
    logger_ptr->info("L2 state change:{}->{}", state_map(L2.state),
                     state_map(L2.next_state));
  }
  L2.step(); // 记录这一轮的状态，下一轮的状态，以及上一轮的状态
}

} // namespace px4ctrl