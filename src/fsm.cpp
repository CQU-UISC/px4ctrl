#include "fsm.h"
#include "bridge.h"
#include "controller.h"
#include "datas.h"
#include "params.h"
#include "types.h"

#include <mavros_msgs/msg/state.hpp>
#include <px4ctrl_msgs/msg/command.hpp>
#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <memory>
#include <spdlog/spdlog.h>
#include <thread>

namespace px4ctrl {
Px4Ctrl::Px4Ctrl(
                std::shared_ptr<Px4CtrlRosBridge> px4_bridge,
                std::shared_ptr<Px4State> px4_state,
                std::shared_ptr<Px4CtrlParams> px4ctrl_params,
                std::shared_ptr<ui::Px4Server> px4_server)
  : px4_bridge(px4_bridge), px4_state(px4_state),px4_server(px4_server),px4ctrl_params(px4ctrl_params){
  assert(px4_bridge != nullptr && px4_state != nullptr && px4ctrl_params != nullptr);
  if (!init()) {
    spdlog::error("Px4Ctrl init failed");
    throw std::runtime_error("Px4Ctrl init failed");
  }
  controller = std::make_shared<controller::Se3Control>(px4ctrl_params->control_params,px4ctrl_params->quadrotor_params);
}

bool Px4Ctrl::init() {
  // init state
  L0.reset(L0_NON_OFFBOARD);
  L1.reset(L1_UNARMED);
  L2.reset(L2_IDLE);

  // init controller
  return true;
}

void Px4Ctrl::stop() { ok = false; }

void Px4Ctrl::run() {
  int delta_t = int(1000 / px4ctrl_params->statemachine_params.freq);
  odom_last_time = clock::now();
  cmdctrl_last_time = clock::now();
  // last_client_cmd_time = clock::now();
  // register
  odom_hold = px4_state->odom->observe([&](auto &odom) {
    odom_count++;
    return;
  });

  ctrl_hold = px4_state->ctrl_command->observe([&](auto &cmd) {
    // TODO now just pub control command and let fsm decide which should be accept
    if(L2.state==L2_CMD_CTRL){
        // direct apply control
        // do cmd conversation
        controller::ControlCommand ctrl_cmd;
        ctrl_cmd.source = controller::ControlSource::CMD;
        switch (px4_state->ctrl_command->value().first->type) {
          case px4ctrl_msgs::msg::Command::ROTORS_FORCE: {
            // TODO
            spdlog::error("not supported type:ROTORS_FORCE");
            break;
          }
          case px4ctrl_msgs::msg::Command::THRUST_BODYRATE: {
            ctrl_cmd.type = controller::ControlType::BODY_RATES;
            ctrl_cmd.thrust =
                controller->thrustMap(px4_state->ctrl_command->value().first->u[0]);
            ctrl_cmd.bodyrates = Eigen::Vector3d(px4_state->ctrl_command->value().first->u[1],
                                                px4_state->ctrl_command->value().first->u[2],
                                                px4_state->ctrl_command->value().first->u[3]);
            break;
          }
          case px4ctrl_msgs::msg::Command::THRUST_TORQUE: {
            // TODO
            ctrl_cmd.type = controller::ControlType::TORQUE;
            ctrl_cmd.thrust =
                controller->thrustMap(px4_state->ctrl_command->value().first->u[0]);
            ctrl_cmd.torques = Eigen::Vector3d(px4_state->ctrl_command->value().first->u[1],
                                                px4_state->ctrl_command->value().first->u[2],
                                                px4_state->ctrl_command->value().first->u[3]);
            break;
          }
          case px4ctrl_msgs::msg::Command::DESIRED_POS: {
            controller::DesiredState des;
            auto des_pos = px4_state->ctrl_command->value().first->pos;
            auto des_vel = px4_state->ctrl_command->value().first->vel;
            auto des_acc = px4_state->ctrl_command->value().first->acc;
            auto des_jerk = px4_state->ctrl_command->value().first->jerk;
            auto des_yaw = px4_state->ctrl_command->value().first->yaw;
            auto des_quat = px4_state->ctrl_command->value().first->quat;
            des.p = Eigen::Vector3d(des_pos[0],des_pos[1],des_pos[2]);
            des.v = Eigen::Vector3d(des_vel[0],des_vel[1],des_vel[2]);
            des.a = Eigen::Vector3d(des_acc[0],des_acc[1],des_acc[2]);
            des.j = Eigen::Vector3d(des_jerk[0],des_jerk[1],des_jerk[2]);
            des.q = Eigen::Quaterniond(des_quat[0],des_quat[1],des_quat[2],des_quat[3]); //w,x,y,z
            des.yaw = des_yaw;
            ctrl_cmd = controller->runControl(des, *px4_state->odom->value().first,
                                                    *px4_state->imu->value().first);
            break;
          }
          case px4ctrl_msgs::msg::Command::THRUST_QUAT:{
            ctrl_cmd.type = controller::ControlType::ATTITUDE;
            auto des_quat = px4_state->ctrl_command->value().first->quat;
            ctrl_cmd.thrust = controller->thrustMap(px4_state->ctrl_command->value().first->u[0]);
            ctrl_cmd.attitude =  Eigen::Quaterniond(des_quat[0],des_quat[1],des_quat[2],des_quat[3]); //w,x,y,z
            break;
          }
      }
      apply_control(ctrl_cmd,controller::ControlSource::CMD);
    }
    cmdctrl_count++; 
    return;
  });

  client_hold = px4_server->client_data.observe([&](auto &cmd) {
    spdlog::info("FSM ClientCmd received");
    client_command_callback(cmd);
    return;
  });

  while (ok) {
    // process ros message
    px4_bridge->spin_once();
    compute_hz();
    if(!guard()){
      process();
    }
    px4_server->pub(fill_server_payload());
    std::this_thread::sleep_for(std::chrono::milliseconds(delta_t));
  }
}

bool Px4Ctrl::guard(){
    //TODO update
    if(!false){
      // not trigger
      return false;
    }

    // check if imu data too old (not recent, imu should run at least 100hz)
    controller::ControlCommand ctrl_cmd;
    constexpr int min_dt = 1000/100; //ms
    auto tp = timePassed(px4_state->imu->value().second);

    // if data too old
    if(tp>min_dt){
      spdlog::error("IMU data too old, thrust map estimate not avaliable");
    }
    ctrl_cmd = controller->runSafeControl(
      *px4_state->imu->value().first
    );
    estimate_thrust();
    apply_control(ctrl_cmd, controller::ControlSource::SAFETY);
    return true;
}

void Px4Ctrl::compute_hz() {
  //TODO using sliding window
  // compute hz
  if(timePassed(odom_last_time) > 100){
    odom_hz = odom_count*10;
    odom_count = 0;
    odom_last_time = clock::now();
  }
  if(timePassed(cmdctrl_last_time)>100){
    cmdctrl_hz = cmdctrl_count * 10;
    cmdctrl_count = 0;
    cmdctrl_last_time = clock::now();
  }
}

void Px4Ctrl::client_command_callback(const ui::ClientPayload &payload){
    auto px4ctrl_fsm_state = get_px4_state();
    spdlog::info("client command:{}", ui::CommandStr[static_cast<int>(payload.command)]);
    switch (payload.command)
    {
      case ui::ClientCommand::HEARTBEAT:
        break;
      case ui::ClientCommand::ARM:
        if (!px4_bridge->set_arm(true))
        {
            spdlog::error("arm failed");
        }
        break;
      case ui::ClientCommand::FORCE_DISARM:
        if (!px4_bridge->force_disarm())
        {
            spdlog::error("force disarm failed");
        }
        // if success then reset thrust mapping
        controller->resetThrustMapping();
        break;
      case ui::ClientCommand::ENTER_OFFBOARD:
        if (!px4_bridge->enter_offboard())
          {
              spdlog::error("enter offboard failed");
          }
        break;
      case ui::ClientCommand::EXIT_OFFBOARD:
        if (!px4_bridge->exit_offboard())
        {
            spdlog::error("exit offboard failed");
        }
        break;
      case ui::ClientCommand::TAKEOFF:
        if(!(px4ctrl_fsm_state[0]==Px4CtrlState::L0_OFFBOARD&&px4ctrl_fsm_state[1]==Px4CtrlState::L1_ARMED))
        {
            spdlog::error("Reject! beacuse L0!=OFFBOARD or L1!=ARMED, L0:{},L1:{}",state_map(px4ctrl_fsm_state[0]),state_map(px4ctrl_fsm_state[1]));
            break;
        } 
        if (px4ctrl_fsm_state[2] == Px4CtrlState::L2_IDLE)
        {
            force_l2_state(Px4CtrlState::L2_TAKING_OFF);
        }
        else
        {
            spdlog::error("Reject! beacuse can not transfer state from {}==>{}",
                          state_map(px4ctrl_fsm_state[2]),
                          state_map(Px4CtrlState::L2_TAKING_OFF));
        }
        break;
      case ui::ClientCommand::LAND:
        if(!(px4ctrl_fsm_state[0]==Px4CtrlState::L0_OFFBOARD&&px4ctrl_fsm_state[1]==Px4CtrlState::L1_ARMED))
        {
            spdlog::error("Reject! beacuse L0!=OFFBOARD or L1!=ARMED, L0:{},L1:{}",state_map(px4ctrl_fsm_state[0]),state_map(px4ctrl_fsm_state[1]));
            break;
        } 
        if (px4ctrl_fsm_state[2] == Px4CtrlState::L2_HOVERING)
        {
            force_l2_state(Px4CtrlState::L2_LANDING);
        }
        else
        {
            spdlog::error("Reject! beacuse can not transfer state from {}==>{}",
                          state_map(px4ctrl_fsm_state[2]),
                          state_map(Px4CtrlState::L2_LANDING));
        }
        break;
      case ui::ClientCommand::FORCE_HOVER:
        if(!(px4ctrl_fsm_state[0]==Px4CtrlState::L0_OFFBOARD&&px4ctrl_fsm_state[1]==Px4CtrlState::L1_ARMED))
        {
            spdlog::error("Reject! beacuse L0!=OFFBOARD or L1!=ARMED, L0:{},L1:{}",state_map(px4ctrl_fsm_state[0]),state_map(px4ctrl_fsm_state[1]));
            break;
        } 
        if (px4ctrl_fsm_state[2] != Px4CtrlState::L2_IDLE)
        {
            force_l2_state(Px4CtrlState::L2_HOVERING);
            L2.last_state = Px4CtrlState::L2_ALLOW_CMD_CTRL;//force update pos
            px4_bridge->pub_allow_cmdctrl(false);
        }
        else
        {
            spdlog::error("Reject! beacuse can not transfer state from {}==>{}",
                          state_map(px4ctrl_fsm_state[2]),
                          state_map(Px4CtrlState::L2_HOVERING));
        }
        break;
      case ui::ClientCommand::ALLOW_CMD_CTRL:
        if(!(px4ctrl_fsm_state[0]==Px4CtrlState::L0_OFFBOARD&&px4ctrl_fsm_state[1]==Px4CtrlState::L1_ARMED))
        {
            spdlog::error("Reject! beacuse L0!=OFFBOARD or L1!=ARMED, L0:{},L1:{}",state_map(px4ctrl_fsm_state[0]),state_map(px4ctrl_fsm_state[1]));
            break;
        } 
        if (px4ctrl_fsm_state[2] == Px4CtrlState::L2_HOVERING)
        {
            force_l2_state(Px4CtrlState::L2_ALLOW_CMD_CTRL);
            px4_bridge->pub_allow_cmdctrl(true);
        }
        else
        {
            spdlog::error("Reject! beacuse can not transfer state from {}==>{}",
                          state_map(px4ctrl_fsm_state[2]),
                          state_map(Px4CtrlState::L2_ALLOW_CMD_CTRL));
        }
        break;
      case ui::ClientCommand::CHANGE_HOVER_POS:{
        if(!(px4ctrl_fsm_state[0]==Px4CtrlState::L0_OFFBOARD&&px4ctrl_fsm_state[1]==Px4CtrlState::L1_ARMED))
        {
            spdlog::error("Reject! beacuse L0!=OFFBOARD or L1!=ARMED, L0:{},L1:{}",state_map(px4ctrl_fsm_state[0]),state_map(px4ctrl_fsm_state[1]));
            break;
        } 
        if(px4ctrl_fsm_state[2]!=Px4CtrlState::L2_HOVERING){
            spdlog::error("Reject! beacuse L2!=HOVERING, L2:{}",state_map(px4ctrl_fsm_state[2]));
            break;
        }
        double data[7];
        std::memcpy(data,payload.data,sizeof(data));
        bool reject = false;
        for(auto& d:data){
            if(std::isnan(d)){
                reject = true;
                spdlog::error("Reject! beacuse data contains nan");
                break;
            }
        }
        if(reject){
            break;
        }
        Eigen::Vector3d pos(data[0],data[1],data[2]);
        Eigen::Quaterniond q(data[3],data[4],data[5],data[6]);
        set_hovering_pos(pos,q);
        break;
      }
      case ui::ClientCommand::RESTART_FCU:{
        if(!(px4ctrl_fsm_state[1]==Px4CtrlState::L1_UNARMED))
        {
            spdlog::error("Reject! beacuse L1!=UNARMED,L1:{}",state_map(px4ctrl_fsm_state[1]));
            break;
        } 
        if (px4_bridge->restart_fcu())
        {
            spdlog::info("restart fcu success");
        }
        else
        {
            spdlog::error("restart fcu failed");
        }
        break;
      }
    }

    last_client_cmd_time = from_uint64(payload.timestamp);
}

ui::ServerPayload Px4Ctrl::fill_server_payload(){
  ui::ServerPayload payload;
  payload.id = 0;//TODO save as the real id
  payload.timestamp = to_uint64(clock::now());
  auto px4 = px4_state->state->value().first;
  auto battery = px4_state->battery->value().first;
  if (battery == nullptr) {
      payload.battery_voltage = 0;
  }else{
      payload.battery_voltage = battery->voltage;
  }
  payload.fsm_state[0] = L0.state;
  payload.fsm_state[1] = L1.state;
  payload.fsm_state[2] = L2.state;
  payload.hover_pos[0] = L2hovering.des_pos.x();
  payload.hover_pos[1] = L2hovering.des_pos.y();
  payload.hover_pos[2] = L2hovering.des_pos.z();
  payload.hover_quat[0] = L2hovering.des_q.w();
  payload.hover_quat[1] = L2hovering.des_q.x();
  payload.hover_quat[2] = L2hovering.des_q.y();
  payload.hover_quat[3] = L2hovering.des_q.z();
  auto odom = px4_state->odom->value().first;
  if (odom == nullptr) {
    payload.pos[0] = 0;
    payload.pos[1] = 0;
    payload.pos[2] = 0;
    payload.vel[0] = 0;
    payload.vel[1] = 0;
    payload.vel[2] = 0;
    payload.omega[0] = 0;
    payload.omega[1] = 0;
    payload.omega[2] = 0;
    payload.quat[0] = 1;
    payload.quat[1] = 0;
    payload.quat[2] = 0;
    payload.quat[3] = 0;
  }else{
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
  payload.odom_hz = odom_hz;
  payload.cmdctrl_hz = cmdctrl_hz;
  return payload;
}

void Px4Ctrl::apply_control(const controller::ControlCommand &cmd, const controller::ControlSource des_source) {
  if(cmd.source != des_source){
    spdlog::error("Control command source not match");
    return;
  }
  switch (L2.state) {
    case L2_CMD_CTRL:{
      if (cmd.source!=controller::ControlSource::CMD){
        spdlog::debug("L2 state is CMD_CTRL, but control command source is not CMD");
        return;
      }
      break;
    }
    default:
      if(cmd.source==controller::ControlSource::CMD){
        spdlog::debug("L2 state is not CMD_CTRL, but control command source is CMD");
        return;
      }
      break;
  }

  double thrust = std::clamp(cmd.thrust, px4ctrl_params->quadrotor_params.min_thrust, px4ctrl_params->quadrotor_params.max_thrust);
  switch (cmd.type) {
    case controller::ControlType::BODY_RATES: {
      Eigen::Vector3d bodyrates = cmd.bodyrates.cwiseMax(-px4ctrl_params->quadrotor_params.max_bodyrate)
                                      .cwiseMin(px4ctrl_params->quadrotor_params.max_bodyrate);
      px4_bridge->pub_bodyrates_target(thrust, bodyrates);
      break;
    }
    case controller::ControlType::ATTITUDE: {
      px4_bridge->pub_attitude_target(thrust, cmd.attitude);
      break;
    }
    case controller::ControlType::TORQUE:{
      px4_bridge->pub_torque_target(thrust, cmd.torques);
      break;
    }
    case controller::ControlType::ROTOR_THRUST:{
      px4_bridge->pub_actuator_target(cmd.rotors_thrust);
      break;
    }
  }
  return;
}

void Px4Ctrl::process() {
  // check for other message except cmd_ctrl
  if (px4_state->state->value().first == nullptr ||
      px4_state->ext_state->value().first == nullptr ||
      px4_state->odom->value().first == nullptr || 
      px4_state->imu->value().first == nullptr ||
      px4_state->battery->value().first == nullptr){
        if (timePassedSeconds(last_log_state_time)>2) {
          spdlog::info("Waiting for All message");
          if (px4_state->state->value().first == nullptr) {
            spdlog::info("state message is nullptr");
          }
          if (px4_state->ext_state->value().first == nullptr) {
            spdlog::info("ext_state message is nullptr");
          }
          if (px4_state->odom->value().first == nullptr) {
            spdlog::info("odom message is nullptr");
          }
          if (px4_state->imu->value().first == nullptr) {
            spdlog::info("imu message is nullptr");
          }
          if (px4_state->battery->value().first == nullptr) {
            spdlog::info("battery message is nullptr");
          }
          last_log_state_time = clock::now();
        }
      return;
  }
  controller::ControlCommand ctrl_cmd;
  ctrl_cmd.source = controller::ControlSource::SE3;
  process_l0(ctrl_cmd);
  // control
  apply_control(ctrl_cmd,controller::ControlSource::SE3);
  //
  if (last_log_state_time.time_since_epoch().count() == 0) {
    last_log_state_time = clock::now();
  }
  if (timePassed(last_log_state_time) > 1000) {
    last_log_state_time = clock::now();
    spdlog::debug("L0:{},L1:{},L2:{}", state_map(L0.state),state_map(L1.state), state_map(L2.state));
  }
}

void Px4Ctrl::proof_alive(controller::ControlCommand &ctrl_cmd) {
  // proof alive
  const auto &quat = px4_state->imu->value().first->orientation;
  ctrl_cmd.source = controller::ControlSource::SE3;
  ctrl_cmd.type = controller::ControlType::ATTITUDE;
  ctrl_cmd.attitude = Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z);
  ctrl_cmd.thrust = 0.01;
  return;
}

void Px4Ctrl::process_l0(controller::ControlCommand &ctrl_cmd) {
  // communication level

  // check if connected
  if (L0.state < L0_NON_OFFBOARD || L0.state >= L0_L1) {
    spdlog::error("Invalid L0 state, reset to NOT_CONNECTED");
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
    spdlog::error("Unexcepted L0 state: {}", state_map(L0.state));
    break;
  }
  }

  // update state
  if (px4_state->state->value().first->mode == mavros_msgs::msg::State::MODE_PX4_OFFBOARD) {
    L0 = L0_OFFBOARD;
  } else {
    L0 = L0_NON_OFFBOARD;
  }

  if (L0.next_state != L0.state) {
    spdlog::info("L0 state change:{}->{}", state_map(L0.state), state_map(L0.next_state));
  }
  L0.step(); // defer assignment
  return;
}

void Px4Ctrl::process_l1(controller::ControlCommand &ctrl_cmd) {
  // motor level
  if (L1.state <= L0_L1 || L1.state >= L1_L2) {
    spdlog::error("Invalid L1 state, reset to UNARMED");
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
    spdlog::error("Unexcepted L1 state:{}", state_map(L1.state));
    break;
  }
  }

  // update state
  bool armed = px4_state->state->value().first->armed;
  if (armed) {
    L1 = L1_ARMED;
  } else {
    L1 = L1_UNARMED;
    // reset L2
    L2.reset(L2_IDLE);
    L2idle.is_first_time = true;
  }

  if (L1.next_state != L1.state) {
    spdlog::info("L1 state change:{}->{}", state_map(L1.state),state_map(L1.next_state));
  }

  L1.step();
}

void Px4Ctrl::estimate_thrust(){
    Eigen::Vector3d est_a(px4_state->imu->value().first->linear_acceleration.x,
                          px4_state->imu->value().first->linear_acceleration.y,
                          px4_state->imu->value().first->linear_acceleration.z);
    controller->estimateThrustModel(est_a, px4_state->imu->value().second);
}

void Px4Ctrl::process_l2(controller::ControlCommand &ctrl_cmd) {
  // task level, 输出控制命令
  if (L2.state <= L1_L2 || L2.state >= END) {
    spdlog::error("Invalid L2 state, reset to IDLE");
    L2 = L2_IDLE;
    return;
  }

  switch (L2.state) {
  case L2_IDLE: { // default
    if (L2idle.is_first_time) {
      L2idle.last_arm_time = clock::now();
      L2idle.is_first_time = false;
    }
    if (timePassed(L2idle.last_arm_time)  > px4ctrl_params->statemachine_params.l2_idle_disarm_time) {
      // if (!px4_bridge->set_arm(false)) {
      //   spdlog::error("Failed to disarm");
      // }
      if(!px4_bridge->force_disarm()){
        spdlog::error("L2_IDLE:Failed to force disarm");
      }
      L2idle.is_first_time = true;
    }
    ctrl_cmd.type = controller::ControlType::BODY_RATES;
    ctrl_cmd.thrust = 0.1;
    ctrl_cmd.bodyrates = Eigen::Vector3d(0, 0, 0);
    break;
  }

  case L2_TAKING_OFF: {
    if (L2.last_state != L2_TAKING_OFF) {
      const auto &pose = px4_state->odom->value().first->pose.pose;
      auto &pos = pose.position;
      auto &quat = pose.orientation;
      L2takingoff.start_pos = Eigen::Vector3d(pos.x, pos.y, pos.z);
      L2takingoff.start_q = Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z);
      L2takingoff.last_takeoff_time = clock::now();
      spdlog::info("Taking off from:{} {} {}", L2takingoff.start_pos.x(),L2takingoff.start_pos.y(), L2takingoff.start_pos.z());
      spdlog::info("Taking off to:{} {} {}", L2takingoff.start_pos.x(),L2takingoff.start_pos.y(),L2takingoff.start_pos.z() + px4ctrl_params->statemachine_params.l2_takeoff_height);
    } else {
      // check if taking off finished
      const auto &pose = px4_state->odom->value().first->pose.pose;
      const Eigen::Vector3d cur_pos(pose.position.x, pose.position.y,
                                    pose.position.z);

      const Eigen::Vector3d des_pos(
          L2takingoff.start_pos.x(), L2takingoff.start_pos.y(),
          L2takingoff.start_pos.z() + px4ctrl_params->statemachine_params.l2_takeoff_height);

      if ((cur_pos - des_pos).norm() < 0.1) {
        L2 = L2_HOVERING;
        spdlog::info("Taking off finished");
        break;
      }
    }
    
    const Eigen::Vector3d des_pos(
          L2takingoff.start_pos.x(), L2takingoff.start_pos.y(),
          L2takingoff.start_pos.z() + px4ctrl_params->statemachine_params.l2_takeoff_height);

    controller::DesiredState des;
    des.p = L2takingoff.start_pos;
    des.q = L2takingoff.start_q;
    des.v = Eigen::Vector3d(0, 0, px4ctrl_params->statemachine_params.l2_takeoff_landing_speed);
    des.yaw = controller::yawFromQuat(des.q);
    des.p.z()+=px4ctrl_params->statemachine_params.l2_takeoff_landing_speed*timePassedSeconds(L2takingoff.last_takeoff_time);
    if(des.p.z()>des_pos.z()){
      des.p.z() = des_pos.z();
      des.v = Eigen::Vector3d(0,0,0);
    }
    ctrl_cmd = controller->runControl(des, *px4_state->odom->value().first,
                                            *px4_state->imu->value().first);
    spdlog::debug("Takeoff Des Position:x :{} y:{} z:{}, velocity: v:{}, ctrl_cmd: thrust:{}\r",des.p.x(),des.p.y(),des.p.z(),des.v.z(),ctrl_cmd.thrust);
    estimate_thrust();
    break;
  }

  case L2_HOVERING: {
    // keep hovering
    // 如果在Hovering的情况下ForceHovering，状态不会发生改变
    // 如果在其他模式下ForceHovering，状态会发生改变
    if (L2.last_state != L2_HOVERING) {
      if(L2.last_state==L2_TAKING_OFF){
        L2hovering.des_pos = L2takingoff.start_pos;
        L2hovering.des_pos.z() += px4ctrl_params->statemachine_params.l2_takeoff_height;
        L2hovering.des_q = L2takingoff.start_q;
      }else{
          // get current position
          const auto &pose = px4_state->odom->value().first->pose.pose;
          auto &pos = pose.position;
          auto &quat = pose.orientation;
          L2hovering.des_pos = Eigen::Vector3d(pos.x, pos.y, pos.z);
          L2hovering.des_q = Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z);
          auto yaw = controller::yawFromQuat(L2hovering.des_q);
          L2hovering.des_q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
      }
      spdlog::info("Hovering at->X:{} Y:{} Z:{}", L2hovering.des_pos.x(), L2hovering.des_pos.y(), L2hovering.des_pos.z());
    }

    controller::DesiredState des;
    des.p = L2hovering.des_pos;
    des.q = L2hovering.des_q;
    des.yaw = controller::yawFromQuat(des.q);
    ctrl_cmd = controller->runControl(des, *px4_state->odom->value().first,
                                            *px4_state->imu->value().first);
    estimate_thrust();
    spdlog::debug("Hover Des Position:x :{} y:{} z:{}, velocity: v:{}, ctrl_cmd: thrust:{}\r",des.p.x(),des.p.y(),des.p.z(),des.v.z(),ctrl_cmd.thrust);
    break;
  }

  case L2_LANDING: {
    //From Any state to LANDING
    if (L2.last_state != L2_LANDING) {
      const auto &pose = px4_state->odom->value().first->pose.pose;
      auto &pos = pose.position;
      auto &quat = pose.orientation;
      L2landing.start_pos = Eigen::Vector3d(pos.x, pos.y, pos.z);
      L2landing.start_q = Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z);
      L2landing.start_time = clock::now();
      L2landing.is_first_time = true;
    }

    Eigen::Vector3d vel =
        Eigen::Vector3d(px4_state->odom->value().first->twist.twist.linear.x,
                        px4_state->odom->value().first->twist.twist.linear.y,
                        px4_state->odom->value().first->twist.twist.linear.z);
    controller::DesiredState des;
    des.p = L2landing.start_pos;
    des.q = L2landing.start_q;
    des.v = Eigen::Vector3d(0, 0, -px4ctrl_params->statemachine_params.l2_takeoff_landing_speed);
    des.yaw = controller::yawFromQuat(des.q);
    des.p.z() -= px4ctrl_params->statemachine_params.l2_takeoff_landing_speed *timePassedSeconds(L2landing.start_time);

    ctrl_cmd = controller->runControl(des, *px4_state->odom->value().first,
                                            *px4_state->imu->value().first);
    bool landed = false;
    // land_detector parameters
    const double POSITION_DEVIATION_C = px4ctrl_params->statemachine_params.l2_land_position_deviation_c;// Constraint 1: target position below real position for POSITION_DEVIATION_C meters.
    const double VELOCITY_THR_C = px4ctrl_params->statemachine_params.l2_land_velocity_thr_c; // Constraint 2: velocity below VELOCITY_MIN_C m/s.
    const double TIME_KEEP_C = px4ctrl_params->statemachine_params.l2_land_time_keep_c;// Constraint 3: Time(ms) the Constraint 1&2 need to keep.
  
    bool C12_satisfy =
        (des.p(2) - px4_state->odom->value().first->pose.pose.position.z) < POSITION_DEVIATION_C &&
        vel.norm() < VELOCITY_THR_C;
    if (C12_satisfy) {
      if (L2landing.is_first_time) {
        L2landing.time_C12_reached = clock::now();
        L2landing.is_first_time = false;
      }
      if (timePassed(L2landing.time_C12_reached) > TIME_KEEP_C)
      { // Constraint 3 reached
        spdlog::info("Successfully landed");
        landed = true;
      }
    }
    spdlog::debug("Land Des Position:x :{} y:{} z:{}, velocity: v:{}, ctrl_cmd: thrust:{}\n",des.p.x(),des.p.y(),des.p.z(),des.v.z(),ctrl_cmd.thrust);
    spdlog::debug("Landing: C12_satisfy:{}, landed:{}, Landed_state:{}\r",
    C12_satisfy, landed, px4_state->ext_state->value().first->landed_state);

    if (landed==true) {
      // Disarm
      if(!px4_bridge->force_disarm()){
        spdlog::error("Failed to force disarm");
      }
      controller->resetThrustMapping();
      L2 = L2_IDLE;
    }
    break;
  }

  case L2_ALLOW_CMD_CTRL: {
    // keep hovering, until cmd received
    if(cmdctrl_hz>=px4ctrl_params->statemachine_params.l2_cmd_ctrl_min_hz){
      L2 = L2_CMD_CTRL;
      break;
    }else{
      spdlog::error("cmdctrl_hz:{} is less than min_hz:{}",cmdctrl_hz,px4ctrl_params->statemachine_params.l2_cmd_ctrl_min_hz);
      L2 = L2_HOVERING;
    }

    if (L2.last_state != L2_ALLOW_CMD_CTRL) {
      const auto &pose = px4_state->odom->value().first->pose.pose;
      auto &pos = pose.position;
      auto &quat = pose.orientation;
      L2hovering.des_pos = Eigen::Vector3d(pos.x, pos.y, pos.z);
      L2hovering.des_q = Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z);
    }

    controller::DesiredState des;
    des.p = L2hovering.des_pos;
    des.q = L2hovering.des_q;
    des.yaw = controller::yawFromQuat(des.q);
    ctrl_cmd = controller->runControl(des, *px4_state->odom->value().first,
                                            *px4_state->imu->value().first);
    estimate_thrust();
    break;
  }

  case L2_CMD_CTRL: {
    // check if cmd is timed out
    if (cmdctrl_hz<px4ctrl_params->statemachine_params.l2_cmd_ctrl_min_hz)
    {
      spdlog::error("cmdctrl_hz:{} is less than min_hz:{}",cmdctrl_hz,px4ctrl_params->statemachine_params.l2_cmd_ctrl_min_hz);
      L2 = L2_HOVERING;
      break;
    }
    break;
  }

  default: {
    spdlog::error("Unexcepted L2 state:{}", state_map(L2.state));
    break;
  }
  }


  if (L2.next_state != L2.state) {
    spdlog::info("L2 state change:{}->{}", state_map(L2.state),
                     state_map(L2.next_state));
  }
  L2.step(); // 记录这一轮的状态，下一轮的状态，以及上一轮的状态
}

bool Px4Ctrl::force_l2_state(const Px4CtrlState &state) {
  L2 = state;
  spdlog::info("Force L2 state:{}, last_state:{}", state_map(L2.state),
              state_map(L2.last_state));
  L2.step();
  return true;
}

std::array<const Px4CtrlState,3> Px4Ctrl::get_px4_state() const{
  return {L0.state,L1.state,L2.state};
}

std::pair<const Eigen::Vector3d, const Eigen::Quaterniond>  Px4Ctrl::get_hovering_pos() const{
  return {L2hovering.des_pos,L2hovering.des_q};
}

bool  Px4Ctrl::set_hovering_pos(const Eigen::Vector3d& pos, const Eigen::Quaterniond& q){
  L2hovering.des_pos = pos;
  L2hovering.des_q = q;
  return true;
}

} // namespace px4ctrl