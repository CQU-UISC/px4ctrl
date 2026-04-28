#pragma once

#include <memory>

#include "bridge.h"
#include "context.h"
#include "controller.h"
#include "datas.h"
#include "params.h"
#include "types.h"

namespace px4ctrl {

class ClientCommandHandler {
public:
  ClientCommandHandler(std::shared_ptr<Px4CtrlRosBridge> bridge,
                       std::shared_ptr<Context> ctx,
                       std::shared_ptr<Px4CtrlParams> params,
                       std::shared_ptr<controller::Se3Control> ctrl);

  void handle(const ui::ClientPayload &payload);

private:
  void handle_arm();
  void handle_force_disarm();
  void handle_enter_offboard();
  void handle_exit_offboard();
  void handle_takeoff();
  void handle_land();
  void handle_force_hover();
  void handle_allow_cmd_ctrl();
  void handle_change_hover_pos(const ui::ClientPayload &payload);
  void handle_set_safety_limits(const ui::ClientPayload &payload);

  bool validate_safety_limit(double v) const;
  void update_safety_limits(const ui::SafetyLimitsPayload &limits);

  std::shared_ptr<Px4CtrlRosBridge> bridge_;
  std::shared_ptr<Context> ctx_;
  std::shared_ptr<Px4CtrlParams> params_;
  std::shared_ptr<controller::Se3Control> controller_;
};

} // namespace px4ctrl
