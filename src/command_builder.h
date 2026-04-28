#pragma once

#include <memory>

#include "context.h"
#include "controller.h"
#include "params.h"
#include "types.h"

namespace px4ctrl {

class CommandBuilder {
public:
  CommandBuilder(std::shared_ptr<controller::Se3Control> ctrl,
                 std::shared_ptr<Context> ctx);

  /// Select control source, build the command with fallback chain.
  /// @return the effective control source used.
  ControlSource build(const MissionContextSnapshot &snap,
                      controller::ControlCommand &out);

private:
  ControlSource select_source(const MissionContextSnapshot &snap) const;

  void build_proof_alive(const MissionContextSnapshot &snap,
                         controller::ControlCommand &out) const;
  bool build_se3(const MissionContextSnapshot &snap,
                 controller::ControlCommand &out);
  bool build_external(const MissionContextSnapshot &snap,
                      controller::ControlCommand &out);
  bool build_safe_landing(const MissionContextSnapshot &snap,
                          controller::ControlCommand &out);

  std::shared_ptr<controller::Se3Control> controller_;
  std::shared_ptr<Context> ctx_;
};

} // namespace px4ctrl
