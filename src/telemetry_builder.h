#pragma once

#include <memory>

#include "context.h"
#include "controller.h"
#include "datas.h"
#include "params.h"
#include "types.h"

namespace px4ctrl {

class TelemetryBuilder {
public:
  TelemetryBuilder(std::shared_ptr<Context> ctx,
                   std::shared_ptr<Px4CtrlParams> params,
                   std::shared_ptr<controller::Se3Control> ctrl);

  ui::ServerPayload build();

private:
  std::shared_ptr<Context> ctx_;
  std::shared_ptr<Px4CtrlParams> params_;
  std::shared_ptr<controller::Se3Control> controller_;
};

} // namespace px4ctrl
