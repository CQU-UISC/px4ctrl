#include "mission_fsm.h"

namespace px4ctrl {

std::shared_ptr<Context> MissionFSM::ctx;

} // namespace px4ctrl

FSM_INITIAL_STATE(px4ctrl::MissionFSM, px4ctrl::Standby)
