#pragma once
#include <string>

#define MAKE_ENUM(VAR) VAR,
#define MAKE_STRINGS(VAR) #VAR,

#define GEN_ENUM(FUNC) \
    FUNC(NOT_CONNECTED)\
    FUNC(L0_NON_OFFBOARD)\
    FUNC(L0_OFFBOARD)\
    FUNC(L0_L1)\
    FUNC(L1_UNARMED)\
    FUNC(L1_ARMED)\
    FUNC(L1_L2)\
    FUNC(L2_IDLE)\
    FUNC(L2_TAKING_OFF)\
    FUNC(L2_HOVERING)\
    FUNC(L2_ALLOW_CMD_CTRL)\
    FUNC(L2_CMD_CTRL)\
    FUNC(L2_LANDING)\
    FUNC(END)\
    FUNC(DEADLOCK)

enum Px4CtrlState{
    GEN_ENUM(MAKE_ENUM)
};

const char* const Px4CtrlStateName[] = {
    GEN_ENUM(MAKE_STRINGS)
};

#undef MAKE_ENUM
#undef MAKE_STRINGS
#undef GEN_ENUM

inline std::string state_map(const Px4CtrlState& state){
    return Px4CtrlStateName[state];
}