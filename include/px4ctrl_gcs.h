#pragma once

#include <cstdint>

namespace px4ctrl {
namespace gcs {
namespace command {

  #define MAKE_ENUM(VAR) VAR,
  #define MAKE_STRINGS(VAR) #VAR,

  #define GEN_ENUM(FUNC) \
    FUNC(EMPTY)\
    FUNC(ENTER_OFFBOARD)\
    FUNC(EXIT_OFFBOARD)\
    FUNC(ARM)\
    FUNC(DISARM)\
    FUNC(TAKEOFF)\
    FUNC(LAND)\
    FUNC(ALLOW_CMD_CTRL)\
    FUNC(FORCE_HOVER)

  enum DRONE_COMMAND{
      GEN_ENUM(MAKE_ENUM)
  };

  const char* const DRONE_COMMAND_NAME[] = {
      GEN_ENUM(MAKE_STRINGS)
  };

  #undef MAKE_ENUM
  #undef MAKE_STRINGS
  #undef GEN_ENUM


  #define ALL_DRONES 0xFF
  #define ALL_GCS 0xFF
}


struct Gcs {
  uint32_t id;//mesage id
  uint8_t version;
  uint8_t gcs_id;
  uint8_t drone_id;
  uint8_t drone_cmd;
  uint8_t len_payload;
  uint8_t payload[];
};

inline void reset(Gcs& gcs){
  gcs.id = 0;
  gcs.version = 0;
  gcs.drone_id = 0;
  gcs.drone_cmd = command::EMPTY;
  gcs.len_payload = 0;
}

struct Drone {
  uint32_t id;//mesage id
  uint8_t version;
  uint8_t gcs_id;
  uint8_t drone_id;
  uint8_t px4ctrl_status[3];
  float pos[3];
  float vel[3];
  float acc[3];
  float omega[3];
  float quat[4];
  float battery;
};

inline void reset(Drone& drone){
  drone.id = 0;
  drone.version = 0;
  drone.drone_id = 0;
  drone.quat[0] = 1;
  for(int i=0;i<3;++i){
    drone.px4ctrl_status[i] = 0;
    drone.pos[i] = 0;
    drone.vel[i] = 0;
    drone.acc[i] = 0;
    drone.omega[i] = 0;
    drone.quat[i+1] = 0;
  }
  drone.battery = 0;
}


template <typename S, typename R> class ICom {
public:
  virtual bool send(const S &) = 0;
  virtual bool receive(R &) = 0;
};

using GcsCom = ICom<Gcs,Drone>;
using DroneCom = ICom<Drone,Gcs>;


} // namespace gcs
} // namespace px4ctrl
