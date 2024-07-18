#pragma once

#include <array>
#include <json.hpp>
#include <cstdint>
#include <string>

using json = nlohmann::json;

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
  uint8_t len_payload;//not used
  uint8_t payload[];
};

inline void reset(Gcs& gcs){
  gcs.id = 0;
  gcs.version = 0;
  gcs.drone_id = 0;
  gcs.drone_cmd = command::EMPTY;
  gcs.len_payload = 0;
}

inline bool serialize(const Gcs& gcs,std::string& str){
json j = {
  {"id",gcs.id},
  {"version",gcs.version},
  {"gcs_id",gcs.gcs_id},
  {"drone_id",gcs.drone_id},
  {"drone_cmd",gcs.drone_cmd},
};
str  =  j.dump();
return true;
}

inline bool deserialize(const std::string& gcs_str,Gcs& gcs){
  json j = json::parse(gcs_str);
  if(!(
    j.contains("id")&&
    j.contains("version")&&
    j.contains("gcs_id")&&
    j.contains("drone_id")&&
    j.contains("drone_cmd")
  )){
    return false;
  }
  gcs.id = j["id"].template get<uint32_t>();
  gcs.version = j["version"].template get<uint8_t>();
  gcs.gcs_id = j["gcs_id"].template get<uint8_t>();
  gcs.drone_id = j["drone_id"].template get<uint8_t>();
  gcs.drone_cmd = j["drone_cmd"].template get<uint8_t>();
  return true;
}

struct Drone {
  uint32_t id;//mesage id
  uint8_t version;
  uint8_t gcs_id;
  uint8_t drone_id;
  std::array<uint8_t, 3> px4ctrl_status;
  std::array<float, 3> pos;
  std::array<float, 3> vel;
  std::array<float, 3> acc;
  std::array<float, 3> omega;
  std::array<float, 4> quat;
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

inline bool serialize(const Drone& drone,std::string& str){
json j = {
  {"id",drone.id},
  {"version",drone.version},
  {"gcs_id",drone.gcs_id},
  {"drone_id",drone.drone_id},
  {"px4ctrl_status",{drone.px4ctrl_status[0],drone.px4ctrl_status[1],drone.px4ctrl_status[2]}},
  {"pos",{drone.pos[0],drone.pos[1],drone.pos[2]}},
  {"vel",{drone.vel[0],drone.vel[1],drone.vel[2]}},
  {"acc",{drone.acc[0],drone.acc[1],drone.acc[2]}},
  {"omega",{drone.omega[0],drone.omega[1],drone.omega[2]}},
  {"quat",{drone.quat[0],drone.quat[1],drone.quat[2],drone.quat[3]}},
  {"battery",drone.battery}
};
str  =  j.dump();
return true;
}

inline bool deserialize(const std::string& drone_str,Drone& drone){
  json j = json::parse(drone_str);
  if(!(
    j.contains("id")&&
    j.contains("version")&&
    j.contains("gcs_id")&&
    j.contains("drone_id")&&
    j.contains("px4ctrl_status")&&
    j.contains("pos")&&
    j.contains("vel")&&
    j.contains("acc")&&
    j.contains("omega")&&
    j.contains("quat")&&
    j.contains("battery")
  )){
    return false;
  }
  j.at("id").get_to(drone.id);
  j.at("version").get_to(drone.version);
  j.at("gcs_id").get_to(drone.gcs_id);
  j.at("drone_id").get_to(drone.drone_id);
  j.at("px4ctrl_status").get_to(drone.px4ctrl_status);
  j.at("pos").get_to(drone.pos);
  j.at("vel").get_to(drone.vel);
  j.at("acc").get_to(drone.acc);
  j.at("omega").get_to(drone.omega);
  j.at("quat").get_to(drone.quat);
  j.at("battery").get_to(drone.battery);

  return true;
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
