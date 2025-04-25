#pragma once
#include <cstdint>
#include <fmt/format.h> //because ros2 foxy do not support cxx20
#include <spdlog/spdlog.h>
#include <string>
#include <zmq.hpp>
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/exceptions.h>
#include <yaml-cpp/node/node.h>

namespace px4ctrl
{
namespace ui
{

struct ServerPayload
{
    uint8_t id;
    uint64_t timestamp;
    // rigid body
    float pos[3];
    float vel[3];
    float omega[3];
    float quat[4];
    // controller debug msg
    float thrust_setpoint;
    float omega_setpoint[3];
    // quad state
    float battery_voltage;
    // fsm state
    int32_t fsm_state[3];
    float thrust_map[3]; // estimate or real
    float hover_pos[3];
    float hover_quat[4]; // actually only have yaw
    float odom_hz;
    float cmdctrl_hz;
};

enum class ClientCommand
{
    HEARTBEAT,
    ARM,
    ENTER_OFFBOARD,
    EXIT_OFFBOARD,
    TAKEOFF,
    LAND,
    FORCE_HOVER,
    ALLOW_CMD_CTRL,
    FORCE_DISARM,
    CHANGE_HOVER_POS,
    RESTART_FCU
};

const char* const CommandStr[] = {
    "HEARTBEAT",      "ARM",
    "ENTER_OFFBOARD", "EXIT_OFFBOARD",
    "TAKEOFF",        "LAND",
    "FORCE_HOVER",    "ALLOW_CMD_CTRL",
    "FORCE_DISARM",   "CHANGE_HOVER_POS",
    "RESTART_FCU"
};

struct ClientPayload
{
    uint8_t id;
    uint64_t timestamp;
    // command
    ClientCommand command;
    // data
    uint8_t data[64]; // for future use
};

template <typename T>
inline void pack(zmq::message_t& msg, const T& data)
{
    msg.rebuild(sizeof(T));
    std::memcpy(msg.data(), &data, sizeof(T));
    return;
}

template <typename T>
inline void unpack(const zmq::message_t& msg, T& data)
{
    // check size
    if (msg.size() != sizeof(T))
    {
        throw std::runtime_error("Message size does not match");
    }
    std::memcpy(&data, msg.data(), sizeof(T));
    return;
}

struct ZmqParas{
    std::string xsub_endpoint_port;
    std::string xpub_endpoint_port;
    std::string xsub_endpoint_ip;
    std::string xpub_endpoint_ip;

    std::string xsub_bind, xpub_bind;
    std::string xsub_url, xpub_url;


    std::string server_topic;
    std::string client_topic;
    std::string log_topic;

    inline static ZmqParas load(const std::string& file){
        // read from file
        ZmqParas paras;
        try {
            YAML::Node config = YAML::LoadFile(file);
            paras.xsub_endpoint_port = config["xsub_endpoint_port"].as<std::string>();
            paras.xpub_endpoint_port = config["xpub_endpoint_port"].as<std::string>();
            paras.xsub_endpoint_ip = config["xsub_endpoint_ip"].as<std::string>();
            paras.xpub_endpoint_ip = config["xpub_endpoint_ip"].as<std::string>();
            paras.server_topic = config["server_topic"].as<std::string>();
            paras.client_topic = config["client_topic"].as<std::string>();
            paras.log_topic = config["log_topic"].as<std::string>();

            paras.xpub_bind = fmt::format("tcp://*:{}",paras.xpub_endpoint_port);
            paras.xsub_bind = fmt::format("tcp://*:{}",paras.xsub_endpoint_port);
            paras.xpub_url = fmt::format("tcp://{}:{}",paras.xpub_endpoint_ip,paras.xpub_endpoint_port);
            paras.xsub_url = fmt::format("tcp://{}:{}",paras.xsub_endpoint_ip,paras.xsub_endpoint_port);
        }
        catch (const YAML::BadFile& e)
        {
            spdlog::error("error:{}", e.what());
            throw e;
        }
        catch (const YAML::ParserException& e)
        {
            spdlog::error("error:{}", e.what());
            throw e;
        }
        return paras;
    }
};
} // namespace ui
} // namespace px4ctrl