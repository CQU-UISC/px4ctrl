#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <spdlog/spdlog.h>
#include <signal.h>
#include <string>
#include <thread>

#include "com/px4ctrl_gcs.h"
#include "com/px4ctrl_gcs_zmq.h"
#include "com/serial.hpp"
#include "com/zmq_proxy.h"
#include "json.hpp"



void sigintHandler( int sig ) {
    spdlog::info( "[Zmq proxy] exit..." );
    exit(0);
}


int main(int argc, char* argv[]){
    // 检查参数数量
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " -c <config_dir>" << std::endl;
        return 1;
    }

    std::string flag = argv[1];
    std::string config_dir;

    // 检查参数是否为 -c
    if (flag == "-c") {
        config_dir = argv[2];
        std::cout << "Config directory: " << config_dir << std::endl;
    } else {
        std::cerr << "Invalid argument: " << flag << std::endl;
        std::cerr << "Usage: " << argv[0] << " -c <config_dir>" << std::endl;
        return 1;
    }
    signal( SIGINT, sigintHandler );

    std::ifstream config_file(config_dir + "/px4ctrl_com.json");
    nlohmann::json json_file;
    if (!config_file.is_open()) {
        spdlog::error("Failed to open config file");
        exit(1);
    }
    config_file >> json_file;
    spdlog::info("Config file loaded");
    config_file.close();

    if (!(json_file.contains("serial") 
        )) {
        spdlog::error("Config file missing key: serial");
        exit(1);
    }

    if (!(json_file["serial"].contains("enable") &&
        json_file["serial"].contains("port") &&
        json_file["serial"].contains("rate") &&
        json_file["serial"].contains("id") &&
        json_file["serial"].contains("hz") 
       )) {
        spdlog::error("Config file missing serial config");
        exit(1);
  }

    px4ctrl::gcs::ZmqProxy proxy(config_dir);
    bool enable_serial = json_file["serial"]["enable"].template get<bool>();
    std::string port = json_file["serial"]["port"].template get<std::string>();
    int rate = json_file["serial"]["rate"].template get<int>();;
    int id = json_file["serial"]["id"].template get<int>();
    int hz = json_file["serial"]["hz"].template get<int>();

    int time_to_sleep  = 1000/rate;
    
    if(enable_serial){
        std::shared_ptr<px4ctrl::gcs::zmqimpl::DroneZmqCom> drone = std::make_shared<px4ctrl::gcs::zmqimpl::DroneZmqCom>(config_dir);
        std::shared_ptr<px4ctrl::gcs::zmqimpl::GcsZmqCom> gcs = std::make_shared<px4ctrl::gcs::zmqimpl::GcsZmqCom>(config_dir);
        SerialPort serial(port,115200,hz);
        // std::function<void (const SwarmPacket &)> function
        // zmq<==>serial<=lora=>serial<==>zmq
        serial.addReceiveCallback([&](const  SwarmPacket & msg){
            //proxy for serial
            if(msg.msg_type==MSG_DRONE){
                px4ctrl::gcs::Drone drone_msg;
                memcpy(&drone_msg, msg.data, sizeof(px4ctrl::gcs::Drone));
                drone_msg.serial_forwad = true;
                drone->send(drone_msg);
            }
            if(msg.msg_type==MSG_GCS){
                px4ctrl::gcs::Gcs gcs_msg;
                memcpy(&gcs_msg, msg.data, sizeof(px4ctrl::gcs::Gcs));
                gcs_msg.serial_forwad = true;
                gcs->send(gcs_msg);
            }
        });
        serial.start();
        while (true) {
            px4ctrl::gcs::Gcs gcs_recv;
            px4ctrl::gcs::Drone drone_recv;
            SwarmPacket pkt;
            pkt.id = id;
            if(gcs->receive(gcs_recv)){
                // send
                if(!gcs_recv.serial_forwad){
                    if(gcs_recv.drone_cmd!=px4ctrl::gcs::command::EMPTY){
                        pkt.dontdrop = 1;
                        spdlog::warn("Recv command, don't drop");
                    }
                    pkt.msg_type = MSG_GCS;
                    memcpy(pkt.data, &gcs_recv, sizeof(px4ctrl::gcs::Gcs));
                    serial.send(pkt);
                }
            }
            if(drone->receive(drone_recv)){
                //send
                if(!drone_recv.serial_forwad){
                    pkt.msg_type = MSG_DRONE;
                    memcpy(pkt.data, &drone_recv, sizeof(px4ctrl::gcs::Drone));
                    serial.send(pkt);
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(time_to_sleep));
        }
    }else{
        proxy.join();
    }
    return 0;
}