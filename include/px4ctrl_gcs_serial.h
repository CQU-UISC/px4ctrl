#pragma once
#include "px4ctrl_gcs.h"
#include "serial.hpp"
#include <memory>
#include <mutex>
#include <queue>
#include <thread>


namespace px4ctrl{
namespace gcs{
    namespace serialimpl{
        class GcsSerialCom : public GcsCom{
        public:
            GcsSerialCom()=delete;
            GcsSerialCom(std::string base_dir);
            ~GcsSerialCom();
            

            bool send(const Gcs&) override;
            bool receive(Drone&) override;
            
        private:
            std::shared_ptr<SerialPort> serial;
            std::mutex mutex_queue;

            std::queue<Drone> cb_queue;
        };

        class DroneSerialCom : public DroneCom{
        public:
            DroneSerialCom()=delete;
            DroneSerialCom(std::string base_dir);
            ~DroneSerialCom();

            bool send(const Drone&) override;
            bool receive(Gcs&) override;
        private:
            std::shared_ptr<SerialPort> serial;
            std::mutex mutex_queue;

            std::queue<Gcs> cb_queue;
        };
    }
}
}
