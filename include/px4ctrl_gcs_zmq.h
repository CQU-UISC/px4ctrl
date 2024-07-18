#pragma once
#include "px4ctrl_gcs.h"
#include <queue>
#include <thread>
#include <zmq.hpp>

namespace px4ctrl{
namespace gcs{
    namespace zmqimpl{
        class GcsZmqCom : public GcsCom{
        public:
            GcsZmqCom()=delete;
            GcsZmqCom(std::string base_dir);
            ~GcsZmqCom();
            
            zmq::context_t ctx;
            zmq::socket_t pub;
            zmq::socket_t sub;
            bool send(const Gcs&) override;
            bool receive(Drone&) override;
            
        private:
            std::thread recv_thread;
            bool ok = true;
            void recv();

            std::queue<Drone> cb_queue;

            std::string xpub_endpoint;
            std::string xsub_endpoint;
            std::string drone_topic;
            std::string gcs_topic;
        };

        class DroneZmqCom : public DroneCom{
        public:
            DroneZmqCom()=delete;
            DroneZmqCom(std::string base_dir);
            ~DroneZmqCom();

            zmq::context_t ctx;
            zmq::socket_t pub;
            zmq::socket_t sub;
            bool send(const Drone&) override;
            bool receive(Gcs&) override;
        private:

            std::thread recv_thread;
            bool ok = true;
            void recv();

            std::queue<Gcs> cb_queue;

            std::string xpub_endpoint;
            std::string xsub_endpoint;
            std::string drone_topic;
            std::string gcs_topic;
        };
    }
}
}
