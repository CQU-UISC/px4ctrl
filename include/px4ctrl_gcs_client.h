#pragma once
#include "px4ctrl_gcs.h"
#include "px4ctrl_def.h"

#include <cstdint>
#include <map>
#include <memory>
#include <queue>
#include <thread>
#include <utility>

#define __ZMQ_IMPL__
// #define __ROS_IMPL__

#ifdef __ZMQ_IMPL__
#undef __ROS_IMPL__
#include <zmq.hpp>
#include <json.hpp>
#endif


namespace px4ctrl{
namespace gcs{

#ifdef __ZMQ_IMPL__
  class ZmqProxy{
    public:
        ZmqProxy(std::string base_dir);
        ~ZmqProxy();
    private:
        void run();
        std::thread proxy_thread;

        zmq::context_t ctx;
        // Init XSUB socket
        zmq::socket_t xsub_socket;
        std::string xsub_endpoint;

        // Init XPUB socket
        zmq::socket_t xpub_socket;
        std::string xpub_endpoint;
  };
#endif


    class GcsClient{
        //
        //use imgui
        public:
            GcsClient(std::shared_ptr<GcsCom>);
            GcsClient() = delete;
            void render_window();
            
        private:
            //runs in 100hz
            void handle_message();

            std::shared_ptr<GcsCom> gcs;
            std::map<uint8_t, std::pair<Drone, clock::time_point>> drone_list;
            std::map<uint8_t, std::queue<Drone>> cb_queue;
            std::queue<Gcs> gcs_queue;
            uint32_t id=0;
            clock::time_point last_recv_time;
    };
}
}