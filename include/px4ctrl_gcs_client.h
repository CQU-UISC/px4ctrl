#pragma once
#include "px4ctrl_gcs.h"
#include "px4ctrl_state.h"
#include <cstdint>
#include <map>
#include <memory>
#include <queue>
#include <utility>

namespace px4ctrl{
namespace gcs{
    
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