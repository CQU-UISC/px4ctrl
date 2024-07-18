#include "px4ctrl_gcs_zmq.h"
#include "px4ctrl_gcs.h"

#include <spdlog/spdlog.h>
#include <zmq.hpp>
#include <zmq_addon.hpp>
#include <json.hpp>
#include <fstream>

using json=nlohmann::json;
namespace px4ctrl{
namespace gcs{
    namespace zmqimpl{
        GcsZmqCom::GcsZmqCom(std::string base_dir):
        pub(ctx,zmq::socket_type::pub),
        sub(ctx,zmq::socket_type::sub){
            //std::string base_dir  = std::filesystem::current_path().string();//current working dir
            std::ifstream config_file(base_dir + "/px4ctrl_zmq.json");
            json json_file;
            if (!config_file.is_open()) {
                spdlog::error("Failed to open config file");
                exit(1);
            }
            config_file >> json_file;
            spdlog::info("Config file loaded");
            config_file.close();

            if (!(json_file.contains("xpub_endpoint") && 
                json_file.contains("xsub_endpoint")&&
                json_file.contains("drone_topic")&&
                json_file.contains("gcs_topic")
                )) {
                spdlog::error("Config file missing key");
                exit(1);
            }

            xpub_endpoint = json_file["xpub_endpoint"].template get<std::string>();
            xsub_endpoint = json_file["xsub_endpoint"].template get<std::string>();
            drone_topic  = json_file["drone_topic"].template get<std::string>();
            gcs_topic  = json_file["gcs_topic"].template get<std::string>();
            
            try
            {
                // The port number here is the XSUB port of the Msg Proxy service (9200)
                pub.connect(xsub_endpoint);
                sub.connect(xpub_endpoint);
            }
            catch (zmq::error_t e)
            {
                spdlog::error("Error connection to endpoint. Error is: {}", e.what());
                exit(1);
            }
            sub.set(zmq::sockopt::subscribe,drone_topic.c_str());
            recv_thread = std::thread(
                &GcsZmqCom::recv,this
            );
        }
        GcsZmqCom::~GcsZmqCom(){
            ok = false;
            recv_thread.join();
            ctx.shutdown();
            ctx.close();
        }

        void GcsZmqCom::recv(){
            while(ok){
                Drone drone;
                // Receive all parts of the message
                std::vector<zmq::message_t> recv_msgs;
                zmq::recv_result_t result =
                zmq::recv_multipart(sub, std::back_inserter(recv_msgs));
                assert(result && "recv failed");
                assert(*result == 2);
                //spdlog::info("[Gcs] recv,tpoic: {} msg:{}",recv_msgs.at(0).to_string(),recv_msgs.at(1).to_string());
                deserialize(recv_msgs[1].to_string(),drone);
                while(cb_queue.size()>10){
                    cb_queue.pop();
                }
                cb_queue.push(drone);
            }
        }

        bool GcsZmqCom::receive(Drone& drone){
            if(cb_queue.empty()){
                return false;
            }
            
            drone = cb_queue.front();
            cb_queue.pop();
            return true;
        }

        bool GcsZmqCom::send(const Gcs& gcs){
            std::string msg;
            if(!serialize(gcs,msg)){
                return false;
            }
            //TODO read doc
            pub.send(zmq::buffer(gcs_topic),zmq::send_flags::sndmore);
            pub.send(zmq::buffer(msg));
            return true;
        }


    DroneZmqCom::DroneZmqCom(std::string base_dir):
        pub(ctx,zmq::socket_type::pub),
        sub(ctx,zmq::socket_type::sub){
            // std::string base_dir  = std::filesystem::current_path().string();//current working dir
            std::ifstream config_file(base_dir + "/px4ctrl_zmq.json");
            json json_file;
            if (!config_file.is_open()) {
                spdlog::error("Failed to open config file");
                exit(1);
            }
            config_file >> json_file;
            spdlog::info("Config file loaded");
            config_file.close();

            if (!(json_file.contains("xpub_endpoint") && 
                json_file.contains("xsub_endpoint")&&
                json_file.contains("drone_topic")&&
                json_file.contains("gcs_topic")
                )) {
                spdlog::error("Config file missing key");
                exit(1);
            }

            xpub_endpoint = json_file["xpub_endpoint"].template get<std::string>();
            xsub_endpoint = json_file["xsub_endpoint"].template get<std::string>();
            drone_topic  = json_file["drone_topic"].template get<std::string>();
            gcs_topic  = json_file["gcs_topic"].template get<std::string>();
            
            try
            {
                // The port number here is the XSUB port of the Msg Proxy service (9200)
                pub.connect(xsub_endpoint);
                sub.connect(xpub_endpoint);
            }
            catch (zmq::error_t e)
            {
                spdlog::error("Error connection to endpoint. Error is: {}", e.what());
                exit(1);
            }
            sub.set(zmq::sockopt::subscribe,gcs_topic.c_str());
            recv_thread = std::thread(
                &DroneZmqCom::recv,this
            );
        }

        DroneZmqCom::~DroneZmqCom(){
            ok = false;
            recv_thread.join();
            ctx.shutdown();
            ctx.close();
        }

        void DroneZmqCom::recv(){
            while(ok){
                Gcs gcs;
                // Receive all parts of the message
                std::vector<zmq::message_t> recv_msgs;
                zmq::recv_result_t result =
                zmq::recv_multipart(sub, std::back_inserter(recv_msgs));
                assert(result && "recv failed");
                assert(*result == 2);
                //spdlog::info("[Drone] recv,tpoic: {} msg:{}",recv_msgs.at(0).to_string(),recv_msgs.at(1).to_string());
                deserialize(recv_msgs[1].to_string(),gcs);
                while(cb_queue.size()>10){
                    cb_queue.pop();
                }
                cb_queue.push(gcs);
            }
        }

        bool DroneZmqCom::receive(Gcs& gcs){
            if(cb_queue.empty()){
                return false;
            }
            
            gcs = cb_queue.front();
            cb_queue.pop();
            return true;
        }

        bool DroneZmqCom::send(const Drone& drone){
            std::string msg;
            if(!serialize(drone,msg)){
                return false;
            }
            //TODO read doc
            pub.send(zmq::buffer(drone_topic),zmq::send_flags::sndmore);
            pub.send(zmq::buffer(msg));
            return true;
        }


}
}
}