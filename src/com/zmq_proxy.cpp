#include "com/zmq_proxy.h"

#include <fstream>
#include <iostream>
#include <spdlog/spdlog.h>
#include <signal.h>

//本地的zmq proxy
namespace px4ctrl {
namespace gcs {
        using json=nlohmann::json;
        ZmqProxy::ZmqProxy(std::string base_dir):ctx(2),
        xsub_socket(ctx,zmq::socket_type::xsub),
        xpub_socket(ctx,zmq::socket_type::xpub){
            std::ifstream config_file(base_dir + "/px4ctrl_com.json");
            json json_file;
            if (!config_file.is_open()) {
                spdlog::error("Failed to open config file");
                exit(1);
            }
            config_file >> json_file;
            spdlog::info("Config file loaded");
            config_file.close();

            if (!(json_file.contains("xpub_endpoint_bind") && 
                json_file.contains("xsub_endpoint_bind")
                )) {
                spdlog::error("Config file missing key");
                exit(1);
            }
            xpub_endpoint = json_file["xpub_endpoint_bind"].template get<std::string>();
            xsub_endpoint = json_file["xsub_endpoint_bind"].template get<std::string>();

            // Init XSUB socket
            try
            {
                xsub_socket.bind(xsub_endpoint);
                xpub_socket.bind(xpub_endpoint);
            }
            catch (zmq::error_t e)
            {
                spdlog::error("unable bind xsub/xpub,check your config file, err:{}",e.what());
                exit(1);
            }
            proxy_thread = std::thread(&ZmqProxy::run,this);
        }

        ZmqProxy::~ZmqProxy(){
            close();
            join();
        }

        void ZmqProxy::run(){
            zmq::proxy(xsub_socket, xpub_socket);
            return;
        }

        void ZmqProxy::join(){
            if(proxy_thread.joinable()){
                proxy_thread.join();
            }
        }

        void ZmqProxy::close(){
            ctx.shutdown();
            ctx.close();
        }
}
} 




    