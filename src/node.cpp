#include <memory>
#include <signal.h>
#include <spdlog/spdlog.h>
#include <string>
#include <filesystem>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>

#include "fsm.h"
#include "bridge.h"
#include "server.h"

std::shared_ptr<px4ctrl::Px4Ctrl> px4ctrl_fsm;

void sigintHandler( int sig ) {
    spdlog::info( "[PX4Ctrl] exit..." );
    px4ctrl_fsm->stop();
}

int main( int argc, char* argv[] ) {
    rclcpp::init(argc, argv);
    auto node =  std::make_shared<rclcpp::Node>("px4ctrl");
    std::string base_dir, cfg_name, zmq_cfg_name;
    node->declare_parameter("px4ctrl_base_dir", "");
    node->declare_parameter("px4ctrl_cfg_name", "px4ctrl.yaml");
    node->declare_parameter("px4ctrl_zmq_cfg_name", "zmq.yaml");
    node->get_parameter("px4ctrl_base_dir", base_dir);
    node->get_parameter("px4ctrl_cfg_name", cfg_name);
    node->get_parameter("px4ctrl_zmq_cfg_name", zmq_cfg_name);
    
    // check if cfg exists
    std::string cfg_file = base_dir + "/config/"+cfg_name;
    std::string zmq_cfg_file = base_dir + "/config/"+zmq_cfg_name;
    if( !std::filesystem::exists( cfg_file ) ) {
        spdlog::error( "px4ctrl config file not found: {}", cfg_file );
        return -1;
    }
    if( !std::filesystem::exists( zmq_cfg_file ) ) {
        spdlog::error( "zmq config file not found: {}", zmq_cfg_file );
        return -1;
    }
    // log filenamez
    std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::string date(30, '\0');
    std::strftime(&date[0], date.size(), "%Y-%m-%d-%H:%M:%S",std::localtime(&now));
    std::string log_file = base_dir + "/log/px4ctrl_" + date + ".log";

    //zmq contex
    zmq::context_t ctx(1);

    // load params
    auto cfg = px4ctrl::Px4CtrlParams::load(cfg_file);
    auto zmq_cfg = px4ctrl::ui::ZmqParas::load(zmq_cfg_file);
    
    //zmq
    std::shared_ptr<px4ctrl::ui::ZmqProxy>  zmq_proxy = std::make_shared<px4ctrl::ui::ZmqProxy>(ctx, zmq_cfg);
    std::shared_ptr<px4ctrl::ui::Px4Server> px4ctrl_server = std::make_shared<px4ctrl::ui::Px4Server>(ctx, zmq_cfg);
    std::shared_ptr<px4ctrl::ui::zmq_sink_mt> zmq_sink = std::make_shared<px4ctrl::ui::zmq_sink_mt>(ctx, zmq_cfg);

    // init logging
    std::shared_ptr<spdlog::sinks::stdout_color_sink_mt> console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    std::shared_ptr<spdlog::sinks::basic_file_sink_mt>  file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(log_file, true);

    std::shared_ptr<spdlog::logger> logger_ptr = std::make_shared<spdlog::logger>("px4ctrl", spdlog::sinks_init_list{console_sink, file_sink,zmq_sink});
    spdlog::set_default_logger(logger_ptr);

    std::shared_ptr<px4ctrl::Px4CtrlParams> px4ctrl_params = std::make_shared<px4ctrl::Px4CtrlParams>(cfg);
    std::shared_ptr<px4ctrl::Px4State> px4_state = std::make_shared<px4ctrl::Px4State>();
    std::shared_ptr<px4ctrl::Px4CtrlRosBridge> px4_bridge = std::make_shared<px4ctrl::Px4CtrlRosBridge>(node, px4_state);
    px4ctrl_fsm = std::make_shared<px4ctrl::Px4Ctrl>(px4_bridge, px4_state, px4ctrl_params, px4ctrl_server);
    signal( SIGINT, sigintHandler );
    px4ctrl_fsm->run();
    rclcpp::shutdown();
    ctx.shutdown();
    ctx.close();
    spdlog::info( "[PX4Ctrl] exited" );
    return 0;
}
