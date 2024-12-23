#include <memory>
#include <ros/ros.h>
#include <signal.h>

#include "px4ctrl_fsm.h"
#include "px4ctrl_bridge.h"
#include "px4ctrl_ui.h"

std::shared_ptr<px4ctrl::Px4Ctrl> px4ctrl_fsm;

void sigintHandler( int sig ) {
    px4ctrl_fsm->stop();
    spdlog::info( "[PX4Ctrl] exit..." );
    ros::shutdown();
}

int main( int argc, char* argv[] ) {
    ros::init( argc, argv, "px4ctrl" );
    ros::NodeHandle nh( "~" );
    
    std::string base_dir;
    std::string cfg_name;
    nh.param( "px4ctrl_base_dir", base_dir, std::string( "" ));
    nh.param( "px4ctrl_cfg_name", cfg_name, std::string( "px4ctrl.yaml" ));
    if( base_dir.empty()){
        ROS_ERROR( "px4ctrl_base_dir is not set" );
        return -1;
    }
    if( cfg_name.empty()){
        ROS_ERROR( "px4ctrl_cfg_name is not set" );
        return -1;
    }

    // init logging
    std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::string date(30, '\0');
    std::strftime(&date[0], date.size(), "%Y-%m-%d-%H:%M:%S",std::localtime(&now));
    std::shared_ptr<px4ctrl::ftxui_sink_mt> ui_sink = std::make_shared<px4ctrl::ftxui_sink_mt>();
    std::shared_ptr<spdlog::sinks::basic_file_sink_mt>  file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(base_dir + "/log/px4ctrl_" + date + ".log", true);
    std::shared_ptr<spdlog::logger> logger_ptr = std::make_shared<spdlog::logger>("px4ctrl", spdlog::sinks_init_list{ui_sink, file_sink});
    spdlog::set_default_logger(logger_ptr);

    std::shared_ptr<px4ctrl::Px4CtrlParams> px4ctrl_params = std::make_shared<px4ctrl::Px4CtrlParams>(px4ctrl::Px4CtrlParams::load(base_dir + "/config/"+cfg_name));
    std::shared_ptr<px4ctrl::Px4State> px4_state = std::make_shared<px4ctrl::Px4State>();
    std::shared_ptr<px4ctrl::Px4CtrlRosBridge> px4_bridge = std::make_shared<px4ctrl::Px4CtrlRosBridge>(nh,px4_state);
    px4ctrl_fsm = std::make_shared<px4ctrl::Px4Ctrl>(px4_bridge, px4_state, px4ctrl_params);
    std::shared_ptr<px4ctrl::Px4CtrlUI> px4ctrl_ui = std::make_shared<px4ctrl::Px4CtrlUI>(ui_sink, px4_state, px4_bridge, px4ctrl_fsm);
    signal( SIGINT, sigintHandler );
    px4ctrl_ui->start();
    px4ctrl_fsm->run();
    return 0;
}
