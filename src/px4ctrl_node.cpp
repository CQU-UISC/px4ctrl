#include <memory>
#include <ros/ros.h>
#include <signal.h>
#include "px4ctrl_fsm.h"
#include "px4ctrl_gcs_ros.h"
#include "px4ctrl_mavros.h"
#include "px4ctrl_state.h"


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
    nh.param( "px4ctrl_base_dir", base_dir, std::string( "" ));
    if( base_dir.empty() ) {
        ROS_ERROR( "px4ctrl_base_dir is not set" );
        return -1;
    }
    std::shared_ptr<px4ctrl::PX4_STATE> px4_state = std::make_shared<px4ctrl::PX4_STATE>();
    std::shared_ptr<px4ctrl::PX4CTRL_ROS_BRIDGE> px4_mavros = std::make_shared<px4ctrl::PX4CTRL_ROS_BRIDGE>(nh,px4_state);
    std::shared_ptr<px4ctrl::gcs::rosimpl::DroneRosCom> drone_com = std::make_shared<px4ctrl::gcs::rosimpl::DroneRosCom>(nh);
    px4ctrl_fsm = std::make_shared<px4ctrl::Px4Ctrl>(base_dir,px4_mavros,px4_state,drone_com);
    signal( SIGINT, sigintHandler );
    px4ctrl_fsm->run();
    return 0;
}
