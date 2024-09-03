#pragma once
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/service_client.h"
#include <array>
#include <functional>
#include <memory>
#include <ros/ros.h>
#include <spdlog/spdlog.h>

#include "px4ctrl_state.h"

namespace px4ctrl{

class PX4CTRL_ROS_BRIDGE{

public:
    PX4CTRL_ROS_BRIDGE() = delete;
    PX4CTRL_ROS_BRIDGE(const ros::NodeHandle &nh, std::shared_ptr<PX4_STATE> px4_state);
    
    
    bool set_mode(const std::string &mode);
    bool set_arm(const bool arm);
    bool force_disarm();
    bool pub_bodyrates_target(const double thrust, const std::array<double, 3>& bodyrates);//目前只实现控制bodyrate
    bool pub_attitude_target(const double thrust, const std::array<double, 4>quat);
    void pub_allow_cmdctrl(bool allow);
    void spin_once();

private:
    ros::NodeHandle nh_;

    ros::Subscriber px4_state_sub, px4_extended_state_sub, px4_imu_sub, px4_bat_sub;

    ros::Subscriber vio_odom_sub, ctrl_cmd_sub, user_cmd_sub;

    ros::Publisher px4_cmd_pub, allow_cmdctrl_pub;
    ros::ServiceClient px4_set_mode_client, px4_arming_client, px4_cmd_client;

    std::shared_ptr<PX4_STATE> px4_state_;
};

template <typename MSG_PTR>
using CB_FUNC = std::function<void (MSG_PTR)>;

template <IPX4_ITEM ITEM, IPX4_TIME TIME>
inline CB_FUNC<ITEM> build_px4ros_cb(IPX4_STATE<ITEM, TIME> &msg //reference
){
    return std::bind(
        [](ITEM msg_ptr, IPX4_STATE<ITEM, TIME> * msg){
        msg->first = msg_ptr;
        msg->second = clock::now();
        },
     std::placeholders::_1, &msg
    );
}

}//px4ctrl