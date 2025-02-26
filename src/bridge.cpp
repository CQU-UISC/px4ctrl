#include "bridge.h"
#include <spdlog/spdlog.h>

namespace px4ctrl {

    Px4CtrlRosBridge::Px4CtrlRosBridge(const rclcpp::Node::SharedPtr& node, std::shared_ptr<Px4State> px4_state):node(node),px4_state_(px4_state){
        px4_state_sub = node->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", 10, build_px4ros_cb(px4_state_->state));
        
          px4_extended_state_sub = node->create_subscription<mavros_msgs::msg::ExtendedState>(
            "/mavros/extended_state", 10, build_px4ros_cb(px4_state_->ext_state));
        
          vio_odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
            "odom", rclcpp::SensorDataQoS(), build_px4ros_cb(px4_state_->odom));
        
          ctrl_cmd_sub = node->create_subscription<px4msgs::msg::Command>(
            "cmd", rclcpp::SensorDataQoS(), build_px4ros_cb(px4_state_->ctrl_command));
        
          px4_imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
            "/mavros/imu/data", rclcpp::SensorDataQoS(), build_px4ros_cb(px4_state_->imu));
        
          px4_bat_sub = node->create_subscription<sensor_msgs::msg::BatteryState>(
            "/mavros/battery", rclcpp::SensorDataQoS(), build_px4ros_cb(px4_state_->battery));
        
          // Publishers
          px4_cmd_pub = node->create_publisher<mavros_msgs::msg::AttitudeTarget>(
            "/mavros/setpoint_raw/attitude", 10);
          
          allow_cmdctrl_pub = node->create_publisher<std_msgs::msg::Bool>(
            "allow_cmd", 10);
        
          // Service clients
          px4_set_mode_client = node->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
          px4_arming_client = node->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
          px4_cmd_client = node->create_client<mavros_msgs::srv::CommandLong>("/mavros/cmd/command");
        
        spdlog::info("Init px4ros node");
        return;
    }

    void Px4CtrlRosBridge::spin_once(){
        rclcpp::spin_some(node->get_node_base_interface());
        spdlog::debug("ros spin once");
    }

    bool Px4CtrlRosBridge::set_mode(const std::string &mode){
        if (px4_state_->state->value().first==nullptr){
            spdlog::error("px4 state is nullptr");
            return false;
        }

        if(px4_state_->state->value().first->mode == mode){
            spdlog::info("Already in {} mode", mode);
            return true;
        }
        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = mode;
        auto future = px4_set_mode_client->async_send_request(request);
        if (future.wait_for(1s) != std::future_status::ready) {
            spdlog::error("Set mode service call timed out");
            return false;
        }
        auto response = future.get();
        if (!response->mode_sent) {
          spdlog::error("Failed to set mode: {}", mode);
          return false;
        }else{
            spdlog::info( "Enter {} accepted by PX4!", mode);
            return true;
        }
        return true;
    }

    bool Px4CtrlRosBridge::enter_offboard(){
        return set_mode(mavros_msgs::msg::State::MODE_PX4_OFFBOARD);
    }

    bool Px4CtrlRosBridge::exit_offboard(){
        return set_mode(mavros_msgs::msg::State::MODE_PX4_STABILIZED);
    }

    bool Px4CtrlRosBridge::set_arm(const bool arm){
        if (px4_state_->state->value().first==nullptr){
            spdlog::error("px4 state is nullptr");
            return false;
        }
        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = arm;
        if( px4_state_->state->value().first->mode != mavros_msgs::msg::State::MODE_PX4_OFFBOARD){
            spdlog::error("Not in offboard mode, can't arm");
            return false;
        }
        if( arm==px4_state_->state->value().first->armed) {
            spdlog::info("Already in {} mode", arm?"ARM":"DISARM");
            return true;
        }
        auto future = px4_arming_client->async_send_request(request);
        if (future.wait_for(1s) != std::future_status::ready) {
            spdlog::error("Arm service call timed out");
            return false;
        }
        auto response = future.get();
        if (!response->success) {
            if ( arm )
                spdlog::error( "ARM rejected by PX4!" );
            else
                spdlog::error( "DISARM rejected by PX4!" );
            return false;
        }else{
            if ( arm )
                spdlog::info( "ARM accepted by PX4!" );
            else
                spdlog::info( "DISARM accepted by PX4!" );
            return true;
        }
        return true;
    }

    bool Px4CtrlRosBridge::force_disarm(){
        if (px4_state_->state->value().first==nullptr){
            spdlog::error("px4 state is nullptr");
            return false;
        }
        if( px4_state_->state->value().first->mode != mavros_msgs::msg::State::MODE_PX4_OFFBOARD){
            spdlog::error("Not in offboard mode, can't disarm");
            return false;
        }
        auto request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
        request->command = 400;
        request->param1 = 0;
        request->param2 = 21196;
        request->param3 = 0;
        request->param4 = 0;
        request->param5 = 0;
        request->param6 = 0;
        request->param7 = 0;
        auto future = px4_cmd_client->async_send_request(request);
        if (future.wait_for(1s) != std::future_status::ready) {
            spdlog::error("Disarm service call timed out");
            return false;
        }
        auto response = future.get();
        if (!response->success) {
            spdlog::error( "DISARM rejected by PX4!" );
            return false;
        }else{
            spdlog::info( "DISARM accepted by PX4!" );
            return true;
        }
        return true;
    }

    bool Px4CtrlRosBridge::pub_bodyrates_target(const double thrust, const std::array<double, 3>& bodyrates){//输入的油门应该是映射后的px4油门
        mavros_msgs::msg::AttitudeTarget msg;
        msg.header.stamp    =  rclcpp::Clock(RCL_ROS_TIME).now();
        msg.header.frame_id = std::string( "FCU" );

        msg.type_mask = mavros_msgs::msg::AttitudeTarget::IGNORE_ATTITUDE;

        msg.body_rate.x = bodyrates[0];
        msg.body_rate.y = bodyrates[1];
        msg.body_rate.z = bodyrates[2];

        msg.thrust = thrust;

        px4_cmd_pub->publish( msg );

        return true;
    }

    void Px4CtrlRosBridge::pub_allow_cmdctrl(bool allow){
        std_msgs::msg::Bool msg;
        msg.data = allow;
        allow_cmdctrl_pub->publish(msg);
        return;
    }

    bool Px4CtrlRosBridge::pub_attitude_target(const double thrust, const std::array<double, 4>quat){//w,x,y,z
        mavros_msgs::msg::AttitudeTarget msg;
        msg.header.stamp    = rclcpp::Clock(RCL_ROS_TIME).now();
        msg.header.frame_id = std::string( "FCU" );

        msg.type_mask = mavros_msgs::msg::AttitudeTarget::IGNORE_ROLL_RATE | mavros_msgs::msg::AttitudeTarget::IGNORE_PITCH_RATE | mavros_msgs::msg::AttitudeTarget::IGNORE_YAW_RATE;

        msg.orientation.w = quat[0];
        msg.orientation.x = quat[1];
        msg.orientation.y = quat[2];
        msg.orientation.z = quat[3];
        msg.thrust = thrust;//9.81==>Hover thrust=9.81

        px4_cmd_pub->publish( msg );

        return true;
    }
}