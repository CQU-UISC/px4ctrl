#include "bridge.h"
#include "types.h"
#include <array>
#include <functional>
#include <px4_msgs/msg/detail/vehicle_command__struct.hpp>
#include <px4_msgs/msg/detail/vehicle_command_ack__struct.hpp>
#include <px4_msgs/srv/detail/vehicle_command__struct.hpp>
#include <spdlog/spdlog.h>

namespace px4ctrl {

    Px4CtrlRosBridge::Px4CtrlRosBridge(rclcpp::Node::SharedPtr node,std::shared_ptr<Px4State> px4_state)
        :node(node),px4_state_(px4_state){
        load_params();
        // Subscribers
        px4_state_sub = this->node->create_subscription<mavros_msgs::msg::State>(
            px4_state_sub_topic, 10, build_px4ros_cb(px4_state_->state));
        px4_extended_state_sub = this->node->create_subscription<mavros_msgs::msg::ExtendedState>(
            px4_extended_state_sub_topic, 10, build_px4ros_cb(px4_state_->ext_state));
        ext_odom_sub = this->node->create_subscription<nav_msgs::msg::Odometry>(
            ext_odom_sub_topic, rclcpp::SensorDataQoS(), build_px4ros_cb(px4_state_->odom));
        ctrl_cmd_sub = this->node->create_subscription<px4ctrl_msgs::msg::Command>(
            ctrl_cmd_sub_topic, rclcpp::SensorDataQoS(), build_px4ros_cb(px4_state_->ctrl_command));
        imu_sub = this->node->create_subscription<sensor_msgs::msg::Imu>(
            imu_sub_topic, rclcpp::SensorDataQoS(), build_px4ros_cb(px4_state_->imu));
        bat_sub = this->node->create_subscription<sensor_msgs::msg::BatteryState>(
            bat_sub_topic, rclcpp::SensorDataQoS(), build_px4ros_cb(px4_state_->battery));
        // Defining the compatible ROS 2 predefined QoS for PX4 topics
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(
            rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);        
        vehicle_odometry_sub = this->node->create_subscription<px4_msgs::msg::VehicleOdometry>(
            vehicle_odometry_sub_topic, qos,std::bind(&Px4CtrlRosBridge::vehicle_odom_callback, this, std::placeholders::_1));
        vehicle_status_sub = this->node->create_subscription<px4_msgs::msg::VehicleStatus>(
            vehicle_status_sub_topic, qos, build_px4ros_cb(px4_state_->vehicle_status));
        // Publishers
        attitude_setpoint_pub = this->node->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
            attitude_setpoint_pub_topic, 10);  
        actuator_motors_pub = this->node->create_publisher<px4_msgs::msg::ActuatorMotors>(
            actuator_motors_pub_topic, 10);
        offboard_control_mode_pub = this->node->create_publisher<px4_msgs::msg::OffboardControlMode>(
            offboard_control_mode_pub_topic, 10);
        thrust_setpoint_pub = this->node->create_publisher<px4_msgs::msg::VehicleThrustSetpoint>(
            thrust_setpoint_pub_topic, 10);
        torque_setpoint_pub = this->node->create_publisher<px4_msgs::msg::VehicleTorqueSetpoint>(
            torque_setpoint_pub_topic, 10);
        allow_cmdctrl_pub = this->node->create_publisher<std_msgs::msg::Bool>(
            allow_cmdctrl_pub_topic, 10);
        vehicle_odometry_pub = this->node->create_publisher<nav_msgs::msg::Odometry>(
            vehicle_odometry_pub_topic, 10);
        // Service clients
        px4_set_mode_client = this->node->create_client<mavros_msgs::srv::SetMode>(px4_set_mode_client_topic);
        px4_arming_client = this->node->create_client<mavros_msgs::srv::CommandBool>(px4_arming_client_topic);
        px4_cmd_client = this->node->create_client<mavros_msgs::srv::CommandLong>(px4_cmd_client_topic);
        px4_vehicle_command_client = this->node->create_client<px4_msgs::srv::VehicleCommand>(vehicle_command_client_topic);
        spdlog::info("Init px4ros node");
        return;
    }

    void Px4CtrlRosBridge::vehicle_odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg){
        nav_msgs::msg::Odometry odom;
        vehicle_odom_from_px4_to_ros(*msg, odom);
        vehicle_odometry_pub->publish(odom);
    }

    void Px4CtrlRosBridge::vehicle_odom_from_px4_to_ros(const px4_msgs::msg::VehicleOdometry& msg, nav_msgs::msg::Odometry& odom){
        Eigen::Vector3d position;
        if(msg.pose_frame==msg.POSE_FRAME_FRD){
            position = rotateVectorFromToFRD_FLU(Eigen::Vector3d(msg.position[0], msg.position[1], msg.position[2]));
        }else if (msg.pose_frame==msg.POSE_FRAME_NED) {
            position = rotateVectorFromToENU_NED(Eigen::Vector3d(msg.position[0], msg.position[1], msg.position[2]));
        }
        else if (msg.pose_frame==msg.POSE_FRAME_UNKNOWN) {
            spdlog::error("Unknown pose frame");
            return;
        }

        Eigen::Vector3d velocity;
        Eigen::Quaterniond quat = rotateQuaternionFromToENU_NED(
            Eigen::Quaterniond(msg.q[0], msg.q[1], msg.q[2], msg.q[3]));
        Eigen::Vector3d angular_velocity = rotateVectorFromToFRD_FLU(
            Eigen::Vector3d(msg.angular_velocity[0], msg.angular_velocity[1], msg.angular_velocity[2]));
        
        if(msg.velocity_frame==msg.VELOCITY_FRAME_NED){
            velocity = rotateVectorFromToENU_NED(
                Eigen::Vector3d(msg.velocity[0], msg.velocity[1], msg.velocity[2])
            );
        }
        else if(msg.velocity_frame==msg.VELOCITY_FRAME_FRD){
            velocity = rotateVectorFromToFRD_FLU(
                Eigen::Vector3d(msg.velocity[0], msg.velocity[1], msg.velocity[2])
            );
        }
        else if(msg.velocity_frame==msg.VELOCITY_FRAME_BODY_FRD){
            velocity = rotateVectorFromToFRD_FLU(
                Eigen::Vector3d(msg.velocity[0], msg.velocity[1], msg.velocity[2])
            );
            // convert to world frame
            velocity = quat*velocity;
            return;
        }
        else if(msg.velocity_frame==msg.VELOCITY_FRAME_UNKNOWN){
            spdlog::error("Unknown velocity frame");
            return;
        }
        
        odom.header.stamp.set__nanosec(msg.timestamp*1000);
        odom.header.frame_id = "map";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = position[0];
        odom.pose.pose.position.y = position[1];
        odom.pose.pose.position.z = position[2];
        odom.pose.pose.orientation.w = quat.w();
        odom.pose.pose.orientation.x = quat.x();
        odom.pose.pose.orientation.y = quat.y();
        odom.pose.pose.orientation.z = quat.z();
        odom.twist.twist.linear.x = velocity[0];
        odom.twist.twist.linear.y = velocity[1];
        odom.twist.twist.linear.z = velocity[2];
        odom.twist.twist.angular.x = angular_velocity[0];
        odom.twist.twist.angular.y = angular_velocity[1];
        odom.twist.twist.angular.z = angular_velocity[2];
    }

    void Px4CtrlRosBridge::spin_once(){
        rclcpp::spin_some(this->node->get_node_base_interface());
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

    bool Px4CtrlRosBridge::restart_fcu(){
        // MAVROS version
        // auto request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
        // request->confirmation = true;
        // request->command = 246;
        // request->param1 = 1;
        // request->param2 = 0;
        // request->param3 = 0;
        // request->param4 = 0;
        // request->param5 = 0;
        // request->param6 = 0;
        // request->param7 = 0;
        // auto future = px4_cmd_client->async_send_request(request);
        // if (future.wait_for(1s) != std::future_status::ready) {
        //     spdlog::error("restart fcu service call timed out");
        //     return false;
        // }
        // auto response = future.get();
        // if (!response->success) {
        //     spdlog::error( "restart fcu rejected by PX4!" );
        //     return false;
        // }else{
        //     spdlog::info( "restart fcu accepted by PX4!" );
        //     return true;
        // }
        // return true;
        auto request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();
        request->request.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
        request->request.param1 = 1;
        auto future = px4_vehicle_command_client->async_send_request(request);
        if (future.wait_for(1s) != std::future_status::ready) {
            spdlog::error("restart fcu service call timed out");
            return false;
        }
        auto response = future.get();
        if (response->reply.result!=response->reply.VEHICLE_CMD_RESULT_ACCEPTED){
            spdlog::error( "restart fcu rejected by PX4!");
            return false;
        }else{
            spdlog::info( "restart fcu accepted by PX4!" );
            return true;
        }
        return true;
    }

    void Px4CtrlRosBridge::pub_allow_cmdctrl(bool allow){
        std_msgs::msg::Bool msg;
        msg.data = allow;
        allow_cmdctrl_pub->publish(msg);
        return;
    }

    void Px4CtrlRosBridge:: pub_bodyrates_target(const double thrust, const Eigen::Vector3d& bodyrates){//输入的油门应该是映射后的px4油门
        pub_offboard_control_mode_msg(controller::ControlType::BODY_RATES);
        px4_msgs::msg::VehicleAttitudeSetpoint attitude_setpoint_msg;
        attitude_setpoint_msg.timestamp = this->node->get_clock()->now().nanoseconds() / 1000;
        // Rotate bodyrates setpoints from FLU to FRD and fill the msg
        Eigen::Vector3d rotated_bodyrates_sp;
        rotated_bodyrates_sp = rotateVectorFromToFRD_FLU(bodyrates);
        attitude_setpoint_msg.roll_body = bodyrates.x();
        attitude_setpoint_msg.pitch_body = bodyrates.y();
        attitude_setpoint_msg.yaw_body = bodyrates.z();
        if (thrust > 0.1){
            attitude_setpoint_msg.thrust_body[0] = 0.0;
            attitude_setpoint_msg.thrust_body[1] = 0.0;
            attitude_setpoint_msg.thrust_body[2] = -thrust;         // DO NOT FORGET THE MINUS SIGN (body NED frame)
        }
        else {
            attitude_setpoint_msg.thrust_body[2] = -0.1;
        }

        attitude_setpoint_pub->publish(attitude_setpoint_msg);
        return;
    }

    void Px4CtrlRosBridge::pub_attitude_target(const double thrust, const Eigen::Quaterniond&  quat){//w,x,y,z
        pub_offboard_control_mode_msg(controller::ControlType::ATTITUDE);
        px4_msgs::msg::VehicleAttitudeSetpoint attitude_setpoint_msg;
        attitude_setpoint_msg.timestamp = this->node->get_clock()->now().nanoseconds() / 1000;
        Eigen::Quaterniond rotated_quat;
        rotated_quat = rotateQuaternionFromToENU_NED(quat);
        attitude_setpoint_msg.q_d[0] = rotated_quat.w();
        attitude_setpoint_msg.q_d[1] = rotated_quat.x();
        attitude_setpoint_msg.q_d[2] = rotated_quat.y();
        attitude_setpoint_msg.q_d[3] = rotated_quat.z();

        if (thrust > 0.1){
            attitude_setpoint_msg.thrust_body[0] = 0.0;
            attitude_setpoint_msg.thrust_body[1] = 0.0;
            attitude_setpoint_msg.thrust_body[2] = -thrust;         // DO NOT FORGET THE MINUS SIGN (body NED frame)
        }
        else {
            attitude_setpoint_msg.thrust_body[2] = -0.1;
        }

        attitude_setpoint_pub->publish(attitude_setpoint_msg);
        return;
    }

    void Px4CtrlRosBridge::pub_torque_target(const double thrust, const Eigen::Vector3d& torque){
        pub_offboard_control_mode_msg(controller::ControlType::TORQUE);
        // Lockstep should be disabled from PX4 and from the model.sdf file
        // Prepare msgs
        px4_msgs::msg::VehicleThrustSetpoint thrust_sp_msg;
        px4_msgs::msg::VehicleTorqueSetpoint torque_sp_msg;
        thrust_sp_msg.timestamp_sample = this->node->get_clock()->now().nanoseconds() / 1000;
        torque_sp_msg.timestamp_sample = thrust_sp_msg.timestamp_sample ;
        thrust_sp_msg.timestamp = thrust_sp_msg.timestamp_sample ;
        torque_sp_msg.timestamp = thrust_sp_msg.timestamp_sample ;
        // Fill thrust setpoint msg
        thrust_sp_msg.xyz[0] = 0.0;
        thrust_sp_msg.xyz[1] = 0.0;
        if (thrust > 0.1){
            thrust_sp_msg.xyz[2] = -thrust;         // DO NOT FORGET THE MINUS SIGN (body NED frame)
        }
        else {
            thrust_sp_msg.xyz[2] = -0.1;
        }
        // Rotate torque setpoints from FLU to FRD and fill the msg
        Eigen::Vector3d rotated_torque_sp;
        rotated_torque_sp = rotateVectorFromToFRD_FLU(torque);
        torque_sp_msg.xyz[0] = rotated_torque_sp[0];
        torque_sp_msg.xyz[1] = rotated_torque_sp[1];
        torque_sp_msg.xyz[2] = rotated_torque_sp[2];

        // Publish msgs
        thrust_setpoint_pub->publish(thrust_sp_msg);
        torque_setpoint_pub->publish(torque_sp_msg);
        return;
    }

    void Px4CtrlRosBridge::pub_actuator_target(const Eigen::Vector4d& motors){
        pub_offboard_control_mode_msg(controller::ControlType::ROTOR_THRUST);
        // Lockstep should be disabled from PX4 and from the model.sdf file
        // direct motor throttles control
        // Prepare msg
        std::array<double, 4> throttles = {
            motors.x(),motors.y(),motors.z(),motors.w()
        };
        px4_msgs::msg::ActuatorMotors actuator_motors_msg;
        actuator_motors_msg.control = { (float) throttles[0], (float) throttles[1], (float) throttles[2], (float) throttles[3], 
                                std::nanf("1"), std::nanf("1"), std::nanf("1"), std::nanf("1"),
                                std::nanf("1"), std::nanf("1"), std::nanf("1"), std::nanf("1")};
        actuator_motors_msg.reversible_flags = 0;
        actuator_motors_msg.timestamp = this->node->get_clock()->make_shared()->now().nanoseconds() / 1000;
        actuator_motors_msg.timestamp_sample = actuator_motors_msg.timestamp;

        actuator_motors_pub->publish(actuator_motors_msg);
        return;
    }

    void Px4CtrlRosBridge::pub_offboard_control_mode_msg(const controller::ControlType type){
        px4_msgs::msg::OffboardControlMode offboard_msg{};
        offboard_msg.position = false;
        offboard_msg.velocity = false;
        offboard_msg.acceleration = false;
        switch (type)
        {
            case controller::ControlType::BODY_RATES:
                offboard_msg.body_rate = true;
                offboard_msg.attitude = false;
                offboard_msg.thrust_and_torque = false;
                offboard_msg.direct_actuator = false;
                break;
            case controller::ControlType::ATTITUDE:
                offboard_msg.body_rate = false;
                offboard_msg.attitude = true;
                offboard_msg.thrust_and_torque = false;
                offboard_msg.direct_actuator = false;
                break;
            case controller::ControlType::TORQUE:
                offboard_msg.body_rate = false;
                offboard_msg.attitude = false;
                offboard_msg.thrust_and_torque = true;
                offboard_msg.direct_actuator = false;
                break;
            case  controller::ControlType::ROTOR_THRUST:
                offboard_msg.body_rate = false;
                offboard_msg.attitude = false;
                offboard_msg.thrust_and_torque = false;
                offboard_msg.direct_actuator = true;
                break;
            default:
                offboard_msg.body_rate = false;
                offboard_msg.attitude = true;
                offboard_msg.thrust_and_torque = false;
                offboard_msg.direct_actuator = false;
                break;
        }
        offboard_msg.timestamp = this->node->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_pub->publish(offboard_msg);
    }

    void Px4CtrlRosBridge::load_params(){
         // Topics Names
         // Sub
        this->node->declare_parameter("topics_names.vehicle_status_sub_topic", "fmu/out/vehicle_status");
        this->node->declare_parameter("topics_names.vehicle_odometry_sub_topic", "fmu/out/vehicle_odometry");
        this->node->declare_parameter("topics_names.px4_state_sub_topic", "/mavros/state");
        this->node->declare_parameter("topics_names.px4_extended_state_sub_topic", "/mavros/extended_state");
        this->node->declare_parameter("topics_names.imu_sub_topic", "/mavros/imu/data");
        this->node->declare_parameter("topics_names.bat_sub_topic", "/mavros/battery");
        this->node->declare_parameter("topics_names.ext_odom_sub_topic", "/px4ctrl/ext_odom");
        this->node->declare_parameter("topics_names.ctrl_cmd_sub_topic", "/px4ctrl/ctrl_cmd");
        // Pub
        this->node->declare_parameter("topics_names.actuator_motors_pub_topic", "/fmu/in/actuator_motors");
        this->node->declare_parameter("topics_names.attitude_setpoint_pub_topic", "/fmu/in/vehicle_attitude_setpoint");
        this->node->declare_parameter("topics_names.thrust_setpoint_pub_topic", "/fmu/in/vehicle_thrust_setpoint");
        this->node->declare_parameter("topics_names.torque_setpoint_pub_topic", "/fmu/in/vehicle_torque_setpoint");
        this->node->declare_parameter("topics_names.offboard_control_mode_pub_topic", "/fmu/in/offboard_control_mode");
        this->node->declare_parameter("topics_names.allow_cmdctrl_pub_topic", "/px4ctrl/allow_cmd_ctrl");
        this->node->declare_parameter("topics_names.vehicle_odometry_pub_topic", "/px4ctrl/vehicle_odometry");
        // Client
        this->node->declare_parameter("topics_names.px4_set_mode_client_topic", "/mavros/set_mode");
        this->node->declare_parameter("topics_names.px4_arming_client_topic", "/mavros/cmd/arming");
        this->node->declare_parameter("topics_names.px4_cmd_client_topic", "/mavros/cmd/command");
        this->node->declare_parameter("topics_names.vehicle_command_client_topic", "/fmu/vehicle_command");
        // Sub
        vehicle_status_sub_topic = this->node->get_parameter("topics_names.vehicle_status_sub_topic").as_string();
        vehicle_odometry_sub_topic = this->node->get_parameter("topics_names.vehicle_odometry_sub_topic").as_string();
        px4_state_sub_topic = this->node->get_parameter("topics_names.px4_state_sub_topic").as_string();
        px4_extended_state_sub_topic = this->node->get_parameter("topics_names.px4_extended_state_sub_topic").as_string();
        imu_sub_topic = this->node->get_parameter("topics_names.imu_sub_topic").as_string();
        bat_sub_topic = this->node->get_parameter("topics_names.bat_sub_topic").as_string();
        ext_odom_sub_topic = this->node->get_parameter("topics_names.ext_odom_sub_topic").as_string();
        ctrl_cmd_sub_topic = this->node->get_parameter("topics_names.ctrl_cmd_sub_topic").as_string();
        
        // Pub
        vehicle_odometry_pub_topic = this->node->get_parameter("topics_names.vehicle_odometry_pub_topic").as_string();
        actuator_motors_pub_topic = this->node->get_parameter("topics_names.actuator_motors_pub_topic").as_string();
        attitude_setpoint_pub_topic = this->node->get_parameter("topics_names.attitude_setpoint_pub_topic").as_string();
        thrust_setpoint_pub_topic = this->node->get_parameter("topics_names.thrust_setpoint_pub_topic").as_string();
        torque_setpoint_pub_topic = this->node->get_parameter("topics_names.torque_setpoint_pub_topic").as_string();
        offboard_control_mode_pub_topic = this->node->get_parameter("topics_names.offboard_control_mode_pub_topic").as_string();
        allow_cmdctrl_pub_topic = this->node->get_parameter("topics_names.allow_cmdctrl_pub_topic").as_string();
      
        // client
        px4_set_mode_client_topic = this->node->get_parameter("topics_names.px4_set_mode_client_topic").as_string();
        px4_arming_client_topic = this->node->get_parameter("topics_names.px4_arming_client_topic").as_string();
        px4_cmd_client_topic = this->node->get_parameter("topics_names.px4_cmd_client_topic").as_string();
        vehicle_command_client_topic = this->node->get_parameter("topics_names.vehicle_command_client_topic").as_string();
    }
}