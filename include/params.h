#pragma once
#include <ostream>
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/exceptions.h>
#include <yaml-cpp/node/node.h>

#include "types.h"

namespace px4ctrl{

    namespace params {

    enum class ThrustMod{
        ESTIMATE,
        THRUSTMAP
    };
    
    inline ThrustMod thrustModFromString(const std::string& str){
        if(str == "ESTIMATE"){
            return ThrustMod::ESTIMATE;
        }else if(str == "THRUSTMAP"){
            return ThrustMod::THRUSTMAP;
        }else{
            spdlog::error("Invalid ThrustMod type:{}",str);
            throw std::runtime_error("Invalid ThrustMod type, must be ESTIMATE or THRUSTMAP, but got " + str);
        }
    }

    struct QuadrotorParams{
        double mass;//kg
        std::array<double,3> inertia;//kg*m^2
        double g;//m/s^2
        ThrustMod thrustmod;//估计推力 or 推力映射
        std::array<double,3> thrustmap; //推力映射 c2,c1,c0
        double init_hover_thrust; //初始悬停推力
        double max_thrust; //最大推力 无单位 <= 1
        double min_thrust; //最小推力 无单位 >= 0
        double max_bodyrate; //最大角速度 rad/s
    };

    enum class Guard{
        HOLD,
        LAND,
        DISARM
    };

    inline Guard guardFromString(const std::string& str){
        if(str == "HOLD"){
            return Guard::HOLD;
        }else if(str == "LAND"){
            return Guard::LAND;
        }else if(str == "DISARM"){
            return Guard::DISARM;
        }else{
            spdlog::error("Invalid Guard type:{}",str);
            throw std::runtime_error("Invalid Guard type, must be HOLD, LAND or DISARM, but got " + str);
        }
    }

    struct GuardParams{
        uint freq;
        uint land_timeout; //ms after this time, disarm

        uint mavros_timeout; //ms
        Guard mavros_triggered;

        uint odom_timeout; //ms
        uint odom_min_hz; //Hz
        Guard odom_triggered;

        uint ui_timeout; //ms
        Guard ui_triggered;
        
        double low_battery_voltage; //V
        Guard lowvolt_triggered;
    };

    struct StateMachineParams{
        uint freq;
        double l2_takeoff_height;//m
        double l2_idle_disarm_time;//ms
        double l2_cmd_ctrl_min_hz; //Hz
        double l2_takeoff_landing_speed; //m/s
        double l2_land_position_deviation_c; //m
        double l2_land_velocity_thr_c; //m/s
        double l2_land_time_keep_c; //ms
    };
    

    struct ControlParams{
        uint freq;
        double Kp_pos;
        double Kd_pos;
        double Ki_pos;
        double max_pos_error;
        double max_vel_error;
        double max_vel_int;

        controller::ControlType type;
        double Kw;
        double max_bodyrate_error;
    };

    }
    
    struct Px4CtrlParams{
        params::QuadrotorParams quadrotor_params;
        params::GuardParams guard_params;
        params::StateMachineParams statemachine_params;
        params::ControlParams control_params;

        inline static Px4CtrlParams load(const std::string& file){
            Px4CtrlParams params;
            try {
                YAML::Node config = YAML::LoadFile(file);
                if(!config["quadrotor"]){
                    spdlog::error("quadrotor params not found");
                    throw std::runtime_error("quadrotor params not found");
                }
                if(!config["guard"]){
                    spdlog::error("guard params not found");
                    throw std::runtime_error("guard params not found");
                }
                if(!config["px4ctrl"]){
                    spdlog::error("px4ctrl params not found");
                    throw std::runtime_error("px4ctrl params not found");
                }
                if(!config["controller"]){
                    spdlog::error("control params not found");
                    throw std::runtime_error("control params not found");
                }
                auto quadrotor = config["quadrotor"];
                if(!quadrotor["mass"] || !quadrotor["inertia"] || !quadrotor["g"] || !quadrotor["thrustmod"] || !quadrotor["thrustmap"] || !quadrotor["init_hover_thrust"] || !quadrotor["max_thrust"] || !quadrotor["min_thrust"] || !quadrotor["max_bodyrate"]){
                    spdlog::error("quadrotor params not complete");
                    throw std::runtime_error("quadrotor params not complete");
                }
                params.quadrotor_params.mass = quadrotor["mass"].as<double>();
                params.quadrotor_params.inertia = quadrotor["inertia"].as<std::array<double,3>>();
                params.quadrotor_params.g = quadrotor["g"].as<double>();
                params.quadrotor_params.thrustmod = params::thrustModFromString(quadrotor["thrustmod"].as<std::string>());
                params.quadrotor_params.thrustmap = quadrotor["thrustmap"].as<std::array<double,3>>();
                params.quadrotor_params.init_hover_thrust = quadrotor["init_hover_thrust"].as<double>();
                params.quadrotor_params.max_thrust = quadrotor["max_thrust"].as<double>();
                params.quadrotor_params.min_thrust = quadrotor["min_thrust"].as<double>();
                params.quadrotor_params.max_bodyrate = quadrotor["max_bodyrate"].as<double>();

                auto guard = config["guard"];
                if(!guard["freq"] || !guard["land_timeout"] || !guard["mavros_timeout"] || !guard["mavros_triggered"] || !guard["odom_timeout"] || !guard["odom_triggered"] || !guard["ui_timeout"] || !guard["ui_triggered"] || !guard["low_battery_voltage"] || !guard["lowvolt_triggered"]){
                    spdlog::error("guard params not complete");
                    throw std::runtime_error("guard params not complete");
                }
                params.guard_params.freq = guard["freq"].as<uint>();
                params.guard_params.land_timeout = guard["land_timeout"].as<uint>();
                params.guard_params.mavros_timeout = guard["mavros_timeout"].as<uint>();
                params.guard_params.mavros_triggered = params::guardFromString(guard["mavros_triggered"].as<std::string>());
                params.guard_params.odom_timeout = guard["odom_timeout"].as<uint>();
                params.guard_params.odom_min_hz = guard["odom_min_hz"].as<uint>();
                params.guard_params.odom_triggered = params::guardFromString(guard["odom_triggered"].as<std::string>());
                params.guard_params.ui_timeout = guard["ui_timeout"].as<uint>();
                params.guard_params.ui_triggered = params::guardFromString(guard["ui_triggered"].as<std::string>());
                params.guard_params.low_battery_voltage = guard["low_battery_voltage"].as<double>();
                params.guard_params.lowvolt_triggered = params::guardFromString(guard["lowvolt_triggered"].as<std::string>());

                auto px4ctrl = config["px4ctrl"];
                if(!px4ctrl["freq"] || !px4ctrl["l2_takeoff_height"] || !px4ctrl["l2_idle_disarm_time"] || !px4ctrl["l2_cmd_ctrl_min_hz"] || !px4ctrl["l2_takeoff_landing_speed"]){
                    spdlog::error("px4ctrl params not complete");
                    throw std::runtime_error("px4ctrl params not complete");
                }
                params.statemachine_params.freq = px4ctrl["freq"].as<uint>();
                params.statemachine_params.l2_takeoff_height = px4ctrl["l2_takeoff_height"].as<double>();
                params.statemachine_params.l2_idle_disarm_time = px4ctrl["l2_idle_disarm_time"].as<double>();
                params.statemachine_params.l2_cmd_ctrl_min_hz = px4ctrl["l2_cmd_ctrl_min_hz"].as<double>();
                params.statemachine_params.l2_takeoff_landing_speed = px4ctrl["l2_takeoff_landing_speed"].as<double>();
                params.statemachine_params.l2_land_position_deviation_c = px4ctrl["l2_land_position_deviation_c"].as<double>();
                params.statemachine_params.l2_land_velocity_thr_c = px4ctrl["l2_land_velocity_thr_c"].as<double>();
                params.statemachine_params.l2_land_time_keep_c = px4ctrl["l2_land_time_keep_c"].as<double>();


                auto control = config["controller"];
                if(!control["freq"] || !control["Kp_pos"] || !control["Kd_pos"] || !control["Ki_pos"] || !control["max_pos_error"] || !control["max_vel_error"] || !control["max_vel_int"] || !control["type"] || !control["Kw"] || !control["max_bodyrate_error"]){
                    spdlog::error("control params not complete");
                    throw std::runtime_error("control params not complete");
                }
                params.control_params.freq = control["freq"].as<uint>();
                params.control_params.Kp_pos = control["Kp_pos"].as<double>();
                params.control_params.Kd_pos = control["Kd_pos"].as<double>();
                params.control_params.Ki_pos = control["Ki_pos"].as<double>();
                params.control_params.max_pos_error = control["max_pos_error"].as<double>();
                params.control_params.max_vel_error = control["max_vel_error"].as<double>();    
                params.control_params.max_vel_int = control["max_vel_int"].as<double>();
                params.control_params.type = controller::controlTypeFromString(control["type"].as<std::string>());
                params.control_params.Kw = control["Kw"].as<double>();
                params.control_params.max_bodyrate_error = control["max_bodyrate_error"].as<double>();
            } catch (const YAML::BadFile& e) {
                spdlog::error("error:{}",e.what());
                throw e;
            } catch(const YAML::ParserException& e){
                spdlog::error("error:{}",e.what());
                throw e;
            }
            return params;
        }

        friend inline std::ostream &operator<<(std::ostream &os, const Px4CtrlParams& px4paras){
            os << "QuadrotorParams:" << std::endl;
            os << "mass:" << px4paras.quadrotor_params.mass << std::endl;
            os << "inertia:" << px4paras.quadrotor_params.inertia[0] << " " << px4paras.quadrotor_params.inertia[1] << " " << px4paras.quadrotor_params.inertia[2] << std::endl;
            os << "g:" << px4paras.quadrotor_params.g << std::endl;
            os << "thrustmod:" << static_cast<int>(px4paras.quadrotor_params.thrustmod) << std::endl;
            os << "thrustmap:" << px4paras.quadrotor_params.thrustmap[0] << " " << px4paras.quadrotor_params.thrustmap[1] << " " << px4paras.quadrotor_params.thrustmap[2] << std::endl;
            os << "init_hover_thrust:" << px4paras.quadrotor_params.init_hover_thrust << std::endl;
            os << "max_thrust:" << px4paras.quadrotor_params.max_thrust << std::endl;
            os << "min_thrust:" << px4paras.quadrotor_params.min_thrust << std::endl;
            os << "max_bodyrate:" << px4paras.quadrotor_params.max_bodyrate << std::endl;

            os << "GuardParams:" << std::endl;
            os << "freq:" << px4paras.guard_params.freq << std::endl;
            os << "land_timeout:" << px4paras.guard_params.land_timeout << std::endl;
            os << "mavros_timeout:" << px4paras.guard_params.mavros_timeout << std::endl;
            os << "mavros_triggered:" << static_cast<int>(px4paras.guard_params.mavros_triggered) << std::endl;
            os << "odom_timeout:" << px4paras.guard_params.odom_timeout << std::endl;
            os << "odom_triggered:" << static_cast<int>(px4paras.guard_params.odom_triggered) << std::endl;
            os << "ui_timeout:" << px4paras.guard_params.ui_timeout << std::endl;
            os << "ui_triggered:" << static_cast<int>(px4paras.guard_params.ui_triggered) << std::endl;
            os << "low_battery_voltage:" << px4paras.guard_params.low_battery_voltage << std::endl;
            os << "lowvolt_triggered:" << static_cast<int>(px4paras.guard_params.lowvolt_triggered) << std::endl;

            os << "Px4CtrlParams:" << std::endl;
            os << "freq:" << px4paras.statemachine_params.freq << std::endl;
            os << "l2_takeoff_height:" << px4paras.statemachine_params.l2_takeoff_height << std::endl;
            os << "l2_idle_disarm_time:" << px4paras.statemachine_params.l2_idle_disarm_time << std::endl;
            os << "l2_cmd_ctrl_min_hz:" << px4paras.statemachine_params.l2_cmd_ctrl_min_hz << std::endl;
            os << "l2_takeoff_landing_speed:" << px4paras.statemachine_params.l2_takeoff_landing_speed << std::endl;
            
            os << "ControlParams:" << std::endl;
            os << "freq:" << px4paras.control_params.freq << std::endl;
            os << "Kp_pos:" << px4paras.control_params.Kp_pos << std::endl;
            os << "Kd_pos:" << px4paras.control_params.Kd_pos << std::endl;
            os << "Ki_pos:" << px4paras.control_params.Ki_pos << std::endl;
            os << "max_pos_error:" << px4paras.control_params.max_pos_error << std::endl;
            os << "max_vel_error:" << px4paras.control_params.max_vel_error << std::endl;
            os << "max_vel_int:" << px4paras.control_params.max_vel_int << std::endl;
            os << "type:" << static_cast<int>(px4paras.control_params.type) << std::endl;
            os << "Kw:" << px4paras.control_params.Kw << std::endl;
            os << "max_bodyrate_error:" << px4paras.control_params.max_bodyrate_error << std::endl;
            return os;
        }

        operator std::string() const{
            std::stringstream ss;
            ss << *this;
            return ss.str();
        }

    };
}