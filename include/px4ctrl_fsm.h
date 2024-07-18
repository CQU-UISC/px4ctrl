#pragma once

#include <cstdint>
#include <memory>
#include <spdlog/logger.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/sink.h>
#include <string>
#include <spdlog/spdlog.h>

#include "px4ctrl_gcs.h"
#include "px4ctrl_se3_controller.h"
#include "px4ctrl_mavros.h"
#include "px4ctrl_state.h"
#include "px4ctrl_def.h"

namespace px4ctrl {
/* 
    enum Px4CtrlState{
        NOT_CONNECTED,// 没有连接到PX4
        NON_OFFBOARD,//(目前PX4的状态不是OFFBOARD)
        UNARMED,// (MAVLINK手动ARMING==>通过GCS)
        ARMED,//  (OFFBOARD&&ARMED)
        TAKING_OFF,//  (手动进入)
        HOVERING ,// (TAKING_OFF后自动进入，或者CMD_CTRL命令超时后进入，或者是地面站强制进入, 或者是Guard触发)
        ALLOW_CMD_CTRL,//  (在HOVERING下用户手动进入)
        CMD_CTRL,// （在ALLOW_CMD_CTRL收到命令后自动进入）
        LANDING ,// (UNARM，在HOVERING下由用户手动/Guard触发，执行成功后进入UNARMED)
        DEADLOCK,// 死🔓
    }; 
*/
    struct LState{
        Px4CtrlState state,last_state,next_state;
        
        //下一次调用step后，state变为next_state
        LState& operator=(const Px4CtrlState& rhs) noexcept{
            if(state==rhs){
                return *this;
            }

            // last_state = state;
            next_state = rhs;
            return *this;
        }

        LState& reset( const Px4CtrlState& rhs) noexcept{
            state = rhs;
            last_state = rhs;
            next_state = rhs;
            return *this;
        }

        LState& step(){
            last_state = state;
            state = next_state;
            return *this;
        }
    };

    inline bool operator==(const LState& lhs, const LState& rhs){
        return lhs.state == rhs.state;
    }
    

    struct L0State:public LState{
        using LState::operator=;
    };

    struct L1State:public LState{
        using LState::operator=;
    };


    namespace  L2{
        struct L2IdleState{
            bool is_first_time = true;
            clock::time_point last_arm_time;
        };

        struct L2TakingOffState{
            Eigen::Vector3d start_pos;
            Eigen::Quaterniond start_q;
        };

        struct L2HoveringState{
            Eigen::Vector3d des_pos;
            Eigen::Quaterniond des_q;
        };

        struct L2AllowCmdCtrlState{
            Eigen::Vector3d hovering_pos;
            Eigen::Quaterniond hovering_q;
        };

        struct L2LandingState{
            Eigen::Vector3d start_pos;
            Eigen::Quaterniond start_q;
            clock::time_point start_time;
            bool is_first_time = true;
            clock::time_point time_C12_reached; // time_Constraints12_reached
        };
    }


    struct L2State:public LState{
        L2::L2IdleState idle;
        L2::L2TakingOffState takingoff;
        L2::L2HoveringState hovering;
        L2::L2AllowCmdCtrlState allow_cmd_ctrl;
        L2::L2LandingState landing;

        using LState::operator=;
    };

    enum class GuardStatus{
        OK,
        MAVROS_TIMEOUT,
        ODOM_TIMEOUT,
        GCS_TIMEOUT,
        LOW_VOLTAGE,
    };

    struct GuardParams{
        double mavros_timeout;
        double odom_timeout;
        double gcs_timeout;
        double low_battery_voltage;
        //TODO,处理超速，姿态等
    };

    struct Px4CtrlParams{
        uint freq;
        double l2_takeoff_height;
        double l2_idle_disarm_time;//在这个时间内没有收到用户命令将会disarm, seconds
        double l2_allow_cmd_ctrl_allow_time;//cmd在这个时间内到达将会进入cmd_ctrl, milliseconds
        double l2_cmd_ctrl_cmd_timeout;//cmd_ctrl在这个时间内没有收到命令将会进入hovering, milliseconds
        double l2_takeoff_landing_speed;//m/s

        double max_thrust;
        double min_thrust;
        double max_bodyrate;
        controller::ControlParams control_params;
        uint8_t drone_id,gcs_id,version;//load from config

        //guard
        GuardParams guard_params;
    };

    class Px4Ctrl{
        public:
            Px4Ctrl(std::shared_ptr<PX4CTRL_ROS_BRIDGE> px4_ros, std::shared_ptr<PX4_STATE> px4_state, std::shared_ptr<gcs::DroneCom> drone_com);
            Px4Ctrl(std::string base_dir, std::shared_ptr<PX4CTRL_ROS_BRIDGE> px4_ros, std::shared_ptr<PX4_STATE> px4_state,std::shared_ptr<gcs::DroneCom> drone_com);
            ~Px4Ctrl() = default;
            
            //run control loop
            void run();
            void stop();

        private:
            bool ok=true;

            bool init();
            bool load_config();

            //process, main loop
            clock::time_point last_log_state_time;
            bool init_com=true;
            void process();
            void process_l0(controller::ControlCommand&);
            void process_l1(controller::ControlCommand&);
            void process_l2(controller::ControlCommand&);
            void proof_alive(controller::ControlCommand&);

            GuardStatus guard();//监视状态（电池，速度，位置，姿态等）&& gcs && px4, true 代表触发

            //ctrl state
            L0State L0;
            L1State L1;
            L2State L2;

            //px4 state
            std::shared_ptr<PX4CTRL_ROS_BRIDGE> px4_mavros;
            std::shared_ptr<PX4_STATE> px4_state;

            //controller
            std::shared_ptr<controller::LinearControl> controller;
            void apply_control(const controller::ControlCommand &cmd);
            

            //config
            Px4CtrlParams params;

            //logging
            //
            std::shared_ptr<spdlog::sinks::stdout_color_sink_mt> console_sink;
            std::shared_ptr<spdlog::sinks::basic_file_sink_mt>  file_sink;
            std::shared_ptr<spdlog::logger> logger_ptr;

            //base directory of logging and config
            std::string base_dir;

            //GCS
            std::shared_ptr<gcs::DroneCom> drone_com;
            void fill_drone_message();
            gcs::Drone drone_message;
            clock::time_point last_gcs_time;
            gcs::Gcs gcs_message;
    };

    class PositionController;

}//namepace px4ctrl