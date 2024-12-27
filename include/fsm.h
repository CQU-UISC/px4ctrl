#pragma once

#include <array>
#include <memory>
#include <spdlog/logger.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/sink.h>
#include <spdlog/spdlog.h>
#include <utility>

#include "bridge.h"
#include "datas.h"
#include "server.h"
#include "types.h"
#include "controller.h"

namespace px4ctrl {
/* 
    enum Px4CtrlState{
        NOT_CONNECTED,// 没有连接到PX4
        NON_OFFBOARD,//(目前PX4的状态不是OFFBOARD)
        UNARMED,// (MAVLINK手动ARMING==>通过UI)
        ARMED,//  (OFFBOARD&&ARMED)
        TAKING_OFF,//  (手动进入)
        HOVERING ,// (TAKING_OFF后自动进入，或者CMD_CTRL命令超时后进入，或者是地面站强制进入, 或者是Guard触发)
        ALLOW_CMD_CTRL,//  (在HOVERING下用户手动进入)
        CMD_CTRL,// （在ALLOW_CMD_CTRL收到命令后自动进入）
        LANDING ,// (UNARM，在HOVERING下由用户手动/Guard触发，执行成功后进入UNARMED)
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
    
    namespace  L2{
        struct L2IdleState{
            bool is_first_time = true;
            clock::time_point last_arm_time;
        };

        struct L2TakingOffState{
            Eigen::Vector3d start_pos;
            Eigen::Quaterniond start_q;
            clock::time_point last_takeoff_time;
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

    class Px4Ctrl{
        public:
            Px4Ctrl(
                std::shared_ptr<Px4CtrlRosBridge> px4_bridge,
                std::shared_ptr<Px4State> px4_state,
                std::shared_ptr<Px4CtrlParams> px4ctrl_params,
                std::shared_ptr<ui::Px4Server> px4_server);
            ~Px4Ctrl() = default;
            
            //run control loop
            void run();
            void stop();

            //interface
            std::array<const Px4CtrlState,3> get_px4_state() const;
            //force
            bool force_l2_state(const Px4CtrlState& state);

            std::pair<const Eigen::Vector3d, const Eigen::Quaterniond> get_hovering_pos() const;
            bool set_hovering_pos(const Eigen::Vector3d& pos, const Eigen::Quaterniond& q);

        private:
            bool ok=true;

            bool init();

            //process, main loop
            clock::time_point last_log_state_time;
            bool init_com=true;
            void process();
            void process_l0(controller::ControlCommand&);
            void process_l1(controller::ControlCommand&);
            void process_l2(controller::ControlCommand&);
            void proof_alive(controller::ControlCommand&);

            void guard();//监视状态（电池，速度，位置，姿态等
            bool faile_safe = false;

            //ctrl state
            LState L0,L1,L2;
            L2::L2IdleState L2idle;
            L2::L2TakingOffState L2takingoff;
            L2::L2HoveringState L2hovering;
            L2::L2AllowCmdCtrlState L2allow_cmd_ctrl;
            L2::L2LandingState L2landing;

            //px4 state
            std::shared_ptr<Px4CtrlRosBridge> px4_bridge;
            std::shared_ptr<Px4State> px4_state;
            std::shared_ptr<ui::Px4Server> px4_server;
            Px4DataObserver odom_hold, ctrl_hold, client_hold;

            clock::time_point last_client_cmd_time;
            ui::ServerPayload fill_server_payload();
            void client_command_callback(const ui::ClientPayload& payload);

            //controller
            std::shared_ptr<controller::Se3Control> controller;
            void apply_control(const controller::ControlCommand &cmd);
            
            // misc
            int odom_hz = 0;
            int cmdctrl_hz = 0;
            
            //config
            std::shared_ptr<Px4CtrlParams> px4ctrl_params;
    };
}//namepace px4ctrl