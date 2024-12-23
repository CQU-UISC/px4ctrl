#pragma once

#include "px4ctrl_bridge.h"
#include "px4ctrl_def.h"
#include "px4ctrl_fsm.h"
#include <deque>
#include <ftxui/component/component.hpp>
#include <ftxui/component/loop.hpp>
#include <ftxui/component/screen_interactive.hpp>
#include <ftxui/dom/node.hpp>
#include <ftxui/screen/screen.hpp>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>

#include <ftxui/dom/elements.hpp>
#include <spdlog/sinks/base_sink.h>
#include <string>
#include <thread>
#include <vector>


namespace px4ctrl {

    template<typename Mutex>
    class ftxui_sink : public spdlog::sinks::base_sink<Mutex>{
    public:
        inline std::vector<std::string> getLog() { 
            std::lock_guard<Mutex> lock(this->mutex_);
            std::vector<std::string> log(_log.begin(),_log.end());
            _log.clear();
            return log;
        };
    protected:
        void sink_it_(const spdlog::details::log_msg& msg) override{
            spdlog::memory_buf_t formatted;
            spdlog::sinks::base_sink<Mutex>::formatter_->format(msg, formatted);

            std::stringstream ss;
            ss.write(formatted.data(), static_cast<std::streamsize>(formatted.size()));
            _log.push_back(ss.str());
            while(_log.size()>50){
                _log.pop_front();
            }
        };
        void flush_() override {};

        std::deque<std::string> _log;
    };

    using ftxui_sink_mt = ftxui_sink<std::mutex>;
    using ftxui_sink_st = ftxui_sink<spdlog::details::null_mutex>;

    class Px4CtrlUI{
        using onClick = std::function<void()>;
        public:
            Px4CtrlUI(std::shared_ptr<ftxui_sink_mt>, 
            std::shared_ptr<Px4State>, 
            std::shared_ptr<Px4CtrlRosBridge>,
            std::shared_ptr<Px4Ctrl>) noexcept;
            ~Px4CtrlUI();
            void start();
        
        private:
            std::shared_ptr<ftxui_sink_mt> _sink;
            std::shared_ptr<Px4State> _px4_state;
            std::shared_ptr<Px4CtrlRosBridge> _px4_bridge;
            std::shared_ptr<Px4Ctrl> _px4_ctrl;
            std::thread _ui_thread,_data_thread;
            
            //ui
            ftxui::ScreenInteractive _screen;
            std::shared_ptr<ftxui::Loop> _loop_ptr;
            ftxui::Component render();
            ftxui::Element wrap(const std::string& title,ftxui::Element);
            //callback functions
            ftxui::Component container;
            std::vector<std::string> log_list;
            int log_menu;
            std::map<std::string,onClick> _callbacks;
            onClick force_disarm,arm,disarm,enter_offboard,exit_offboard,takeoff,land,force_hover,allow_cmd_ctrl;
            void setup_callbacks();

            void data_thread();
            void ui_thread();

            //data
            Px4DataObserver odom_hold, cmdctrl_hold;
            Eigen::Vector3d _pos,_vel,_omega;
            Eigen::Quaterniond _quat;
            double battery_voltage;
            int odom_count,cmdctrl_count;
            int odom_hz,cmdctrl_hz;
            clock::time_point _last_odom_time,_last_cmdctrl_time;

    };
}// namespace px4ctrl