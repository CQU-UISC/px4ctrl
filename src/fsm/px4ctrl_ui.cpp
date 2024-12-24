#include "px4ctrl_ui.h"
#include "px4ctrl_bridge.h"
#include "px4ctrl_def.h"

#include <ftxui/component/component.hpp>
#include <ftxui/component/component_base.hpp>
#include <ftxui/component/event.hpp>
#include <ftxui/component/task.hpp>
#include <ftxui/dom/elements.hpp>
#include <ftxui/dom/node.hpp>
#include <ftxui/screen/screen.hpp>
#include <functional>
#include <memory>
#include <nav_msgs/Odometry.h>
#include <string>
#include <vector>

namespace px4ctrl
{

Px4CtrlUI::Px4CtrlUI(std::shared_ptr<ftxui_sink_mt> sink,
                     std::shared_ptr<Px4State> px4_state,
                     std::shared_ptr<Px4CtrlRosBridge> px4_bridge,
                     std::shared_ptr<Px4Ctrl> px4_ctrl) noexcept
    : _sink(sink), _px4_state(px4_state), _px4_bridge(px4_bridge),
      _px4_ctrl(px4_ctrl), _screen(ftxui::ScreenInteractive::Fullscreen())
{
    odom_count = 0;
    cmdctrl_count = 0;
    odom_hz = 0;
    cmdctrl_hz = 0;
    battery_voltage = 0;
    _pos = Eigen::Vector3d::Zero();
    _vel = Eigen::Vector3d::Zero();
    _omega = Eigen::Vector3d::Zero();
    _quat = Eigen::Quaterniond::Identity();
    _last_odom_time = clock::now();
    _last_cmdctrl_time = clock::now();

    odom_hold = px4_state->odom->observe(
        [&](const auto& data)
        {
            nav_msgs::OdometryConstPtr odom = data.first;
            _pos = Eigen::Vector3d(odom->pose.pose.position.x,
                                   odom->pose.pose.position.y,
                                   odom->pose.pose.position.z);
            _vel = Eigen::Vector3d(odom->twist.twist.linear.x,
                                   odom->twist.twist.linear.y,
                                   odom->twist.twist.linear.z);
            _omega = Eigen::Vector3d(odom->twist.twist.angular.x,
                                     odom->twist.twist.angular.y,
                                     odom->twist.twist.angular.z);
            _quat = Eigen::Quaterniond(
                odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
            if (timePassedSeconds(_last_odom_time) > 1)
            {
                _last_odom_time = clock::now();
                odom_hz = odom_count;
                odom_count = 0;
            }
            else
            {
                odom_count++;
            }
        });

    cmdctrl_hold = px4_state->ctrl_command->observe(
        [&](const auto& data)
        {
            if (timePassedSeconds(_last_cmdctrl_time) > 1)
            {
                _last_cmdctrl_time = clock::now();
                cmdctrl_hz = cmdctrl_count;
                cmdctrl_count = 0;
            }
            else
            {
                cmdctrl_count++;
            }
        });
    setup_callbacks();
    _loop_ptr = std::make_shared<ftxui::Loop>(&this->_screen, render());
}

void Px4CtrlUI::start()
{
    _ui_thread = std::thread(&Px4CtrlUI::ui_thread, this);
    _data_thread = std::thread(&Px4CtrlUI::data_thread, this);
}

void Px4CtrlUI::setup_callbacks()
{
    auto build_cb = [](std::function<void()> cb)
    {
        return [cb]()
        {
            try {
             cb();
            } catch (std::exception& e) {
                spdlog::error("Exception: {}", e.what());
            }
           
        };
    };

    force_disarm = [&]()
    {
        if (!_px4_bridge->force_disarm())
        {
            spdlog::error("force disarm failed");
        }
    };

    arm = [&]()
    {
        if (!_px4_bridge->set_arm(true))
        {
            spdlog::error("arm failed");
        }
    };

    disarm = [&]()
    {
        if (!_px4_bridge->set_arm(false))
        {
            spdlog::error("disarm failed");
        }
    };

    enter_offboard = [&]()
    {
        if (!_px4_bridge->enter_offboard())
        {
            spdlog::error("enter offboard failed");
        }
    };

    exit_offboard = [&]()
    {
        if (!_px4_bridge->exit_offboard())
        {
            spdlog::error("exit offboard failed");
        }
    };

    takeoff = [&]()
    {
        auto px4ctrl_fsm_state = _px4_ctrl->get_px4_state();
        if (px4ctrl_fsm_state[2] == Px4CtrlState::L2_IDLE)
        {
            _px4_ctrl->force_l2_state(Px4CtrlState::L2_TAKING_OFF);
        }
        else
        {
            spdlog::error("Reject! beacuse can not transfer state from {}==>{}",
                          state_map(px4ctrl_fsm_state[2]),
                          state_map(Px4CtrlState::L2_TAKING_OFF));
        }
    };

    land = [&]()
    {
        auto px4ctrl_fsm_state = _px4_ctrl->get_px4_state();
        if (px4ctrl_fsm_state[2] == Px4CtrlState::L2_HOVERING)
        {
            _px4_ctrl->force_l2_state(Px4CtrlState::L2_LANDING);
        }
        else
        {
            spdlog::error("Reject! beacuse can not transfer state from {}==>{}",
                          state_map(px4ctrl_fsm_state[2]),
                          state_map(Px4CtrlState::L2_LANDING));
        }
    };

    force_hover = [&]()
    {
        auto px4ctrl_fsm_state = _px4_ctrl->get_px4_state();
        if (px4ctrl_fsm_state[2] == Px4CtrlState::L2_IDLE)
        {
            _px4_ctrl->force_l2_state(Px4CtrlState::L2_HOVERING);
            _px4_bridge->pub_allow_cmdctrl(false);
        }
        else
        {
            spdlog::error("Reject! beacuse can not transfer state from {}==>{}",
                          state_map(px4ctrl_fsm_state[2]),
                          state_map(Px4CtrlState::L2_HOVERING));
        }
    };

    allow_cmd_ctrl = [&]()
    {
        auto px4ctrl_fsm_state = _px4_ctrl->get_px4_state();
        if (px4ctrl_fsm_state[2] == Px4CtrlState::L2_HOVERING)
        {
            _px4_ctrl->force_l2_state(Px4CtrlState::L2_ALLOW_CMD_CTRL);
            _px4_bridge->pub_allow_cmdctrl(true);
        }
        else
        {
            spdlog::error("Reject! beacuse can not transfer state from {}==>{}",
                          state_map(px4ctrl_fsm_state[2]),
                          state_map(Px4CtrlState::L2_ALLOW_CMD_CTRL));
        }
    };

    _callbacks.clear();
    _callbacks["FORCE_DISARM"] = build_cb(force_disarm);
    _callbacks["ARM"] = build_cb(arm);
    _callbacks["DISARM"] = build_cb(disarm);
    _callbacks["ENTER_OFFBOARD"] = build_cb(enter_offboard);
    _callbacks["EXIT_OFFBOARD"] = build_cb(exit_offboard);
    _callbacks["TAKEOFF"] = build_cb(takeoff);
    _callbacks["LAND"] = build_cb(land);
    _callbacks["FORCE_HOVER"] = build_cb(force_hover);
    _callbacks["ALLOW_CMD_CTRL"] = build_cb(allow_cmd_ctrl);
}

ftxui::Component Px4CtrlUI::render()
{

    auto style = ftxui::ButtonOption::Animated(
        ftxui::Color::Default, ftxui::Color::GrayDark, ftxui::Color::Default,
        ftxui::Color::White);
    auto vc = ftxui::Container::Vertical({});
    for (auto& [name, callback] : _callbacks)
    {
        vc->Add(ftxui::Button(name, callback, style));
    }
    log_menu = 0;
    auto menu = ftxui::Menu(&log_list, &log_menu);
    auto log_comp = ftxui::Renderer(
        menu,
        [=]
        {
            return ftxui::vbox({ftxui::text("Log") | ftxui::center, ftxui::separator(),
                ftxui::vbox(menu->Render()) | ftxui::vscroll_indicator | ftxui::frame |
                ftxui::size(ftxui::HEIGHT, ftxui::LESS_THAN, 20)
            });
        });

    container = ftxui::Container::Horizontal({
        vc,
        log_comp,
    });

    auto ui = ftxui::Renderer(
        container,
        [&]
        {
            auto px4ctrl_fsm_state = _px4_ctrl->get_px4_state();
            return ftxui::vbox(
                {ftxui::text("PX4Ctrl") | ftxui::center, ftxui::separator(),
                 ftxui::hbox(
                     ftxui::vbox(container->Render()),
                     ftxui::vbox(
                         {wrap("FSM",
                               ftxui::hbox({
                                   ftxui::text("L0: " +
                                               state_map(px4ctrl_fsm_state[0])),
                                               ftxui::separator(),
                                   ftxui::text(" L1: " +
                                               state_map(px4ctrl_fsm_state[1])),
                                                ftxui::separator(),
                                   ftxui::text(" L2: " +
                                               state_map(px4ctrl_fsm_state[2])),
                               })) |
                              ftxui::border,
                          wrap("Quad",
                               ftxui::vbox({
                                   ftxui::text(fmt::format(
                                       "Pos: ({:.2f},{:.2f},{:.2f})", _pos.x(),
                                       _pos.y(), _pos.z())),
                                   ftxui::text(fmt::format(
                                       "Vel: ({:.2f},{:.2f},{:.2f})", _vel.x(),
                                       _vel.y(), _vel.z())),
                                   ftxui::text(fmt::format(
                                       "Omega: ({:.2f},{:.2f},{:.2f})",
                                       _omega.x(), _omega.y(), _omega.z())),
                                   ftxui::text(fmt::format(
                                       "Quat: ({:.2f},{:.2f},{:.2f},{:.2f})",
                                       _quat.w(), _quat.x(), _quat.y(),
                                       _quat.z())),
                                   ftxui::text(
                                       fmt::format("Odom Hz: {}, last updated:{}s ago", odom_hz,timePassedSeconds(_last_odom_time))),
                                   ftxui::text(fmt::format("CmdCtrl Hz: {} last updated:{}s ago", cmdctrl_hz,timePassedSeconds(_last_cmdctrl_time))),
                               })) |
                              ftxui::border}))});
        });

    ui |= ftxui::CatchEvent(
             [&](ftxui::Event event) -> bool
             {
                
                 // press space to force hover
                 // press j to force disarm
                 double step_size_vel = 0.1;
                 double sttep_size_yaw = 0.5;
                 int time_duration = 1;//
                 if (event.is_character())
                 {
                     if(timePassedSeconds(last_click_time)>1){

                     }
                     auto hover_pos =  _px4_ctrl->get_hovering_pos();
                     auto pos = hover_pos.first;
                     auto quat = hover_pos.second;
                     if (event == ftxui::Event::Character('j'))
                     {
                         spdlog::info("force disarm");
                         return true;
                     }
                     if (event == ftxui::Event::Character(' '))
                     {
                         spdlog::info("force hover");
                         return true;
                     }
                     if (event == ftxui::Event::Character('w')){
                        
                     }
                     if (event == ftxui::Event::Character('a')){
                        
                     }
                     if (event == ftxui::Event::Character('s')){
                        
                     }
                     if (event == ftxui::Event::Character('d')){
                        
                     }
                     if (event == ftxui::Event::Character('q')){
                        
                     }
                     if (event == ftxui::Event::Character('e')){
                        
                     }
                 }
                 return false;
             });
    return ui;
}

// Display a component nicely with a title on the left.
ftxui::Element Px4CtrlUI::wrap(const std::string& name, ftxui::Element element)
{
    return ftxui::hbox({
               ftxui::text(name) | ftxui::size(ftxui::WIDTH, ftxui::EQUAL, 8),
               ftxui::separator(),
               element | ftxui::xflex,
           }) |
           ftxui::xflex;
}

void Px4CtrlUI::data_thread()
{
    while (!_loop_ptr->HasQuitted())
    {
        auto log = _sink->getLog();
        // log_list append vector
        // log_list.clear();
        log_list.insert(log_list.end(), log.begin(), log.end());
        log_menu = log_list.size() - 1;
        _screen.PostEvent(ftxui::Event::Custom);
        // Update every 50ms.
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void Px4CtrlUI::ui_thread()
{
    _loop_ptr->Run();
    // while (!_loop_ptr->HasQuitted()) {
    // _loop_ptr->RunOnce();
    // }
}

Px4CtrlUI::~Px4CtrlUI()
{
    _screen.Exit();
    _ui_thread.join();
    _data_thread.join();
}

} // namespace px4ctrl