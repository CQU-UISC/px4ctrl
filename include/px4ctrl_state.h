#pragma once

#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <queue>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <quadrotor_msgs/CtrlCommand.h>
#include <chrono>
#include <type_traits>
#include <utility>


namespace px4ctrl{
    using clock = std::chrono::high_resolution_clock;

    template <typename T, typename... U>
    concept any_of = (std::same_as<T, U> || ...);

    //判断是否为std::pair<T,clock::time_point>的类型
    template<typename T>
    concept IPX4_ITEM = requires(T t){
        {std::remove_cvref_t<T>(t)} -> any_of<
        mavros_msgs::State::ConstPtr,
        mavros_msgs::ExtendedState::ConstPtr,
        sensor_msgs::BatteryState::ConstPtr,
        nav_msgs::Odometry::ConstPtr,
        sensor_msgs::Imu::ConstPtr,
        quadrotor_msgs::CtrlCommand::ConstPtr
        >;
    };

    template<typename T>
    concept IPX4_TIME = requires(T t){
        {std::remove_cvref_t<T>(t)} -> std::same_as<clock::time_point>;
    };

    template <IPX4_ITEM ITEM, IPX4_TIME TIME>
    using IPX4_STATE = std::pair<ITEM, TIME>;

    struct PX4_STATE{
        IPX4_STATE<mavros_msgs::State::ConstPtr, clock::time_point> state;
        IPX4_STATE<mavros_msgs::ExtendedState::ConstPtr, clock::time_point> ext_state;
        IPX4_STATE<sensor_msgs::BatteryState::ConstPtr,clock::time_point> battery;
        IPX4_STATE<nav_msgs::Odometry::ConstPtr,clock::time_point> odom;
        IPX4_STATE<sensor_msgs::Imu::ConstPtr,clock::time_point> imu;
        IPX4_STATE<quadrotor_msgs::CtrlCommand::ConstPtr, clock::time_point> ctrl_command;
    };
}