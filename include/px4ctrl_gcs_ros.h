#pragma once
#include "px4ctrl_gcs.h"
#include "quadrotor_msgs/Drone.h"
#include "quadrotor_msgs/GCS.h"
#include <queue>
#include <ros/ros.h>

namespace px4ctrl{
namespace gcs{
    namespace rosimpl{
        class GcsRosCom : public GcsCom{
        public:
            GcsRosCom()=delete;
            GcsRosCom(ros::NodeHandle& nh);
            bool send(const Gcs&) override;
            bool receive(Drone&) override;
            void cb(const quadrotor_msgs::DroneConstPtr&);
        private:
            ros::Publisher pub;
            ros::Subscriber sub;
            std::queue<Drone> cb_queue;
        };

        class DroneRosCom : public DroneCom{
        public:
            DroneRosCom()=delete;
            DroneRosCom(ros::NodeHandle& nh);
            bool send(const Drone&) override;
            bool receive(Gcs&) override;
            void cb(const quadrotor_msgs::GCSConstPtr&);
        private:
            ros::Publisher pub;
            ros::Subscriber sub;
            std::queue<Gcs> cb_queue;
        };
    }
}
}
