#include "px4ctrl_gcs_ros.h"
#include "px4ctrl_gcs.h"
#include "ros/init.h"

namespace px4ctrl{
namespace gcs{
    namespace rosimpl{
        //Gcs
        GcsRosCom::GcsRosCom(ros::NodeHandle& nh){
            pub = nh.advertise<quadrotor_msgs::GCS>("/px4ctrl/gcs",10);
            sub = nh.subscribe("/px4ctrl/drone",10,&GcsRosCom::cb,this);
        }


        bool GcsRosCom::send(const Gcs& gcs){
            quadrotor_msgs::GCS msg;
            msg.id = gcs.id;
            msg.version = gcs.version;
            msg.gcs_id = gcs.gcs_id;
            msg.drone_id = gcs.drone_id;
            msg.drone_cmd = gcs.drone_cmd;
            msg.len_payload = gcs.len_payload;
            for(int i=0;i<gcs.len_payload;i++){
                msg.payload.push_back(gcs.payload[i]);
            }
            pub.publish(msg);
            return true;
        }

        bool GcsRosCom::receive(Drone& drone){
            ros::spinOnce();
            if(cb_queue.empty()){
                return false;
            }
            while(cb_queue.size()>10){
                cb_queue.pop();
            }
            drone = cb_queue.front();
            cb_queue.pop();
            return true;
        }

        void GcsRosCom::cb(const quadrotor_msgs::DroneConstPtr& msg){
            Drone drone;
            drone.id = msg->id;
            drone.version = msg->version;
            drone.gcs_id = msg->gcs_id;
            drone.drone_id = msg->drone_id;
            for(int i=0;i<3;i++){
                drone.px4ctrl_status[i] = msg->px4ctrl_status[i];
                drone.pos[i] = msg->pos[i];
                drone.vel[i] = msg->vel[i];
                drone.acc[i] = msg->acc[i];
                drone.omega[i] = msg->omega[i];
                drone.quat[i] = msg->quat[i];
            }
            drone.quat[3] = msg->quat[3];
            drone.battery = msg->battery;
            cb_queue.push(drone);
        }


        DroneRosCom::DroneRosCom(ros::NodeHandle& nh){
            pub = nh.advertise<quadrotor_msgs::Drone>("/px4ctrl/drone",10);
            sub = nh.subscribe("/px4ctrl/gcs",10,&DroneRosCom::cb,this);
        }

        bool DroneRosCom::send(const Drone& drone){
            quadrotor_msgs::Drone msg;
            msg.id = drone.id;
            msg.version = drone.version;
            msg.gcs_id = drone.gcs_id;
            msg.drone_id = drone.drone_id;
            for(int i=0;i<3;i++){
                msg.px4ctrl_status[i] = drone.px4ctrl_status[i];
                msg.pos[i] = drone.pos[i];
                msg.vel[i] = drone.vel[i];
                msg.acc[i] = drone.acc[i];
                msg.omega[i] = drone.omega[i];
                msg.quat[i] = drone.quat[i];
            }
            msg.quat[3] = drone.quat[3];
            msg.battery = drone.battery;
            pub.publish(msg);
            return true;
        }

        bool DroneRosCom::receive(Gcs& gcs){
            ros::spinOnce();
            if(cb_queue.empty()){
                return false;
            }
            while(cb_queue.size()>10){
                cb_queue.pop();
            }
            gcs = cb_queue.front();
            cb_queue.pop();
            return true;
        }

        void DroneRosCom::cb(const quadrotor_msgs::GCSConstPtr& msg){
            Gcs gcs;
            gcs.id = msg->id;
            gcs.version = msg->version;
            gcs.gcs_id = msg->gcs_id;
            gcs.drone_id = msg->drone_id;
            gcs.drone_cmd = msg->drone_cmd;
            gcs.len_payload = msg->len_payload;
            for(int i=0;i<gcs.len_payload;i++){
                gcs.payload[i] = msg->payload[i];
            }
            cb_queue.push(gcs);
        }
    }
}
}
