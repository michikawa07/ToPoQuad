#include <string>

#include <ros/ros.h>
// #include "leg_node.hpp"

#include <iostream>
#include <dynamixel_handler/DynamixelState.h>
#include <dynamixel_handler/DynamixelCmd.h>

struct Leg {
    int id_hip_yaw;
    int id_hip_pitch;
    int id_knee_pitch;
    float angle_hip_yaw;
    float angle_hip_pitch;
    float angle_knee_pitch;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "leg_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");
    
    // Legのベクタ
    std::vector<Leg> legs = {
        Leg{ 2, 3, 4, 0.0, 0.0, 0.0},
        Leg{12,13,14, 0.0, 0.0, 0.0},
        Leg{22,23,24, 0.0, 0.0, 0.0},
        Leg{32,33,34, 0.0, 0.0, 0.0}
    };

    // ros::Subscriber sub_dyn_state   = nh.subscribe("/dynamixel/state",   10, CallBackOfDynamixelState);
    ros::Publisher  pub_dyn_cmd     = nh.advertise<dynamixel_handler::DynamixelCmd>("/dynamixel/cmd", 10);

    ros::Rate rate(20);
    while(ros::ok()) {
        // Dynamixelから現在角をRead & topicをPublish
        dynamixel_handler::DynamixelCmd msg;
        msg.command = "write";
        msg.ids.resize(12);
        msg.goal_angles.resize(12);
        for (int i=0; i<4; i++) {
            msg.ids[3*i+0] = legs[i].id_hip_yaw;
            msg.ids[3*i+1] = legs[i].id_hip_pitch;
            msg.ids[3*i+2] = legs[i].id_knee_pitch;
            msg.goal_angles[3*i+0] = legs[i].angle_hip_yaw;
            msg.goal_angles[3*i+1] = legs[i].angle_hip_pitch;
            msg.goal_angles[3*i+2] = legs[i].angle_knee_pitch;
        }
        pub_dyn_cmd.publish(msg);

        // topicをSubscribe & Dynamixelへ目標角をWrite
        ros::spinOnce();
        rate.sleep();
    }
}