#include <string>
#include <Eigen/Core>

#include <ros/ros.h>
// #include "leg_node.hpp"

#include <dynamixel_handler/DynamixelState.h>
#include <dynamixel_handler/DynamixelCmd.h>
#include <topoquad_master/QuadRobotState.h>
#include <topoquad_master/QuadRobotCmd.h>

class Joint {
    public:
        Joint(int id): servo_angle_(0.0), joint_angle_(0.0), id_(1), gear_ratio_(1.0), fixed_coord_(Eigen::Vector3d(0.0, 0.0, 0.0)) {}
        Joint(int id, double gear_ratio, double joint_angle, Eigen::Vector3d fixed_coord):
            joint_angle_(joint_angle), servo_angle_(joint_angle*gear_ratio),
            id_(id),  gear_ratio_(gear_ratio), fixed_coord_(fixed_coord) {}
        // 角度の入力
        void SetAngle(double angle) { 
            joint_angle_ = angle;
            servo_angle_ = gear_ratio_*angle; //todo 可動域の制限など
        }
       
        double servo_angle_; //[rad] 
        double joint_angle_; //[rad] // サーボと関節の角度,ギア比分だけ異なる
        int id_;  // DynamixelのID, 固定値
        double gear_ratio_; // Dynamixelと関節のギア比 (逆転は負の値)
        Eigen::Vector3d fixed_coord_; // [m]  // 直前の関節座標系から見たこの関節座標系の原点の位置ベクトル 軸方向がx軸，サーボ回転軸がy軸，z軸は右手系.
};

class Leg {
    public:
        Leg(): is_updated_(true), hip_yaw_(Joint(1)), hip_pitch_(Joint(2)), knee_pitch_(Joint(3)) {}
        void initialize(const Joint& hip_yaw, const Joint& hip_pitch, const Joint& knee_pitch) {
            hip_yaw_ = hip_yaw;
            hip_pitch_ = hip_pitch;
            knee_pitch_ = knee_pitch;
            is_updated_ = true;
        }
        void SetAngles(const std::vector<double>& angles) {
            if (angles.size() != 3) {
                ROS_ERROR("The size of angles must be 3");
                return;
            }
            hip_yaw_.SetAngle(angles[0]);
            hip_pitch_.SetAngle(angles[1]);
            knee_pitch_.SetAngle(angles[2]);
            is_updated_ = true;
        }

        bool is_updated_; // 関節角が更新されたかどうか
        Joint hip_yaw_;
        Joint hip_pitch_;
        Joint knee_pitch_;
};

Leg leg_FR;
Leg leg_FL;
Leg leg_BR;
Leg leg_BL;

void CallBackOfLegCmd(const topoquad_master::QuadRobotCmd::ConstPtr& msg) {
    if(msg->angles_FR.size() > 1) leg_FR.SetAngles(msg->angles_FR);
    if(msg->angles_FL.size() > 1) leg_FL.SetAngles(msg->angles_FL);
    if(msg->angles_BR.size() > 1) leg_BR.SetAngles(msg->angles_BR);
    if(msg->angles_BL.size() > 1) leg_BL.SetAngles(msg->angles_BL);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "leg_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");
    
    leg_BR.initialize( Joint{  4, -1.0,   0.0 , Eigen::Vector3d(0.0, 0.0, 0.0) },
                       Joint{  3, +1.0, M_PI/4, Eigen::Vector3d(0.0, 0.0, 0.0) },
                       Joint{  2, +1.0, M_PI/4, Eigen::Vector3d(0.0, 0.0, 0.0) }  );
    leg_FR.initialize( Joint{ 14, -1.0,   0.0 , Eigen::Vector3d(0.0, 0.0, 0.0) },
                       Joint{ 13, +1.0, M_PI/4, Eigen::Vector3d(0.0, 0.0, 0.0) },
                       Joint{ 12, +1.0, M_PI/4, Eigen::Vector3d(0.0, 0.0, 0.0) }  ); 
    leg_FL.initialize( Joint{ 24, +1.0,   0.0 , Eigen::Vector3d(0.0, 0.0, 0.0) },
                       Joint{ 23, +1.0, M_PI/4, Eigen::Vector3d(0.0, 0.0, 0.0) },
                       Joint{ 22, +1.0, M_PI/4, Eigen::Vector3d(0.0, 0.0, 0.0) }  );
    leg_BL.initialize( Joint{ 34, +1.0,   0.0 , Eigen::Vector3d(0.0, 0.0, 0.0) },
                       Joint{ 33, +1.0, M_PI/4, Eigen::Vector3d(0.0, 0.0, 0.0) },
                       Joint{ 32, +1.0, M_PI/4, Eigen::Vector3d(0.0, 0.0, 0.0) }  );           

    ros::Subscriber sub_leg_cmd   = nh.subscribe("/legs/cmd", 10, CallBackOfLegCmd);
    ros::Publisher  pub_dyn_cmd   = nh.advertise<dynamixel_handler::DynamixelCmd>("/dynamixel/cmd", 10);

    ros::Duration(1).sleep();

    // ros::Subscriber sub_dyn_state   = nh.subscribe("/dynamixel/state",   10, CallBackOfDynamixelState);  // サーボの角度をsubscribe
    // ros::Publisher  pub_leg_state   = nh.advertise<dynamixel_handler::DynamixelCmd>("/legs/state", 10); // サーボの角度を関節の状態に変換してpublish

    ros::Rate rate(100);
    while(ros::ok()) {
        dynamixel_handler::DynamixelCmd dyn_msg;
        dyn_msg.command = "write";
        for ( auto& leg : {std::ref(leg_FR), std::ref(leg_FL), std::ref(leg_BR), std::ref(leg_BL)}) {
            if( !leg.get().is_updated_ ) continue;
            dyn_msg.ids.push_back(leg.get().hip_yaw_.id_);
            dyn_msg.ids.push_back(leg.get().hip_pitch_.id_);
            dyn_msg.ids.push_back(leg.get().knee_pitch_.id_);
            dyn_msg.goal_angles.push_back(leg.get().hip_yaw_.servo_angle_);
            dyn_msg.goal_angles.push_back(leg.get().hip_pitch_.servo_angle_);
            dyn_msg.goal_angles.push_back(leg.get().knee_pitch_.servo_angle_);
            leg.get().is_updated_ = false;

            ROS_INFO("%f, %f, %f", leg.get().hip_yaw_.servo_angle_, leg.get().hip_pitch_.servo_angle_, leg.get().knee_pitch_.servo_angle_);
        }

        if (dyn_msg.ids.size() != 0) {
            pub_dyn_cmd.publish(dyn_msg);
            ROS_INFO("publish, id size: %d", dyn_msg.ids.size());
        }

        ros::spinOnce();
        rate.sleep();
    }
}