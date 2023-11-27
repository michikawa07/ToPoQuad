// #include <string>

// #include <ros/ros.h>
// #include "leg_node.hpp"

// //=== static メンバ変数の定義 ===============//
// //--- Subscriber ---//
// ros::Subscriber MasterArm::sub_command_arm_;
// //--- Publisher ---//
// ros::Publisher MasterArm::pub_motor_state_arm_;  // モーターの状態をSub
// ros::Publisher MasterArm::pub_motor_error_arm_;  // モーターのエラー情報をPub
// ros::Publisher MasterArm::pub_voltage_arm_;      // モーター電源電圧をPub
// ros::Publisher MasterArm::pub_magnet;      // モーター電源電圧をPub

// double MasterArm::time_last_commanded_;

// #define ID_SHOULDER_ROLL          2
// #define ID_SHOULDER_PITCH         3
// #define ID_ELBOW                  4
// #define ID_WRIST_PITCH            5
// #define ID_WRIST_YAW              6
// #define ID_WRIST_ROLL             7
// #define ID_GRIPPER_RIGHT          8
// #define ID_GRIPPER_LEFT           9
// #define ID_SHOULDER_ROLL_SMALL    11

// #define ID_HAND_LIGHT    ID_GRIPPER_LEFT
// #define ID_MAGNET_SENSOR ID_GRIPPER_RIGHT

// static const double OFFSET_SHOULDER_ROLL           =  0.0;
// static const double OFFSET_SHOULDER_ROLL_SMALL     =  0.0;
// static const double OFFSET_ELBOW_PITCH             =  0.0;
// static const double GEAR_RATIO_SHOULDER_ROLL       =  2.0;
// static const double GEAR_RATIO_SHOULDER_ROLL_SMALL =  4.0;
// static const double GEAR_RATIO_SHOULDER_PITCH      =  1.0;
// static const double GEAR_RATIO_ELBOW               = -1.0;
// static const double GEAR_RATIO_WRIST_PITCH         = -1.0;
// static const double GEAR_RATIO_WRIST_YAW           = -1.0;
// static const double GEAR_RATIO_WRIST_ROLL          =  1.0;
// static const double GEAR_RATIO_GRIPPER_RIGHT       = -1.0;
// static const double GEAR_RATIO_GRIPPER_LEFT        =  1.0;

// static double GetTime() {
//   return double(ros::Time::now().sec) + double(ros::Time::now().nsec)*1.0e-9;
// }

// static void Wait(double dt) {
//   double t_goal = GetTime() + dt;
//   while(t_goal > GetTime()) ;
// }

// void MasterArm::CallBackOfCommandArm(const fuhga_msgs::FuhgaCommand &cmd) {
//   time_last_commanded_ = double(ros::Time::now().sec) + double(ros::Time::now().nsec)*1.0e-9;

//   TargetData td = CommandToTargetData(cmd);
//   switch (cmd.command) {
//     case CMD_ARM_ACTIVATE                 : Activate(td);                 break;
//     case CMD_ARM_DEACTIVATE               : Deactivate(td);               break;
//     case CMD_ARM_SET_POSITION             : WritePosition(td);            break;
//     case CMD_ARM_SET_VELOCITY             : WriteVelocity(td);            break;
//     case CMD_ARM_SET_CURRENT              : WriteCurrent(td);             break;
//     case CMD_ARM_SET_LIGHT                : SetHandLight(cmd.data_i[0]);  break;
//     case CMD_ARM_SET_GAIN_POSITION_P      : WriteGainPositionP(td);       break;
//     case CMD_ARM_READ_POSITION            : ReadPosition(td);             break;
//     case CMD_ARM_READ_ERROR               : ReadError(td);                break;
//     case CMD_ARM_CLEAR_ERROR              : ClearError(td);               break;
//     case CMD_ARM_READ_VOLTAGE             : ReadVoltage(td);              break;
//     case CMD_ARM_SET_PROFILE_ACCELERATION : WriteProfileAcceleration(td); break;
//     case CMD_ARM_SET_PROFILE_VELOCITY     : WriteProfileVelocity(td);     break;
//   }
// }

// MasterArm::TargetData MasterArm::CommandToTargetData(fuhga_msgs::FuhgaCommand cmd) {
//   TargetData td;

//   std::vector<uint8_t> id_list = ConvertNameListToIdList(cmd.target);
//   if (cmd.data_i.size() != id_list.size()) {
//     cmd.data_i.clear();
//     for(int i = 0; i < id_list.size(); i++) cmd.data_i.push_back(0);
//   }
//   if (cmd.data_f.size() != id_list.size()) {
//     cmd.data_f.clear();
//     for(int i = 0; i < id_list.size(); i++) cmd.data_f.push_back(0.0);
//   }

//   for(int i = 0; i < id_list.size(); i++) {
//     if (id_list[i] == ID_SHOULDER_PITCH or
//         id_list[i] == ID_ELBOW or
//         id_list[i] == ID_WRIST_PITCH) {
//       td.id_pp.push_back(id_list[i]);
//       td.data_i_pp.push_back(cmd.data_i[i]);
//       td.data_f_pp.push_back(cmd.data_f[i]);
//     } else if (id_list[i] == ID_WRIST_YAW     or
//                id_list[i] == ID_WRIST_ROLL    or
//                id_list[i] == ID_GRIPPER_RIGHT or
//                id_list[i] == ID_GRIPPER_LEFT    ) {
//       td.id_x.push_back(id_list[i]);
//       td.data_i_x.push_back(cmd.data_i[i]);
//       td.data_f_x.push_back(cmd.data_f[i]);
//     } else if (id_list[i] == ID_SHOULDER_ROLL){
//       td.id_pp.push_back(id_list[i]);
//       td.data_i_pp.push_back(cmd.data_i[i]);
//       td.data_f_pp.push_back(cmd.data_f[i]);
//       td.id_x.push_back(ID_SHOULDER_ROLL_SMALL);
//       td.data_i_x.push_back(cmd.data_i[i]);
//       td.data_f_x.push_back(cmd.data_f[i]);
//     }
//   }
//   return td;
// }

// std::vector<uint8_t> MasterArm::ConvertNameListToIdList(std::vector<std::string> target) {
//   std::vector<uint8_t> target_id;

//   if (target.size() == 1 and target[0] == NAME_ARM_ALL) {
//     target_id.push_back(ID_SHOULDER_ROLL );
//     target_id.push_back(ID_SHOULDER_PITCH);
//     target_id.push_back(ID_ELBOW         );
//     target_id.push_back(ID_WRIST_PITCH   );
//     target_id.push_back(ID_WRIST_YAW     );
//     target_id.push_back(ID_WRIST_ROLL    );
//     target_id.push_back(ID_GRIPPER_RIGHT );
//     target_id.push_back(ID_GRIPPER_LEFT  );
//     return target_id;
//   }

//   for(std::vector<std::string>::iterator t = target.begin(); t != target.end(); ++t) {
//     if (*t == NAME_SHOULDER_ROLL ){
//       target_id.push_back(ID_SHOULDER_ROLL );
//     }
//     else if (*t == NAME_SHOULDER_PITCH) target_id.push_back(ID_SHOULDER_PITCH);
//     else if (*t == NAME_ELBOW         ) target_id.push_back(ID_ELBOW         );
//     else if (*t == NAME_WRIST_PITCH   ) target_id.push_back(ID_WRIST_PITCH   );
//     else if (*t == NAME_WRIST_YAW     ) target_id.push_back(ID_WRIST_YAW     );
//     else if (*t == NAME_WRIST_ROLL    ) target_id.push_back(ID_WRIST_ROLL    );
//     else if (*t == NAME_GRIPPER_RIGHT ) target_id.push_back(ID_GRIPPER_RIGHT );
//     else if (*t == NAME_GRIPPER_LEFT  ) target_id.push_back(ID_GRIPPER_LEFT  );
//   }

//   return target_id;
// }

// std::string MasterArm::ConvertIdToName(uint8_t id) {
//   switch(id) {
//    case ID_SHOULDER_ROLL       : return NAME_SHOULDER_ROLL ;
//    case ID_SHOULDER_ROLL_SMALL : return NAME_SHOULDER_ROLL ;
//    case ID_SHOULDER_PITCH      : return NAME_SHOULDER_PITCH;
//    case ID_ELBOW               : return NAME_ELBOW         ;
//    case ID_WRIST_PITCH         : return NAME_WRIST_PITCH   ;
//    case ID_WRIST_YAW           : return NAME_WRIST_YAW     ;
//    case ID_WRIST_ROLL          : return NAME_WRIST_ROLL    ;
//    case ID_GRIPPER_RIGHT       : return NAME_GRIPPER_RIGHT ;
//    case ID_GRIPPER_LEFT        : return NAME_GRIPPER_LEFT  ;
//   }
// }

// void MasterArm::Activate(TargetData td) {
//   servo_.Write(ID_HAND_LIGHT,    external_port_mode_1_x, EXTERNAL_PORT_MODE_DOUT_NOPULL);
//   servo_.Write(ID_MAGNET_SENSOR, external_port_mode_1_x, EXTERNAL_PORT_MODE_AIN        );
//   servo_.Write(ID_MAGNET_SENSOR, external_port_mode_2_x, EXTERNAL_PORT_MODE_AIN        );
//   servo_.Write(ID_MAGNET_SENSOR, external_port_mode_3_x, EXTERNAL_PORT_MODE_AIN        );

//   //pro plusシリーズの初期設定
//   static const int acc_max_pp54 =  9982; // 17.42 [rad/s^2]くらい,実際はもっと途方もない値まで行けるが，デフォルト上限がこの値．
//   static const int vel_max_pp54 =  2900; //  3.04 [rad/s]くらい
//   static const int acc_max_pp42 = 10765; // 18.79 [rad/s^2]くらい,実際はもっと途方もない値まで行けるが，デフォルト上限がこの値．
//   static const int vel_max_pp42 =  2920; //  3.06 [rad/s]くらい
//   std::vector<int64_t> data_pp_int(td.id_pp.size(), TORQUE_ENABLE);
//   std::vector<int64_t> mode_pp_int(td.id_pp.size(), OPERATING_MODE_POSITION);
//   for (int i=0; i<td.id_pp.size(); i++) {
//     if(td.id_pp[i] == ID_SHOULDER_ROLL ) mode_pp_int[i] = OPERATING_MODE_EXTENDED_POSITION;
//     else if(td.id_pp[i] == ID_WRIST_PITCH) mode_pp_int[i] = OPERATING_MODE_EXTENDED_POSITION;
//   }
//   if (td.id_pp.size() != 0) {
//     servo_.SyncWrite(td.id_pp.size(), td.id_pp.data(), operating_mode_pp, mode_pp_int.data());
//     servo_.SyncWrite(td.id_pp.size(), td.id_pp.data(), torque_enable_pp, data_pp_int.data());
//     for(int i=0; i<td.id_pp.size(); i++){
//       switch(td.id_pp[i]){
//         case ID_SHOULDER_ROLL:
//           //servo_.Write(td.id_pp[i], profile_acceleration_pp, 0.2*acc_max_pp54);
//           //servo_.Write(td.id_pp[i], profile_velocity_pp, 1.0*vel_max_pp54);
//           break;
//         case ID_SHOULDER_PITCH:
//         case ID_ELBOW: 
//           servo_.Write(td.id_pp[i], profile_acceleration_pp, 0.1*acc_max_pp54);
//           servo_.Write(td.id_pp[i], profile_velocity_pp, 1.0*vel_max_pp54);
//           break;
//         case ID_WRIST_PITCH:
//           servo_.Write(td.id_pp[i], profile_acceleration_pp, 0.2*acc_max_pp42);
//           servo_.Write(td.id_pp[i], profile_velocity_pp, 1.0*vel_max_pp42);
//           break;
//       }
//     }
//   }

//   //Xシリーズの初期設定
//   static const int acc_max_x = 32767; // ×214.577*2*M_PI/3600 = 12271.5 [rad/s^2]くらい
//   static const int vel_max_x = 32767; // ×0.229*2*M_PI/60 =  785.8 [rad/s]くらい
//   std::vector<int64_t> data_x_int(td.id_x.size(), TORQUE_ENABLE);
//   std::vector<int64_t> mode_x_int(td.id_x.size(), OPERATING_MODE_CURRENT_BASE_POSITION);
//   if (td.id_x.size() != 0) {
//     servo_.SyncWrite(td.id_x.size(), td.id_x.data(), operating_mode_x, mode_x_int.data());
//     servo_.SyncWrite(td.id_x.size(), td.id_x.data(), torque_enable_x, data_x_int.data());
//     for(int i=0; i<td.id_x.size(); i++){
//       switch (td.id_x[i]) {
//         case ID_SHOULDER_ROLL_SMALL:
//           //servo_.Write(td.id_x[i], profile_acceleration_x, 0.5*acc_max_x);
//           //servo_.Write(td.id_x[i], profile_velocity_x, 1.0*vel_max_x);
//           break;
//         case ID_WRIST_ROLL:
//           servo_.Write(td.id_x[i], position_p_gain_x, 425); //振動を押さえるため
//           servo_.Write(td.id_x[i], position_d_gain_x, 1000);//振動を押さえるため
//           servo_.Write(td.id_x[i], profile_acceleration_x, 0.5*acc_max_x);
//           servo_.Write(td.id_x[i], profile_velocity_x, 1.0*vel_max_x);
//           break;
//         case ID_WRIST_YAW: 
//           servo_.Write(td.id_x[i], position_p_gain_x, 300); //振動を押さえる
//           servo_.Write(td.id_x[i], position_d_gain_x, 1000);//振動を押さえる
//           servo_.Write(td.id_x[i], profile_acceleration_x, 0.5*acc_max_x);
//           servo_.Write(td.id_x[i], profile_velocity_x, 1.0*vel_max_x);
//           break;
//         case ID_GRIPPER_LEFT: 
//         case ID_GRIPPER_RIGHT: 
//           servo_.Write(td.id_x[i], profile_acceleration_x, 0.5*acc_max_x);
//           servo_.Write(td.id_x[i], profile_velocity_x, 1.0*vel_max_x);
//           break;
//       }
//     }
//   }
// }

// void MasterArm::Deactivate(TargetData td) {
//   std::vector<int64_t> data_pp_int(td.id_pp.size(), TORQUE_DISABLE);
//   std::vector<int64_t> data_x_int(td.id_x.size(), TORQUE_DISABLE);

//   if (td.id_pp.size() != 0) {
//     servo_.SyncWrite(td.id_pp.size(), td.id_pp.data(), torque_enable_pp, data_pp_int.data());
//   }
//   if (td.id_x.size() != 0) {
//     servo_.SyncWrite(td.id_x.size(), td.id_x.data(), torque_enable_x, data_x_int.data());
//   }
// }

// //=====================================
// // main 
// //=====================================

// int main(int argc, char **argv) {
//   ros::init(argc, argv, "fuhga_master_node");
//   ros::NodeHandle nh;
//   ros::NodeHandle nh_p("~");
//   std::string port_arm;
//   std::string port_mob_flipper;
//   std::string port_mob_crawler;
//   nh_p.getParam("port_arm", port_arm);
//   nh_p.getParam("port_mob_flipper", port_mob_flipper);
//   nh_p.getParam("port_mob_crawler", port_mob_crawler);

//   MasterArm::Initialize(port_arm);
//   MasterMob::Initialize(port_mob_flipper, port_mob_crawler);

//   ros::Subscriber sub_voltage = nh.subscribe("robot_state/voltage", 1, CallBackOfVoltage);

//   ros::Rate rate(50);

//   while(ros::ok()) {

//     const static double T_TH_NO_COM_STOP = 1.0;
//     const static double T_TH_NO_COM_DEACTIVATE = 5.0;
//     double t_now = double(ros::Time::now().sec) + double(ros::Time::now().nsec)*1.0e-9;

//     if (t_now - MasterArm::time_last_commanded() > T_TH_NO_COM_DEACTIVATE and
//         t_now - MasterMob::time_last_commanded() > T_TH_NO_COM_DEACTIVATE ) {
//       MasterMob::DeactivateAll();
//       MasterArm::DeactivateAll();
//       ROS_WARN("No command : Deactivate All!");
//     } else if (t_now - MasterMob::time_last_commanded() > T_TH_NO_COM_STOP ) {
//       MasterMob::StopCrawler();
//       ROS_WARN("No command : Stop crawler!");
//     }

//     static double t_last_over_vol = t_now;
//     if (t_now - t_last_over_vol > 3.0 ) {
//       MasterMob::DeactivateAll();
//       MasterArm::DeactivateAll();
//       ROS_WARN("Low Voltage!");
//     }
//     if (voltage > VOLTAGE_MIN) {
//       t_last_over_vol = t_now;
//     }

//     //　磁気センサの出力を -1 ~ +1 でパブリッシュする．
//     // このノードに配置するのは一貫性がなくてよくないんだけど，他におくところないから...
//     MasterArm::ReadMagnetSensor();

//     ros::spinOnce();
//     rate.sleep();
//   }
// }

#include <iostream>
#include <dynamixel_handler/dynamixel_state.h>
#include <dynamixel_handler/dynamixel_command.h>

int main(int argc, char **argv){
    // hello world
    std::cout << "Hello World" << std::endl;
}