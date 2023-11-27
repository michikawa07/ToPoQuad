// #ifndef FUHGA_MASTER_SRC_MASTER_ARM_H_
// #define FUHGA_MASTER_SRC_MASTER_ARM_H_

// #include <string>
// #include <vector>

// #include <ros/ros.h>
// #include <std_msgs/Float32.h>
// #include <sensor_msgs/JointState.h>

// class MasterArm {
//  public:
//   static void Initialize(std::string port_name) {
//     ros::NodeHandle nh;

//     sub_command_arm_ = nh.subscribe("command/arm", 20, MasterArm::CallBackOfCommandArm);
//     pub_motor_state_arm_ = nh.advertise<sensor_msgs::JointState>("joint_state/arm", 10);
//     pub_motor_error_arm_ = nh.advertise<fuhga_msgs::FuhgaMotorError>("motor_error/arm", 10);
//     pub_voltage_arm_ = nh.advertise<std_msgs::Float32>("robot_state/voltage", 10);
//     pub_magnet = nh.advertise<std_msgs::Float32>("magnet", 10);

//     servo_.GetPortHandler(port_name.c_str());
//     servo_.set_baudrate(1000000);
//     servo_.OpenPort();

//     time_last_commanded_ = double(ros::Time::now().sec) + double(ros::Time::now().nsec)*1.0e-9;
//   }

//   static double time_last_commanded(){ return time_last_commanded_; }
//   static void DeactivateAll();
//   static void ReadMagnetSensor(); // ダイナミクセルのIDがfuhga_masterにしか書かれてないので仕方なくここに作る．

//  private:
//   struct TargetData {
//     std::vector<uint8_t> id_pp;
//     std::vector<uint8_t> id_x;
//     std::vector<int64_t> data_i_pp;
//     std::vector<int64_t> data_i_x;
//     std::vector<double> data_f_pp;
//     std::vector<double> data_f_x;
//   };

//   static void CallBackOfCommandArm(const fuhga_msgs::FuhgaCommand &cmd);

//   static std::vector<uint8_t> ConvertNameListToIdList(std::vector<std::string> target);
//   static TargetData CommandToTargetData(fuhga_msgs::FuhgaCommand cmd);
//   static std::string ConvertIdToName(uint8_t id);
//   static void Activate(TargetData td);
//   static void Deactivate(TargetData td);
//   static void WritePosition(TargetData td);
//   static void WriteVelocity(TargetData td);
//   static void WriteCurrent(TargetData td);
//   static void WriteProfileAcceleration(TargetData td);
//   static void WriteProfileVelocity(TargetData td);
//   static void WriteGainPositionP(TargetData td);
//   static void ReadPosition(TargetData td);
//   static void ReadError(TargetData td);
//   static void ReadVoltage(TargetData td);
//   static void ClearError(TargetData td);
//   static void SetHandLight(int8_t level);

//   //--- Subscriber ---//
//   static ros::Subscriber sub_command_arm_;
//   //--- Publisher ---//
//   static ros::Publisher pub_motor_state_arm_;  // モーターの状態をSub
//   static ros::Publisher pub_motor_error_arm_;  // モーターのエラー情報をPub
//   static ros::Publisher pub_voltage_arm_;  // モーター電源電圧をPub
//   static ros::Publisher pub_magnet;

//   static Dynamixel servo_;

//   static double time_last_commanded_;

// };

// #endif /* FUHGA_MASTER_SRC_MASTER_ARM_H_ */