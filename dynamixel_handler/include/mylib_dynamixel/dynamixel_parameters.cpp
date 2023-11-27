#include "dynamixel_parameters.h"
#include <math.h>

// 道川
// あまり良くない実装だと思いながら前例踏襲で書いている．
// 型番ごとに微妙にパラメータ(特にvaleu_par_bit)が異なるので，以下のようにべた書きすると事故の基．
// 自動認識する仕組みを導入したい． ros2移行時に一緒にやるつもり．

// dynamixel x
DynamixelParameter id_x(                     7, TYPE_UINT8);
DynamixelParameter baudrate_x(               8, TYPE_UINT8);
DynamixelParameter return_deray_time_x(      9, TYPE_UINT8);
DynamixelParameter operating_mode_x(        11, TYPE_UINT8);
DynamixelParameter homing_offset_x(         20, TYPE_INT32, M_PI/2048.0);
DynamixelParameter pwm_limit_x(             36, TYPE_UINT16, 100.0/885 /*%*/);
DynamixelParameter current_limit_x(         38, TYPE_UINT16, 2.69/*mA*/); // 型番によって異なる
DynamixelParameter current_limit_xh430v(    38, TYPE_UINT16, 1.34/*mA*/); // 型番によって異なる
DynamixelParameter current_limit_x_330_(    38, TYPE_UINT16, 1.00/*mA*/); // 型番によって異なる
DynamixelParameter acceleration_limit_x(    40, TYPE_UINT32, 214.577*2.0*M_PI/3600.0);
DynamixelParameter velocity_limit_x(        44, TYPE_UINT32, 0.229*2.0*M_PI/60.0 ); // XL330だけ異なる
DynamixelParameter max_position_limit_x(    48, TYPE_UINT32, M_PI/2048.0);
DynamixelParameter min_position_limit_x(    52, TYPE_UINT32, M_PI/2048.0);
DynamixelParameter external_port_mode_1_x(  56, TYPE_UINT8);
DynamixelParameter external_port_mode_2_x(  57, TYPE_UINT8);
DynamixelParameter external_port_mode_3_x(  58, TYPE_UINT8);
DynamixelParameter shutdown_x(              63, TYPE_UINT8);
DynamixelParameter torque_enable_x(         64, TYPE_UINT8);
DynamixelParameter led_x(                   65, TYPE_UINT8);
DynamixelParameter status_return_level_x(   68, TYPE_UINT8);
DynamixelParameter hardware_error_status_x( 70, TYPE_UINT8);
DynamixelParameter velocity_i_gain_x(       76, TYPE_UINT16); // 型番によってデフォルト値が異なる
DynamixelParameter velocity_p_gain_x(       78, TYPE_UINT16); // 型番によってデフォルト値が異なる
DynamixelParameter position_d_gain_x(       80, TYPE_UINT16); // 型番によってデフォルト値が異なる
DynamixelParameter position_i_gain_x(       82, TYPE_UINT16); // 型番によってデフォルト値が異なる
DynamixelParameter position_p_gain_x(       84, TYPE_UINT16); // 型番によってデフォルト値が異なる
DynamixelParameter goal_current_x(         102, TYPE_INT16, 2.69/1000.0/*A*/); // 型番によって異なる
DynamixelParameter goal_current_xh430v(    102, TYPE_INT16, 1.34/1000.0/*A*/); // 型番によって異なる
DynamixelParameter goal_current_x_330_(    102, TYPE_INT16, 1.00/1000.0/*A*/); // 型番によって異なる
DynamixelParameter goal_velocity(          104, TYPE_INT32, 0.229*2.0*M_PI/60.0);
DynamixelParameter profile_acceleration_x( 108, TYPE_INT32, 214.577*2.0*M_PI/3600.0); // unit 214.577 [rpm^2]
DynamixelParameter profile_velocity_x(     112, TYPE_INT32, 0.229*2.0*M_PI/60.0 );    // unit 0.229  [rpm]
DynamixelParameter goal_position_x(        116, TYPE_INT32, M_PI/2048.0);
DynamixelParameter present_current_x(      126, TYPE_INT16, 2.69/*mA*/); // 型番によって異なる
DynamixelParameter present_current_xh430v( 126, TYPE_INT16, 1.34/*mA*/); // 型番によって異なる
DynamixelParameter present_current_x_330_( 126, TYPE_INT16, 1.00/*mA*/); // 型番によって異なる
DynamixelParameter present_velocity_x(     128, TYPE_INT32, M_PI/180.0*0.229);
DynamixelParameter present_position_x(     132, TYPE_INT32, M_PI/2048.0);
DynamixelParameter present_input_voltage_x(144, TYPE_UINT16, 0.1);
DynamixelParameter present_temperture_x(   146, TYPE_UINT8);
DynamixelParameter external_port_data_1_x( 152, TYPE_UINT16);
DynamixelParameter external_port_data_2_x( 154, TYPE_UINT16);
DynamixelParameter external_port_data_3_x( 156, TYPE_UINT16);

// // dynamixel pro plus
// DynamixelParameter id_pp(7, TYPE_UINT8);
// DynamixelParameter baudrate_pp(8, TYPE_UINT8);
// DynamixelParameter return_deray_time_pp(9, TYPE_UINT8);
// DynamixelParameter drive_mode_pp(10, TYPE_UINT8);
// DynamixelParameter operating_mode_pp(11, TYPE_UINT8);
// DynamixelParameter homing_offset_pp42(20, TYPE_INT32, M_PI/303750);
// DynamixelParameter homing_offset_pp54(20, TYPE_INT32, M_PI/501923);
// DynamixelParameter current_limit_pp(38, TYPE_UINT16, 0.001);
// DynamixelParameter acceleration_limit_pp(38, TYPE_UINT32, 58000*2.0*M_PI/3600.0); //unit 58000 [rpm^2]
// DynamixelParameter velocity_limit_pp(38, TYPE_UINT32, 0.01*2.0*M_PI/60.0); //unit 0.01 [rpm]
// DynamixelParameter max_position_limit_pp42(48, TYPE_INT32, M_PI/303750);
// DynamixelParameter max_position_limit_pp54(48, TYPE_INT32, M_PI/501923);
// DynamixelParameter min_position_limit_pp42(52, TYPE_INT32, M_PI/303750);
// DynamixelParameter min_position_limit_pp54(52, TYPE_INT32, M_PI/501923);
// DynamixelParameter external_port_mode_1_pp(56, TYPE_UINT8);
// DynamixelParameter external_port_mode_2_pp(57, TYPE_UINT8);
// DynamixelParameter external_port_mode_3_pp(58, TYPE_UINT8);
// DynamixelParameter external_port_mode_4_pp(59, TYPE_UINT8);
// DynamixelParameter shutdown_pp(63, TYPE_UINT8);
// DynamixelParameter torque_enable_pp(512, TYPE_UINT8);
// DynamixelParameter led_red_pp(513, TYPE_UINT8);
// DynamixelParameter led_green_pp(514, TYPE_UINT8);
// DynamixelParameter led_blue_pp(515, TYPE_UINT8);
// DynamixelParameter velocity_i_gain_pp(524, TYPE_UINT16);
// DynamixelParameter velocity_p_gain_pp(526, TYPE_UINT16);
// DynamixelParameter position_d_gain_pp(528, TYPE_UINT16);
// DynamixelParameter position_i_gain_pp(530, TYPE_UINT16);
// DynamixelParameter position_p_gain_pp(532, TYPE_UINT16);
// DynamixelParameter goal_current_pp(550, TYPE_INT16, 0.001);
// DynamixelParameter goal_velocity_pp(552, TYPE_INT32, 0.01*2.0*M_PI/60.0);
// DynamixelParameter profile_acceleration_pp(556, TYPE_UINT32, 1.0*2.0*M_PI/3600.0);// unit 1 [rpm^2]
// DynamixelParameter profile_velocity_pp(560, TYPE_UINT32, 0.01*2.0*M_PI/60.0);// unit 0.01 [rpm]
// DynamixelParameter goal_position_pp42(564, TYPE_INT32, M_PI/303750);
// DynamixelParameter goal_position_pp54(564, TYPE_INT32, M_PI/501923);
// DynamixelParameter present_current_pp(574, TYPE_INT16, 0.001);
// DynamixelParameter present_velocity_pp(576, TYPE_INT32, 0.01*2.0*M_PI/60.0);
// DynamixelParameter present_position_pp42(580, TYPE_INT32, M_PI/303750);
// DynamixelParameter present_position_pp54(580, TYPE_INT32, M_PI/501923);
// DynamixelParameter present_input_voltage_pp(592, TYPE_UINT16, 0.1);
// DynamixelParameter present_temperture_pp(594, TYPE_UINT8, 1.0);
// DynamixelParameter external_port_data_1_pp(600, TYPE_UINT16);
// DynamixelParameter external_port_data_2_pp(602, TYPE_UINT16);
// DynamixelParameter external_port_data_3_pp(604, TYPE_UINT16);
// DynamixelParameter external_port_data_4_pp(606, TYPE_UINT16);
// DynamixelParameter status_return_level_pp(516, TYPE_UINT8);
// DynamixelParameter hardware_error_status_pp(518, TYPE_UINT8);
