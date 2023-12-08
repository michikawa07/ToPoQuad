#include "dynamixel_parameters.h"
#include <math.h>

// dynamixel X series   
namespace dyn_x {
    DynamixelParameter id                    (  7, TYPE_UINT8 );
    DynamixelParameter baudrate              (  8, TYPE_UINT8 );
    DynamixelParameter return_deray_time     (  9, TYPE_UINT8 );
    DynamixelParameter operating_mode        ( 11, TYPE_UINT8 );
    DynamixelParameter homing_offset         ( 20, TYPE_INT32 ); // M_PI/2048.0);
    DynamixelParameter pwm_limit             ( 36, TYPE_UINT16); //, 100.0/885 /*%*/);
    DynamixelParameter current_limit         ( 38, TYPE_UINT16); //, def: 2.69/*mA*/, xh430v: 1.34/*mA*/ x_330_: 1.00/*mA*/
    DynamixelParameter acceleration_limit    ( 40, TYPE_UINT32); //, 214.577*2.0*M_PI/3600.0);
    DynamixelParameter velocity_limit        ( 44, TYPE_UINT32); //, 0.229*2.0*M_PI/60.0/*rad/s*/  XL330: /*rad/s*/
    DynamixelParameter max_position_limit    ( 48, TYPE_UINT32); //, M_PI/2048.0);
    DynamixelParameter min_position_limit    ( 52, TYPE_UINT32); //, M_PI/2048.0);
    DynamixelParameter external_port_mode_1  ( 56, TYPE_UINT8 );
    DynamixelParameter external_port_mode_2  ( 57, TYPE_UINT8 );
    DynamixelParameter external_port_mode_3  ( 58, TYPE_UINT8 );
    DynamixelParameter shutdown              ( 63, TYPE_UINT8 );
    DynamixelParameter torque_enable         ( 64, TYPE_UINT8 );
    DynamixelParameter led                   ( 65, TYPE_UINT8 );
    DynamixelParameter status_return_level   ( 68, TYPE_UINT8 );
    DynamixelParameter hardware_error_status ( 70, TYPE_UINT8 );
    DynamixelParameter velocity_i_gain       ( 76, TYPE_UINT16); // 型番によってデフォルト値が異なる
    DynamixelParameter velocity_p_gain       ( 78, TYPE_UINT16); // 型番によってデフォルト値が異なる
    DynamixelParameter position_d_gain       ( 80, TYPE_UINT16); // 型番によってデフォルト値が異なる
    DynamixelParameter position_i_gain       ( 82, TYPE_UINT16); // 型番によってデフォルト値が異なる
    DynamixelParameter position_p_gain       ( 84, TYPE_UINT16); // 型番によってデフォルト値が異なる
    DynamixelParameter goal_current          (102, TYPE_INT16 ); //, def: 2.69/*mA*/, xh430v: 1.34/*mA*/ x_330_: 1.00/*mA*/
    DynamixelParameter goal_velocity         (104, TYPE_INT32 ); //, 0.229*2.0*M_PI/60.0);
    DynamixelParameter profile_acceleration  (108, TYPE_INT32 ); //, 214.577*2.0*M_PI/3600.0); // unit 214.577 [rpm^2]
    DynamixelParameter profile_velocity      (112, TYPE_INT32 ); //, 0.229*2.0*M_PI/60.0 );    // unit 0.229  [rpm]
    DynamixelParameter goal_position         (116, TYPE_INT32 ); //, M_PI/2048.0);
    DynamixelParameter present_current       (126, TYPE_INT16 ); //, def: 2.69/*mA*/, xh430v: 1.34/*mA*/ x_330_: 1.00/*mA*/
    DynamixelParameter present_velocity      (128, TYPE_INT32 ); //, M_PI/180.0*0.229);
    DynamixelParameter present_position      (132, TYPE_INT32 ); //, M_PI/2048.0);
    DynamixelParameter present_input_voltage (144, TYPE_UINT16); //, 0.1);
    DynamixelParameter present_temperture    (146, TYPE_UINT8 );
    DynamixelParameter external_port_data_1  (152, TYPE_UINT16);
    DynamixelParameter external_port_data_2  (154, TYPE_UINT16);
    DynamixelParameter external_port_data_3  (156, TYPE_UINT16);
}

// dynamixel P series (old: pro plus)
namespace dyn_p {
    DynamixelParameter id                   ( 7,  TYPE_UINT8 );
    DynamixelParameter baudrate             ( 8,  TYPE_UINT8 );
    DynamixelParameter return_deray_time    ( 9,  TYPE_UINT8 );
    DynamixelParameter drive_mode           ( 10, TYPE_UINT8 );
    DynamixelParameter operating_mode       ( 11, TYPE_UINT8 );
    DynamixelParameter homing_offset        ( 20, TYPE_INT32 ); //, 42: M_PI/303750 54: M_PI/501923
    DynamixelParameter current_limit        ( 38, TYPE_UINT16); //, 0.001);
    DynamixelParameter acceleration_limit   ( 38, TYPE_UINT32); //, 58000*2.0*M_PI/3600.0); //unit 58000 [rpm^2]
    DynamixelParameter velocity_limit       ( 38, TYPE_UINT32); //, 0.01*2.0*M_PI/60.0); //unit 0.01 [rpm]
    DynamixelParameter max_position_limit   ( 48, TYPE_INT32 ); //, 42: M_PI/303750 54: M_PI/501923
    DynamixelParameter min_position_limit   ( 52, TYPE_INT32 ); //, 42: M_PI/303750 54: M_PI/501923
    DynamixelParameter external_port_mode_1 ( 56, TYPE_UINT8 );
    DynamixelParameter external_port_mode_2 ( 57, TYPE_UINT8 );
    DynamixelParameter external_port_mode_3 ( 58, TYPE_UINT8 );
    DynamixelParameter external_port_mode_4 ( 59, TYPE_UINT8 );
    DynamixelParameter shutdown             ( 63, TYPE_UINT8 );
    DynamixelParameter torque_enable        (512, TYPE_UINT8 );
    DynamixelParameter led_red              (513, TYPE_UINT8 );
    DynamixelParameter led_green            (514, TYPE_UINT8 );
    DynamixelParameter led_blue             (515, TYPE_UINT8 );
    DynamixelParameter velocity_i_gain      (524, TYPE_UINT16);
    DynamixelParameter velocity_p_gain      (526, TYPE_UINT16);
    DynamixelParameter position_d_gain      (528, TYPE_UINT16);
    DynamixelParameter position_i_gain      (530, TYPE_UINT16);
    DynamixelParameter position_p_gain      (532, TYPE_UINT16);
    DynamixelParameter goal_current         (550, TYPE_INT16 ); //, 0.001);
    DynamixelParameter goal_velocity        (552, TYPE_INT32 ); //, 0.01*2.0*M_PI/60.0);
    DynamixelParameter profile_acceleration (556, TYPE_UINT32); //, 1.0*2.0*M_PI/3600.0);// unit 1 [rpm^2]
    DynamixelParameter profile_velocity     (560, TYPE_UINT32); //, 0.01*2.0*M_PI/60.0);// unit 0.01 [rpm]
    DynamixelParameter goal_position        (564, TYPE_INT32 ); //, 42: M_PI/303750 54: M_PI/501923
    DynamixelParameter present_current      (574, TYPE_INT16 ); //, 0.001);
    DynamixelParameter present_velocity     (576, TYPE_INT32 ); //, 0.01*2.0*M_PI/60.0);
    DynamixelParameter present_position     (580, TYPE_INT32 ); //, 42: M_PI/303750 54: M_PI/501923
    DynamixelParameter present_input_voltage(592, TYPE_UINT16); //, 0.1);
    DynamixelParameter present_temperture   (594, TYPE_UINT8 ); //, 1.0);
    DynamixelParameter external_port_data_1 (600, TYPE_UINT16);
    DynamixelParameter external_port_data_2 (602, TYPE_UINT16);
    DynamixelParameter external_port_data_3 (604, TYPE_UINT16);
    DynamixelParameter external_port_data_4 (606, TYPE_UINT16);
    DynamixelParameter status_return_level  (516, TYPE_UINT8 );
    DynamixelParameter hardware_error_status(518, TYPE_UINT8 );
}
