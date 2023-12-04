
#ifndef DYNAMIXEL_PARAMETERS_H_
#define DYNAMIXEL_PARAMETERS_H_

#include <stdint.h>

enum DynamixelBaudrateIndex {
  BAUDRATE_INDEX_9600 = 0,
  BAUDRATE_INDEX_57600 = 1,
  BAUDRATE_INDEX_115200 = 2,
  BAUDRATE_INDEX_1M = 3,
  BAUDRATE_INDEX_2M = 4,
  BAUDRATE_INDEX_3M = 5,
  BAUDRATE_INDEX_4M = 6,
  BAUDRATE_INDEX_4M5 = 7,
  BAUDRATE_INDEX_10M5 = 8,
};
enum DynamixelOperatingMode {
  OPERATING_MODE_TORQUE = 0,
  OPERATING_MODE_VELOCITY = 1,
  OPERATING_MODE_POSITION = 3,
  OPERATING_MODE_EXTENDED_POSITION = 4,
  OPERATING_MODE_CURRENT_BASE_POSITION = 5,
  OPERATING_MODE_PWM = 16,
};
enum DynamixelExternalPortMode {
  EXTERNAL_PORT_MODE_AIN = 0,
  EXTERNAL_PORT_MODE_DOUT_NOPULL = 1,
  EXTERNAL_PORT_MODE_DOUT_PULLUP = 2,
  EXTERNAL_PORT_MODE_DOUT_PULLDOWN = 3,
};
enum DynamixelTorquePermission {
  TORQUE_DISABLE = 0,
  TORQUE_ENABLE = 1,
};
enum DynamixelStatusReturnLevel {
  STATUS_RETURN_LEVEL_PING_ONLY = 0,
  STATUS_RETURN_LEVEL_READ_ONLY = 1,
  STATUS_RETURN_LEVEL_ALL = 2,
};
enum DynamixelHardwareErrorIndex {
  HARDWARE_ERROR_INPUT_VOLTAGE = 0,
  HARDWARE_ERROR_MOTOR_HALL_SENSOR = 1,
  HARDWARE_ERROR_OVERHEATING = 2,
  HARDWARE_ERROR_MOTOR_ENCODER = 3,
  HARDWARE_ERROR_ELECTRONICAL_SHOCK = 4,
  HARDWARE_ERROR_OVERLOAD = 5,
};
enum DynamixelDataType {
  TYPE_INT8,
  TYPE_UINT8,
  TYPE_INT16,
  TYPE_UINT16,
  TYPE_INT32,
  TYPE_UINT32,
};

class DynamixelParameter {
 public:
//  DynamixelParameter();
  DynamixelParameter( uint16_t address, DynamixelDataType data_type, double value_per_bit=1.0 ) {
    address_ = address;
    data_type_ = data_type;
    if (data_type_ == TYPE_INT8 or data_type_ == TYPE_UINT8) {
      size_ = 1;
    } else if (data_type_ == TYPE_INT16 or data_type_ == TYPE_UINT16) {
      size_ = 2;
    } else if (data_type_ == TYPE_INT32 or data_type_ == TYPE_UINT32) {
      size_ = 4;
    }
    value_per_bit_ = value_per_bit;
  }

  uint16_t address() const { return address_; }
  DynamixelDataType data_type() const { return data_type_; }
  uint16_t size() const { return size_; }
  double value_per_bit() const { return value_per_bit_; }

 private:
  uint16_t address_;
  uint8_t size_;
  DynamixelDataType data_type_;
  double value_per_bit_;
};

// dynamixel x
extern DynamixelParameter id_x                   ;
extern DynamixelParameter baudrate_x             ;
extern DynamixelParameter return_deray_time_x    ;
extern DynamixelParameter operating_mode_x       ;
extern DynamixelParameter homing_offset_x        ;
extern DynamixelParameter pwm_limit_x            ;
extern DynamixelParameter current_limit_x        ;
extern DynamixelParameter current_limit_xh430v   ;
extern DynamixelParameter current_limit_x_330_   ;
extern DynamixelParameter acceleration_limit_x   ;
extern DynamixelParameter velocity_limit_x       ;
extern DynamixelParameter max_position_limit_x   ;
extern DynamixelParameter min_position_limit_x   ;
extern DynamixelParameter external_port_mode_1_x ;
extern DynamixelParameter external_port_mode_2_x ;
extern DynamixelParameter external_port_mode_3_x ;
extern DynamixelParameter shutdown_x             ;
extern DynamixelParameter torque_enable_x        ;
extern DynamixelParameter led_x                  ;
extern DynamixelParameter status_return_level_x  ;
extern DynamixelParameter hardware_error_status_x;
extern DynamixelParameter velocity_i_gain_x      ;
extern DynamixelParameter velocity_p_gain_x      ;
extern DynamixelParameter position_d_gain_x      ;
extern DynamixelParameter position_i_gain_x      ;
extern DynamixelParameter position_p_gain_x      ;
extern DynamixelParameter goal_current_x         ;
extern DynamixelParameter goal_current_xh430v    ;
extern DynamixelParameter goal_current_x_330_    ;
extern DynamixelParameter goal_velocity          ;
extern DynamixelParameter profile_acceleration_x ;
extern DynamixelParameter profile_velocity_x     ;
extern DynamixelParameter goal_position_x        ;
extern DynamixelParameter present_current_x      ;
extern DynamixelParameter present_velocity_x     ;
extern DynamixelParameter present_position_x     ;
extern DynamixelParameter present_input_voltage_x;
extern DynamixelParameter present_temperture_x   ;
extern DynamixelParameter external_port_data_1_x ;
extern DynamixelParameter external_port_data_2_x ;
extern DynamixelParameter external_port_data_3_x ;
// // dynamixel pro plus
// extern DynamixelParameter id_pp;
// extern DynamixelParameter baudrate_pp;
// extern DynamixelParameter return_deray_time_pp;
// extern DynamixelParameter drive_mode_pp;
// extern DynamixelParameter operating_mode_pp;
// extern DynamixelParameter homing_offset_pp42;
// extern DynamixelParameter homing_offset_pp54;
// extern DynamixelParameter current_limit_pp;
// extern DynamixelParameter acceleration_limit_pp;
// extern DynamixelParameter velocity_limit_pp;
// extern DynamixelParameter max_position_limit_pp42;
// extern DynamixelParameter max_position_limit_pp54;
// extern DynamixelParameter min_position_limit_pp42;
// extern DynamixelParameter min_position_limit_pp54;
// extern DynamixelParameter external_port_mode_1_pp;
// extern DynamixelParameter external_port_mode_2_pp;
// extern DynamixelParameter external_port_mode_3_pp;
// extern DynamixelParameter external_port_mode_4_pp;
// extern DynamixelParameter shutdown_pp;
// extern DynamixelParameter torque_enable_pp;
// extern DynamixelParameter led_red_pp;
// extern DynamixelParameter led_green_pp;
// extern DynamixelParameter led_blue_pp;
// extern DynamixelParameter velocity_i_gain_pp;
// extern DynamixelParameter velocity_p_gain_pp;
// extern DynamixelParameter position_d_gain_pp;
// extern DynamixelParameter position_i_gain_pp;
// extern DynamixelParameter position_p_gain_pp;
// extern DynamixelParameter profile_acceleration_pp;
// extern DynamixelParameter profile_velocity_pp;
// extern DynamixelParameter goal_current_pp;
// extern DynamixelParameter goal_velocity_pp;
// extern DynamixelParameter goal_position_pp42;
// extern DynamixelParameter goal_position_pp54;
// extern DynamixelParameter present_current_pp;
// extern DynamixelParameter present_velocity_pp;
// extern DynamixelParameter present_position_pp42;
// extern DynamixelParameter present_position_pp54;
// extern DynamixelParameter present_input_voltage_pp;
// extern DynamixelParameter present_temperture_pp;
// extern DynamixelParameter external_port_data_1_pp;
// extern DynamixelParameter external_port_data_2_pp;
// extern DynamixelParameter external_port_data_3_pp;
// extern DynamixelParameter external_port_data_4_pp;
// extern DynamixelParameter status_return_level_pp;
// extern DynamixelParameter hardware_error_status_pp;

#endif /* DYNAMIXEL_PARAMETERS_H_ */
