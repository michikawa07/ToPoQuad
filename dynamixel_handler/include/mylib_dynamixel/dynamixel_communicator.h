
#ifndef DYNAMIXEL_COMMUNICATOR_H_
#define DYNAMIXEL_COMMUNICATOR_H_

#include <vector>
#include <cmath>
#include <string>

#include "download/port_handler.h"
#include "dynamixel_parameters.h"

class DynamixelComunicator {
 public:
  enum FactoryResetLevel {
    FACTORY_RESET_EXCLUDE_ID = 0x01,
    FACTORY_RESET_EXCLUDE_ID_BAUDRATE = 0x02,
    FACTORY_RESET_ALL = 0xFF,
  };

  DynamixelComunicator() {
    status_return_level_ = 2;
  }
  DynamixelComunicator(const char *port_name, int baudrate) {
    GetPortHandler(port_name);
    set_baudrate(baudrate);
    status_return_level_ = 2;
  }

  void GetPortHandler(const char *port_name) {
    port_name_ = port_name;
    port_handler_ = PortHandler::getPortHandler(port_name);
  }

  void set_baudrate(int baudrate) {
    baudrate_ = baudrate;
  }

  void set_status_return_level(int level) {
    status_return_level_ = level;
  }

  bool error_last_read(){ return error_last_read_; }

  std::string get_port_name(){
    return std::string(port_name_);
  }

  template <typename T>
  T ValueToIntData(double value, DynamixelParameter dp, T type) {
    return value/dp.value_per_bit();
  }

  bool OpenPort();
  bool ClosePort();
  void Reboot(uint8_t servo_id);
  void FactoryReset(uint8_t servo_id, FactoryResetLevel level);
  bool Ping(uint8_t servo_id);

  void Write(uint8_t servo_id, DynamixelParameter dp, int64_t data_int);
  // void clearMultiTurn(uint8_t servo_id);
  // unsigned short updateCRC(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size);
  int64_t Read(uint8_t servo_id, DynamixelParameter dp);
  void SyncWrite(uint8_t num_servo, uint8_t servo_id_list[], DynamixelParameter dp, int64_t data_int_list[]);
  uint8_t SyncRead(uint8_t num_servo, uint8_t servo_id_list[], DynamixelParameter dp, int64_t data_int_list[], uint8_t read_id_list[]);

 private:
  uint16_t CalcChecksum(uint8_t data[], uint8_t length);
  void EncodeDataWrite(DynamixelDataType type, int64_t data_int);
  int64_t DecodeDataRead(DynamixelDataType type);

  const char *port_name_;
  PortHandler *port_handler_;
  uint32_t baudrate_;
  uint8_t status_return_level_;
  double time_per_byte_;
  uint8_t  data_write_[4];
  uint8_t  data_read_[4];
  bool error_last_read_;
};


#endif /* DYNAMIXEL_COMMUNICATOR_H_ */
