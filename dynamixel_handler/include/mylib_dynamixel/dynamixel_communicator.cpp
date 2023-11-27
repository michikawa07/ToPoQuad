#include "dynamixel_communicator.h"
#include <stdio.h>

// #define DXL_LOBYTE(w) ((uint8_t)(((uint64_t)(w)) & 0xff))
// #define DXL_HIBYTE(w) ((uint8_t)((((uint64_t)(w)) >> 8) & 0xff))

static const uint8_t INSTRUCTION_PING          = 0x01;
static const uint8_t INSTRUCTION_READ          = 0x02;
static const uint8_t INSTRUCTION_WRITE         = 0x03;
static const uint8_t INSTRUCTION_RED_WRITE     = 0x04;
static const uint8_t INSTRUCTION_ACTION        = 0x05;
static const uint8_t INSTRUCTION_FACTORY_RESET = 0x06;
static const uint8_t INSTRUCTION_REBOOT        = 0x08;
static const uint8_t INSTRUCTION_SYNC_READ     = 0x82;
static const uint8_t INSTRUCTION_SYNC_WRITE    = 0x83;
static const uint8_t INSTRUCTION_BULK_READ     = 0x92;
static const uint8_t INSTRUCTION_BULK_WRITE    = 0x93;

static const uint8_t HEADER[4] = {0xFF, 0xFF, 0xFD, 0x00};

static const uint8_t COM_ERROR_RESULT_FAIL = 0;
static const uint8_t COM_ERROR_INSTRUCTION = 1;
static const uint8_t COM_ERROR_CRC         = 2;
static const uint8_t COM_ERROR_DATA_RANGE  = 3;
static const uint8_t COM_ERROR_DATA_LENGTH = 4;
static const uint8_t COM_ERROR_DATA_LIMIT  = 5;
static const uint8_t COM_ERROR_ACCESS      = 6;
static const uint8_t COM_ERROR_ALERT       = 7;

uint16_t DynamixelComunicator::CalcChecksum(uint8_t data[], uint8_t length) {
  uint16_t crc_accum = 0;
  uint16_t i;
  uint16_t crc_table[256] = {0x0000,
  0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
  0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027,
  0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D,
  0x8077, 0x0072, 0x0050, 0x8055, 0x805F, 0x005A, 0x804B,
  0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
  0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF,
  0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5,
  0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093,
  0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
  0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197,
  0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE,
  0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB,
  0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9,
  0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
  0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176,
  0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162, 0x8123,
  0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
  0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104,
  0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D,
  0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B,
  0x032E, 0x0324, 0x8321, 0x0360, 0x8365, 0x836F, 0x036A,
  0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 0x035C,
  0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
  0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3,
  0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
  0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7,
  0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E,
  0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B,
  0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9,
  0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC,
  0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2, 0x02D0, 0x82D5,
  0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
  0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
  0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264,
  0x8261, 0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E,
  0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219, 0x0208,
  0x820D, 0x8207, 0x0202 };

  for (uint16_t j = 0; j < length; j++) {
    i = ((uint16_t)(crc_accum >> 8) ^ data[j]) & 0xFF;
    crc_accum = (crc_accum << 8) ^ crc_table[i];
  }

  return crc_accum;
}

/** @fn
 * @brief 送信用にdata_write_[]にデータを格納する
 * @param DynamixelDataType type 変換後のデータ型
 * @param int64_t data_int 書き込む値．int型に変換済み．どの型にも対応できるようにint64_t
 * @return void
 */
void DynamixelComunicator::EncodeDataWrite(DynamixelDataType type, int64_t data_int) {
  switch (type) {
    case TYPE_INT8 : {
      int8_t d = int8_t(data_int);
      data_write_[0] = d & 0xFF;
      break;
    }
    case TYPE_UINT8 : {
      uint8_t d = uint8_t(data_int);
      data_write_[0] = d & 0xFF;
      break;
    }
    case TYPE_INT16 : {
      int16_t d = int16_t(data_int);
      data_write_[0] = d & 0xFF;
      data_write_[1] = (d>>8) & 0xFF;
      break;
    }
    case TYPE_UINT16 : {
      uint16_t d = uint16_t(data_int);
      data_write_[0] = d & 0xFF;
      data_write_[1] = (d>>8) & 0xFF;
      break;
    }
    case TYPE_INT32 : {
      int32_t d = int32_t(data_int);
      data_write_[0] = d & 0xFF;
      data_write_[1] = (d>>8) & 0xFF;
      data_write_[2] = (d>>16) & 0xFF;
      data_write_[3] = (d>>24) & 0xFF;
      break;
    }
    case TYPE_UINT32 : {
      uint32_t d = uint32_t(data_int);
      data_write_[0] = d & 0xFF;
      data_write_[1] = (d>>8) & 0xFF;
      data_write_[2] = (d>>16) & 0xFF;
      data_write_[3] = (d>>24) & 0xFF;
      break;
    }
  }
}

/** @fn
 * @brief data_read_[]に格納されたデータをつなげて一つのデータに復元する
 * @param DynamixelDataType type 変換後のデータ型
 * @return (int64_t) 変換後の値．どの型にも対応できるようにint64_t
 */
int64_t DynamixelComunicator::DecodeDataRead(DynamixelDataType type) {
  switch (type) {
    case TYPE_INT8 : {
      int8_t data_int = 0;
      data_int |= data_read_[0] << 0;
      return int64_t(data_int);
      break;
    }
    case TYPE_UINT8 : {
      uint8_t data_int = 0;
      data_int |= data_read_[0] << 0;
      return int64_t(data_int);
      break;
    }
    case TYPE_INT16 : {
      int16_t data_int = 0;
      data_int |= data_read_[0] << 0;
      data_int |= data_read_[1] << 8;
      return int64_t(data_int);
      break;
    }
    case TYPE_UINT16 :  {
      uint16_t data_int = 0;
      data_int |= data_read_[0] << 0;
      data_int |= data_read_[1] << 8;
      return int64_t(data_int);
      break;
    }
    case TYPE_INT32 : {
      int32_t data_int = 0;
      data_int |= data_read_[0] << 0;
      data_int |= data_read_[1] << 8;
      data_int |= data_read_[2] << 16;
      data_int |= data_read_[3] << 24;
      return int64_t(data_int);
      break;
    }
    case TYPE_UINT32 : {
      uint32_t data_int = 0;
      data_int |= data_read_[0] << 0;
      data_int |= data_read_[1] << 8;
      data_int |= data_read_[2] << 16;
      data_int |= data_read_[3] << 24;
      return int64_t(data_int);
      break;
    }
  }
  return 0;
}

bool DynamixelComunicator::OpenPort() {
  if (port_handler_->openPort()) {
    printf("Succeeded to open the port : %s!\n", port_name_);
  } else {
    printf("Failed to open the port : %s!\n", port_name_);
    return false;
  }

  if (port_handler_->setBaudRate(baudrate_)) {
    printf("Succeeded to change the baudrate : %s!\n", port_name_);
    return true;
  } else {
    printf("Failed to change the baudrate : %s!\n", port_name_);
    return false;
  }
}

bool DynamixelComunicator::ClosePort() {
  port_handler_->closePort();
  printf("Close port : %s!\n", port_name_);
  return true;
}

void DynamixelComunicator::Reboot(uint8_t servo_id) {
  uint8_t send_data[10] = {0};
  uint16_t length = 3;
  send_data[0] = HEADER[0];
  send_data[1] = HEADER[1];
  send_data[2] = HEADER[2];
  send_data[3] = HEADER[3];
  send_data[4] = servo_id;
  send_data[5] = length & 0xFF;
  send_data[6] = (length>>8) & 0xFF;
  send_data[7] = INSTRUCTION_REBOOT;  // instruction
  uint16_t sum = CalcChecksum(send_data, 8);
  send_data[8] = sum & 0xFF;
  send_data[9] = (sum>>8) & 0xFF;
  port_handler_->clearPort();
  port_handler_->writePort(send_data, 10);

  if (status_return_level_ == 2) {
    port_handler_->setPacketTimeout( uint16_t(11) );
    while(port_handler_->getBytesAvailable() < 11) {
      if (port_handler_->isPacketTimeout()) {
        printf("Read Error(time out): ID %d, available bytes %d\n", servo_id, port_handler_->getBytesAvailable());
        error_last_read_ = true;
        return;
      }
    }
    uint8_t read_data[11];
    port_handler_->readPort(read_data, 11);
  }

}

void DynamixelComunicator::FactoryReset(uint8_t servo_id, FactoryResetLevel level) {
  uint8_t send_data[11] = {0};
  uint16_t length = 4;
  send_data[0] = HEADER[0];
  send_data[1] = HEADER[1];
  send_data[2] = HEADER[2];
  send_data[3] = HEADER[3];
  send_data[4] = servo_id;
  send_data[5] = length & 0xFF;
  send_data[6] = (length>>8) & 0xFF;
  send_data[7] = INSTRUCTION_FACTORY_RESET;  // instruction
  send_data[8] = level;
  uint16_t sum = CalcChecksum(send_data, 9);
  send_data[9] = sum & 0xFF;
  send_data[10] = (sum>>8) & 0xFF;
  port_handler_->writePort(send_data, 11);

  if (status_return_level_ == 2) {
      uint8_t read_data[11];
      port_handler_->readPort(read_data, 11);
  }
}

bool DynamixelComunicator::Ping(uint8_t servo_id) {
  uint8_t send_data[10] = {0};
  uint16_t length = 3;
  send_data[0] = HEADER[0];
  send_data[1] = HEADER[1];
  send_data[2] = HEADER[2];
  send_data[3] = HEADER[3];
  send_data[4] = servo_id;
  send_data[5] = length & 0xFF;
  send_data[6] = (length>>8) & 0xFF;
  send_data[7] = INSTRUCTION_PING;  // instruction
  uint16_t sum = CalcChecksum(send_data, 8);
  send_data[8] = sum & 0xFF;
  send_data[9] = (sum>>8) & 0xFF;
  port_handler_->writePort(send_data, 10);

  uint8_t read_data[14];
  if(port_handler_->readPort(read_data, 14) == 14 and read_data[4] == servo_id) {
    return true;
  } else {
    return false;
  }
}

/** @fn
 * @brief Dynamixelに情報を書き込む
 * @param uint8_t servo_id 対象のID
 * @param DynamixelParameter dp 対象のパラメータのインスタンス
 * @param int64_t data_int 書き込むデータ．intに変換済みのもの．どの型にも対応できるようにint64_t
 * @return void
 */
void DynamixelComunicator::Write(uint8_t servo_id, DynamixelParameter dp, int64_t data_int) {
  uint8_t send_data[20] = {0};
  uint16_t length = dp.size()+5;
  send_data[0] = HEADER[0];
  send_data[1] = HEADER[1];
  send_data[2] = HEADER[2];
  send_data[3] = HEADER[3];
  send_data[4] = servo_id;
  send_data[5] = length & 0xFF;
  send_data[6] = (length>>8) & 0xFF;
  send_data[7] = INSTRUCTION_WRITE;  // instruction
  send_data[8] = dp.address() & 0xFF;
  send_data[9] = (dp.address()>>8) & 0xFF;
  EncodeDataWrite(dp.data_type(), data_int);
  for(int i=0; i<dp.size(); i++) {
    send_data[10+i] = data_write_[i];
  }
  uint16_t sum = CalcChecksum(send_data, 10+dp.size());
  send_data[10+dp.size()] = sum & 0xFF;
  send_data[11+dp.size()] = (sum>>8) & 0xFF;

  port_handler_->clearPort();
  port_handler_->writePort(send_data, 12+dp.size());

  if (status_return_level_ == 2) {
    port_handler_->setPacketTimeout( uint16_t(11) );
    while(port_handler_->getBytesAvailable() < 11) {
      if (port_handler_->isPacketTimeout()) {
        printf("Read Error(time out): ID %d, available bytes %d\n", servo_id, port_handler_->getBytesAvailable());
        error_last_read_ = true;
        return;
      }
    }
    uint8_t read_data[11];
    port_handler_->readPort(read_data, 11);
  }
}


// /** @fn
//  * 
//  * あとで
//  */
// void DynamixelComunicator::clearMultiTurn(uint8_t servo_id) {
//   uint8_t send_data[15] = {0};
//   // make packet header
//   send_data[0]   = 0xFF;
//   send_data[1]   = 0xFF;
//   send_data[2]   = 0xFD;
//   send_data[3]  = 0x00;
//   send_data[4]            = servo_id;
//   send_data[5]      = 8;
//   send_data[6]      = 0;
//   send_data[7]     = 16;
//   send_data[8]    = 0x01;
//   send_data[8+1]  = 0x44;
//   send_data[8+2]  = 0x58;
//   send_data[8+3]  = 0x4C;
//   send_data[8+4]  = 0x22;
//   uint16_t crc = updateCRC(0, send_data, 15 - 2);    // 2: CRC16
//   send_data[13] = DXL_LOBYTE(crc);
//   send_data[14] = DXL_HIBYTE(crc);
  
//   port_handler_->clearPort();
//   port_handler_->writePort(send_data, 15);

// }

// unsigned short DynamixelComunicator::updateCRC(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size)
// {
//   uint16_t i;
//   static const uint16_t crc_table[256] = {0x0000,
//   0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
//   0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027,
//   0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D,
//   0x8077, 0x0072, 0x0050, 0x8055, 0x805F, 0x005A, 0x804B,
//   0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
//   0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF,
//   0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5,
//   0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093,
//   0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
//   0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197,
//   0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE,
//   0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB,
//   0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9,
//   0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
//   0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176,
//   0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162, 0x8123,
//   0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
//   0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104,
//   0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D,
//   0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B,
//   0x032E, 0x0324, 0x8321, 0x0360, 0x8365, 0x836F, 0x036A,
//   0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 0x035C,
//   0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
//   0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3,
//   0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
//   0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7,
//   0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E,
//   0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B,
//   0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9,
//   0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC,
//   0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2, 0x02D0, 0x82D5,
//   0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
//   0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
//   0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264,
//   0x8261, 0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E,
//   0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219, 0x0208,
//   0x820D, 0x8207, 0x0202 };

//   for (uint16_t j = 0; j < data_blk_size; j++)
//   {
//     i = ((uint16_t)(crc_accum >> 8) ^ *data_blk_ptr++) & 0xFF;
//     crc_accum = (crc_accum << 8) ^ crc_table[i];
//   }

//   return crc_accum;
// }

/** @fn
 * @brief Dynamixelから情報を読み込む
 * @param uint8_t servo_id 対象のID
 * @param DynamixelParameter dp 対象のパラメータのインスタンス
 * @param int64_t data_int 書き込むデータ．intに変換済みのもの．どの型にも対応できるようにint64_t
 * @return (int64_t) 読み込んだデータ．intのまま
 */
int64_t DynamixelComunicator::Read(uint8_t servo_id, DynamixelParameter dp) {
  uint8_t send_data[14] = {0};
  uint16_t length = 7;
  send_data[0] = HEADER[0];
  send_data[1] = HEADER[1];
  send_data[2] = HEADER[2];
  send_data[3] = HEADER[3];
  send_data[4] = servo_id;
  send_data[5] = length & 0xFF;
  send_data[6] = (length>>8) & 0xFF;
  send_data[7] = INSTRUCTION_READ;  // instruction
  send_data[8] = dp.address() & 0xFF;
  send_data[9] = (dp.address()>>8) & 0xFF;
  send_data[10] = dp.size() & 0xFF;
  send_data[11] = (dp.size()>>8) & 0xFF;
  uint16_t sum = CalcChecksum(send_data, 12);
  send_data[12] = sum & 0xFF;
  send_data[13] = (sum>>8) & 0xFF;

  port_handler_->clearPort();
  port_handler_->writePort(send_data, 14);

  error_last_read_ = false;
  port_handler_->setPacketTimeout( uint16_t(11+dp.size()) );
  while(port_handler_->getBytesAvailable() < 11+dp.size()) {
    if (port_handler_->isPacketTimeout()) {
      printf("Read Error(time out): ID %d, available bytes %d\n", servo_id, port_handler_->getBytesAvailable());
      error_last_read_ = true;
      return 0;
    }
  }

  if (status_return_level_ != 0) {
    uint8_t read_data[20];
    uint8_t read_length = port_handler_->readPort(read_data, 11+dp.size());
    if (read_length != 11+dp.size()) {
      printf("Read Error(no data): ID %d, data_length = %d\n", servo_id, read_length);
      error_last_read_ = true;
      return 0;
    }
    if (read_data[0] != HEADER[0] or
        read_data[1] != HEADER[1] or
        read_data[2] != HEADER[2] or
        read_data[3] != HEADER[3] or
        read_data[4] != servo_id) {
      printf("Read Error(header): ID %d\n", servo_id);
      error_last_read_ = true;
      return 0;
    }
    uint16_t sum_est = CalcChecksum(read_data, 9+dp.size());
    uint16_t sum_read = uint16_t(read_data[9+dp.size()]) | uint16_t(read_data[9+dp.size()+1])<<8;
    if (sum_est != sum_read) {
      printf("Read Error(crc): ID %d, est:%d, read:%d\n", servo_id, sum_est, sum_read);
      error_last_read_ = true;
      return 0;
    }

    // 正常なデータ
    for(int i=0; i<dp.size(); i++) {
      data_read_[i] = read_data[9+i];
    }
    return DecodeDataRead(dp.data_type());
  }
  return 0;
}

void DynamixelComunicator::SyncWrite(uint8_t num_servo, uint8_t servo_id_list[], DynamixelParameter dp, int64_t data_int_list[]) {
  uint8_t send_data[512] = {0};
  uint16_t length = (1+dp.size())*num_servo+7;
  send_data[0] = HEADER[0];
  send_data[1] = HEADER[1];
  send_data[2] = HEADER[2];
  send_data[3] = HEADER[3];
  send_data[4] = 0xFE;  // id
  send_data[5] = length & 0xFF;
  send_data[6] = (length>>8) & 0xFF;
  send_data[7] = INSTRUCTION_SYNC_WRITE;  // instruction
  send_data[8] = dp.address() & 0xFF;
  send_data[9] = (dp.address()>>8) & 0xFF;
  send_data[10] = dp.size() & 0xFF;
  send_data[11] = (dp.size()>>8) & 0xFF;
  for(int i_servo=0; i_servo<num_servo; i_servo++) {
    send_data[12+i_servo*(dp.size()+1)] = servo_id_list[i_servo];
    EncodeDataWrite(dp.data_type(), data_int_list[i_servo]);
    for(int i_data=0; i_data<dp.size(); i_data++) {
      send_data[12+i_servo*(dp.size()+1)+1+i_data] = data_write_[i_data];
    }
  }
  uint16_t sum = CalcChecksum(send_data, 12+num_servo*(dp.size()+1));
  send_data[12+num_servo*(dp.size()+1)] = sum & 0xFF;
  send_data[13+num_servo*(dp.size()+1)] = (sum>>8) & 0xFF;

  port_handler_->clearPort();
  port_handler_->writePort(send_data, 14+num_servo*(dp.size()+1));
}

uint8_t DynamixelComunicator::SyncRead(uint8_t num_servo, uint8_t servo_id_list[], DynamixelParameter dp,
                            int64_t data_int_list[], uint8_t read_id_list[]) {
  uint8_t send_data[128] = {0};
  uint16_t length = num_servo+7;
  send_data[0] = HEADER[0];
  send_data[1] = HEADER[1];
  send_data[2] = HEADER[2];
  send_data[3] = HEADER[3];
  send_data[4] = 0xFE;  // id
  send_data[5] = length & 0xFF;
  send_data[6] = (length>>8) & 0xFF;
  send_data[7] = INSTRUCTION_SYNC_READ;  // instruction
  send_data[8] = dp.address() & 0xFF;
  send_data[9] = (dp.address()>>8) & 0xFF;
  send_data[10] = dp.size() & 0xFF;
  send_data[11] = (dp.size()>>8) & 0xFF;
  for(int i_servo=0; i_servo<num_servo; i_servo++) {
    send_data[12+i_servo] = servo_id_list[i_servo];
  }
  uint16_t sum = CalcChecksum(send_data, 12+num_servo);
  send_data[12+num_servo] = sum & 0xFF;
  send_data[13+num_servo] = (sum>>8) & 0xFF;

  port_handler_->clearPort();
  port_handler_->writePort(send_data, 14+num_servo);

  uint8_t read_data[20];
  uint8_t num_read = 0;
  for(int i_servo=0; i_servo<num_servo; i_servo++) {
    port_handler_->setPacketTimeout( uint16_t(11+dp.size()) );
    while(port_handler_->getBytesAvailable() < 11+dp.size()) {
      if (port_handler_->isPacketTimeout()) {
        printf("Read Error(time out): ID %d, available bytes %d\n", servo_id_list[i_servo], port_handler_->getBytesAvailable());
        error_last_read_ = true;
        return 0;
      }
    }

    if (port_handler_->readPort(read_data, 11+dp.size()) == -1) {
      printf("Read Error : ID %d\n", servo_id_list[i_servo]);
      continue;
    }
    if (read_data[0] != HEADER[0] or
        read_data[1] != HEADER[1] or
        read_data[2] != HEADER[2] or
        read_data[3] != HEADER[3]) {
      printf("Read Error(header): ID %d\n", servo_id_list[i_servo]);
      continue;
    }
    uint16_t sum_read = CalcChecksum(read_data, 9+dp.size());
    uint8_t sum_l = sum_read & 0xFF;
    uint8_t sum_h  = (sum_read>>8) & 0xFF;
    if (read_data[9+dp.size()] != sum_l or
        read_data[10+dp.size()] != sum_h) {
      printf("Read Error(crc): ID %d\n", servo_id_list[i_servo]);
      error_last_read_ = true;
      continue;
    }
    // 正常なデータ
    for(int i_data=0; i_data<dp.size(); i_data++) {
      data_read_[i_data] = read_data[9+i_data];
    }
    num_read++;
    read_id_list[i_servo] = read_data[4];
    data_int_list[i_servo] = DecodeDataRead(dp.data_type());
  }
  return num_read;
}
