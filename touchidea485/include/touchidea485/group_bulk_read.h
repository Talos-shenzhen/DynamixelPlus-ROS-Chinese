/* Authors: Zhwei123 */

#ifndef TOUCHIDEAS_SDK_GROUPBULKREAD_H_
#define TOUCHIDEAS_SDK_GROUPBULKREAD_H_


#include <map>
#include <vector>
#include "port_handler.h"
#include "packet_handler.h"

namespace TouchIdeas485 {

class WINDECLSPEC GroupBulkRead {
 public:
  GroupBulkRead(PortHandler *port, PacketHandler *ph);
  ~GroupBulkRead() { clearParam(); }

  PortHandler* portHandler()   { return port_; }
  PacketHandler* packetHandler() { return ph_; }

  bool addParam(uint8_t id, uint16_t start_address, uint16_t data_length);
  void removeParam(uint8_t id);
  void clearParam();

  CommErrorCode txPacket();
  CommErrorCode rxPacket();
  CommErrorCode txRxPacket();

  bool isAvailable(uint8_t id, uint16_t address, uint16_t data_length);
  uint32_t getData(uint8_t id, uint16_t address, uint16_t data_length);

private:
 PortHandler* port_;
 PacketHandler* ph_;

 // 下面的表定义了要打印区域的伺服 ID，起始寄存器地址，数据长度
 std::vector<uint8_t>            id_list_;
 std::map<uint8_t, uint16_t>     address_list_;  // <id, start_address>
 std::map<uint8_t, uint16_t>     length_list_;   // <id, data_length>
 // 读取的数据放在这里
 std::map<uint8_t, uint8_t *>    data_list_;     // <id, data>

 bool last_result_;
 bool is_param_changed_;

 uint8_t* param_;

 void makeParam();

};

}


#endif /* TOUCHIDEAS_SDK_GROUPBULKREAD_H_ */
