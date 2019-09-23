/* Authors: Zhwei123 */

#ifndef TOUCHIDEAS_SDK_GROUPSYNCREAD_H_
#define TOUCHIDEAS_SDK_GROUPSYNCREAD_H_


#include <map>
#include <vector>
#include "port_handler.h"
#include "packet_handler.h"

namespace TouchIdeas485 {

// 同步读取可以一次读取多个伺服的一段寄存器数据，这个方式所有伺服读取的寄存器地址必须相同
class WINDECLSPEC GroupSyncRead {
 public:
  GroupSyncRead(PortHandler* port, PacketHandler* ph, uint16_t start_address, uint16_t data_length);
  ~GroupSyncRead() { clearParam(); }

  PortHandler* portHandler()   { return port_; }
  PacketHandler* packetHandler() { return ph_; }

  bool addParam(uint8_t id);     // 添加一个目标伺服
  void removeParam(uint8_t id);  // 移除一个目标伺服
  void clearParam();             // 清空所有目标伺服

  CommErrorCode txPacket();
  CommErrorCode rxPacket();
  CommErrorCode txRxPacket();

  bool isAvailable(uint8_t id, uint16_t address, uint16_t data_length);
  uint32_t getData(uint8_t id, uint16_t address, uint16_t data_length);

private:
 PortHandler* port_;
 PacketHandler* ph_;

 // 下面的表定义了要写入的伺服 ID，起始寄存器地址，数据长度，以及数据
 std::vector<uint8_t> id_list_;
 uint16_t start_address_;
 uint16_t data_length_;
 std::map<uint8_t, uint8_t* > data_list_; // <id, data>

 bool last_result_;
 bool is_param_changed_;

 uint8_t* param_;

 void makeParam();
};

}


#endif /* TOUCHIDEAS_SDK_GROUPSYNCREAD_H_ */
