/* Authors: Zhwei123 */

#ifndef TOUCHIDEAS_SDK_GROUPSYNCWRITE_H_
#define TOUCHIDEAS_SDK_GROUPSYNCWRITE_H_

#include <map>
#include <vector>
#include "port_handler.h"
#include "packet_handler.h"

namespace TouchIdeas485 {

class WINDECLSPEC GroupSyncWrite {
 public:
  GroupSyncWrite(PortHandler *port, PacketHandler *ph, uint16_t start_address, uint16_t data_length);
  ~GroupSyncWrite() { clearParam(); }

  PortHandler* portHandler() { return port_; }
  PacketHandler* packetHandler() { return ph_; }

  void setAddrLen(uint16_t start_address, uint16_t data_length) {
      start_address_ = start_address;
      data_length_ = data_length;
  }

  bool addParam(uint8_t id, uint8_t *data);    // 添加一个目标伺服，以及要设定的数据
  void removeParam(uint8_t id);                // 移除一个目标伺服
  bool changeParam(uint8_t id, uint8_t *data); // 改变一个目标伺服，以及要设定的数据
  void clearParam();                           // 清空所有目标伺服

  bool empty(void) const {
      return (id_list_.size() == 0);
  }

  uint16_t startAddr(void) const { return start_address_; }  //返回我们写入的起始寄存器地址
  uint16_t dataLength(void) const { return data_length_; }  //返回我们写入的数据字节数
  CommErrorCode txPacket(uint8_t sn);

  bool isEmpty(void) const { return (id_list_.size() == 0); }
private:
 PortHandler* port_;
 PacketHandler* ph_;

 // 下面的表定义了要写入的伺服 ID，起始寄存器地址，数据长度，以及数据
 std::vector<uint8_t> id_list_;
 uint16_t start_address_;
 uint16_t data_length_;
 std::map<uint8_t, uint8_t* > data_list_; // <id, data>

 bool is_param_changed_;
 uint8_t* param_;

 void makeParam(uint8_t sn);
};

}


#endif /* TOUCHIDEAS_SDK_GROUPSYNCWRITE_H_ */
