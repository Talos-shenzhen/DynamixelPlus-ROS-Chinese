/* Authors: Zhwei123 */

#ifndef TOUCHIDEAS_SDK_GROUPBULKWRITE_H_
#define TOUCHIDEAS_SDK_GROUPBULKWRITE_H_

#include <map>
#include <vector>
#include "port_handler.h"
#include "packet_handler.h"

namespace TouchIdeas485 {

// 这个类可以一次对多个伺服进行寄存器写入，对多轴联动来说，非常利于同步
class WINDECLSPEC GroupBulkWrite {
 public:
  GroupBulkWrite(PortHandler *port, PacketHandler *ph);
  ~GroupBulkWrite() { clearParam(); }

  PortHandler     *portHandler()   { return port_; }
  PacketHandler   *packetHandler() { return ph_; }

  // 添加一个目标伺服的写入数据设定
  bool addParam(uint8_t id, uint16_t start_address, uint16_t data_length, uint8_t *data);
  void removeParam(uint8_t id);  //删除一个目标伺服
  // 改变一个目标伺服的写入数据设定
  bool changeParam(uint8_t id, uint16_t start_address, uint16_t data_length, uint8_t *data);
  // 清空所有目标伺服
  void clearParam();

  CommErrorCode  txPacket();  // 发送指令数据报

private:
  PortHandler* port_;
  PacketHandler* ph_;

  // 下面的表定义了要写入的伺服 ID，起始寄存器地址，数据长度，以及数据
  std::vector<uint8_t>            id_list_;
  std::map<uint8_t, uint16_t>     address_list_;  // <id, start_address>
  std::map<uint8_t, uint16_t>     length_list_;   // <id, data_length>
  std::map<uint8_t, uint8_t *>    data_list_;     // <id, data>

  bool is_param_changed_;

  // 为了封装仅指令数据报，上面的设定需要转换为一个字节数组
  uint8_t* param_;
  uint16_t param_length_;

  void makeParam();
};

}


#endif /* DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_GROUPBULKWRITE_H_ */
