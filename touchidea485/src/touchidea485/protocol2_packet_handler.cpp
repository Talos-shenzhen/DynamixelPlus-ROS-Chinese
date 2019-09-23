/* Authors: Zhwei123 */

#if defined(__linux__)
#include "../../include/touchidea485/protocol2_packet_handler.h"
#include <time.h>
#elif defined(__APPLE__)
#include "protocol2_packet_handler.h"
#elif defined(_WIN32) || defined(_WIN64)
#define WINDLLEXPORT
#include "../../include/touchidea485/protocol2_packet_handler.h"
#endif

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define BOOST_LOG_DYN_LINK 1
#include <boost/log/trivial.hpp>

using namespace TouchIdeas485;

Protocol2PacketHandler::Protocol2PacketHandler() { }

// 返回通讯结果状态码对应的字符串
const char *Protocol2PacketHandler::txRxResult(CommErrorCode result) {
  switch(result) {
    case CommSuccess:
      return "[TxRxResult] Communication success.";
    case CommPortBusy:
      return "[TxRxResult] Port is in use!";
    case CommTxFailed:
      return "[TxRxResult] Failed transmit instruction packet!";
    case CommRxFailed:
      return "[TxRxResult] Failed get status packet from device!";
    case CommTxError:
      return "[TxRxResult] Incorrect instruction packet!";
    case CommRxWaiting:
      return "[TxRxResult] Now recieving status packet!";
    case CommRxTimeout:
      return "[TxRxResult] There is no status packet!";
    case CommRxCorrupt:
      return "[TxRxResult] Incorrect status packet!";
    case CommNotImplement:
      return "[TxRxResult] Protocol does not support This function!";
    case CommCmdEmpty:
      return "[TxRxResult] Command is empty!";
    default:
      return "";
  }
}

// 这个函数根据响应数据报的错误字节来返回对应的错误字符串
const char *Protocol2PacketHandler::rxPacketError(uint8_t error) {
  if (error & ErrBitAlert) {
      int not_alert_error = error & ~ErrBitAlert;
      switch(not_alert_error) {
        case 0:
          return "";
        case ErrResultFailed:
          return "[RxPacketError] Failed to process the instruction packet!";
        case ErrInstruction:
          return "[RxPacketError] Undefined instruction or incorrect instruction!";
        case ErrCRC:
          return "[RxPacketError] CRC doesn't match!";
        case ErrDataRange:
          return "[RxPacketError] The data value is out of range!";
        case ErrDataLength:
          return "[RxPacketError] The data length does not match as expected!";
        case ErrDataLimit:
          return "[RxPacketError] The data value exceeds the limit value!";
        case ErrAccess:
          return "[RxPacketError] Writing or Reading is not available to target address!";
        default:
          return "[RxPacketError] Unknown error code!";
      }
  }
  return "";
}

// 这个函数负责计算数据报的 CRC 校验码
unsigned short Protocol2PacketHandler::updateCRC(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size) {
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

  for (uint16_t j = 0; j < data_blk_size; j++) {
    i = ((uint16_t)(crc_accum >> 8) ^ *data_blk_ptr++) & 0xFF;
    crc_accum = (crc_accum << 8) ^ crc_table[i];
  }

  return crc_accum;
}

// 为了避免数据内部出现头部特征字符串，所以需要添加一些字节
void Protocol2PacketHandler::addStuffing(uint8_t *packet) {
  int i = 0, index = 0;
  uint16_t packet_length_in = makeWord(packet[PktLengthLow], packet[PktLengthHigh]);
  uint16_t packet_length_out = packet_length_in;
  uint8_t temp[4096] = {0};

  for (uint8_t s = PktHeader0; s <= PktLengthHigh; s++)
    temp[s] = packet[s]; // 这里提起报头到长度这一段数据报：FF FF FD XX ID LEN_L LEN_H
  index = PktInstruction;
  for (i = 0; i < packet_length_in - 2; i++)  {// 复制数据报后面的部分，但是不要复制 CRC
    temp[index++] = packet[i+PktInstruction];
    if (packet[i+PktInstruction] == 0xFD && packet[i+PktInstruction-1] == 0xFF && packet[i+PktInstruction-2] == 0xFF) {   // FF FF FD
      temp[index++] = 0xFD;
      packet_length_out++;
    }
  }
  temp[index++] = packet[PktInstruction+packet_length_in-2];
  temp[index++] = packet[PktInstruction+packet_length_in-1];

  if (packet_length_in != packet_length_out) // 因为要添加填充字节，所以需要申请更多的字节
    packet = (uint8_t *)realloc(packet, index * sizeof(uint8_t));

  for (uint8_t s = 0; s < index; s++) // 将数据拷贝回数据报缓存区
    packet[s] = temp[s];
  packet[PktLengthLow] = lowByte(packet_length_out);
  packet[PktLengthHigh] = highByte(packet_length_out);
}

// 从接收的数据报中移除为了避免数据报内部出现头部特征字符串而添加的填充字符串
void Protocol2PacketHandler::removeStuffing(uint8_t *packet) {
  int i = 0, index = 0;
  int packet_length_in = makeWord(packet[PktLengthLow], packet[PktLengthHigh]);
  int packet_length_out = packet_length_in;

  index = PktInstruction;
  for (i = 0; i < packet_length_in - 2; i++) { // except CRC
    if (packet[i+PktInstruction] == 0xFD && packet[i+PktInstruction+1] == 0xFD && packet[i+PktInstruction-1] == 0xFF && packet[i+PktInstruction-2] == 0xFF) {   // FF FF FD FD
      packet_length_out--;
      i++;
    }
    packet[index++] = packet[i+PktInstruction];
  }
  packet[index++] = packet[PktInstruction+packet_length_in-2];
  packet[index++] = packet[PktInstruction+packet_length_in-1];

  packet[PktLengthLow] = lowByte(packet_length_out);
  packet[PktLengthHigh] = highByte(packet_length_out);
}

// 将这个指令数据报发送到串口
CommErrorCode Protocol2PacketHandler::txPacket(PortHandler *port) {
    uint16_t total_packet_length   = 0;
    uint16_t written_packet_length = 0;

    if ( port->isUsing() )
      return CommPortBusy;
    port->setUsing(true);

    addStuffing(_txpacket);  // 填充字节以避免头部特征字符串，出现在数据报内部

    // 检查数据报是否超过了最大数据报长度
    total_packet_length = makeWord(_txpacket[PktLengthLow], _txpacket[PktLengthHigh]) + 7; // 头部 7 字节: HEADER0 HEADER1 HEADER2 RESERVED ID LENGTH_L LENGTH_H
    if (total_packet_length > _MaxBufSize) {
      port->setUsing(false);
      return CommTxError;
    }

    // 构建数据报报头
    _txpacket[PktHeader0]   = 0xFF;
    _txpacket[PktHeader1]   = 0xFF;
    _txpacket[PktHeader2]   = 0xFD;
    _txpacket[PktReserved]  = 0x00;

    // 添加 CRC16 校验码
    uint16_t crc = updateCRC(0, _txpacket, total_packet_length - 2);    // 2: CRC16
    _txpacket[total_packet_length - 2] = lowByte(crc);
    _txpacket[total_packet_length - 1] = highByte(crc);

/*    std::string str;
    for(int i=0; i< total_packet_length; i++) {
	str += hex2String(_txpacket[i]);
	str += ",";
    }*/

    // 发送指令数据报
    port->clearPort();
    written_packet_length = port->writePort(_txpacket, total_packet_length);
    // 设定串口发送这些字节的时间，避免缓存区数据丢失
    double msec =port->len2Time(total_packet_length);
    if (total_packet_length != written_packet_length) {
      port->setUsing(false);
      return CommTxFailed;
    }

    sleepMillSec(msec);
    if (   _txpacket[PktInstruction] == InstSyncWrite )
	  BOOST_LOG_TRIVIAL(info) << "txPacket " << msec*1000000;

    return CommSuccess;
}

// 接收并检验响应数据报的合法性
CommErrorCode Protocol2PacketHandler::rxPacket(PortHandler *port) {
  CommErrorCode result = CommTxFailed;
  uint16_t readCount =0;
  uint16_t rx_length = 0;
  uint16_t wait_length = 11; // 数据报最小长度(HEADER0 HEADER1 HEADER2 RESERVED ID LENGTH_L LENGTH_H INST ERROR CRC16_L CRC16_H)

  while(true) {
      int cnt = port->readPort(&_rxpacket[rx_length], wait_length - rx_length);
      rx_length += cnt;
      readCount += cnt;
      if (rx_length >= wait_length) {
	  uint16_t idx = 0;

	  for (idx = 0; idx < (rx_length - 3); idx++) { //找到数据报头
	      if ((_rxpacket[idx] == 0xFF) && (_rxpacket[idx+1] == 0xFF) &&
	              (_rxpacket[idx+2] == 0xFD) && (_rxpacket[idx+3] != 0xFD))
		  break;
	  }

	  if (idx == 0)  { // 找到了数据报的报头
	      if (_rxpacket[PktReserved] != 0x00 || _rxpacket[PktID] > 0xFC ||
	                      getHalfWord(&_rxpacket[PktLengthLow]) > _MaxBufSize ||
	         _rxpacket[PktInstruction] != 0x55) { // 这是数据报的特征
		  for (uint16_t s = 0; s < rx_length - 1; s++)  // 不对，则移除数据缓存区的第一个字节
		      _rxpacket[s] = _rxpacket[1 + s];
		  rx_length -= 1;
		  continue;
	      }

	      // 找到了报头，重新计算这个数据报的长度
	      if (wait_length != getHalfWord(&_rxpacket[PktLengthLow]) + PktLengthHigh + 1) {
		  wait_length = getHalfWord(&_rxpacket[PktLengthLow]) + PktLengthHigh + 1;
		  continue;
	      }

	      if (rx_length < wait_length) { // 如果接收到的长度小于希望的长度
		  if (port->isPacketTimeout() == true) {
		      if (rx_length == 0) { // 接收超时
			  result = CommRxTimeout;
		      } else {  // 数据报损毁
			  result = CommRxCorrupt;
		      }
		      break;
		  } else {
		      continue;
		  }
	      }

	      // 计算 CRC16 校验码
	      uint16_t crc = getHalfWord(&_rxpacket[wait_length - 2]);
	      if (updateCRC(0, _rxpacket, wait_length - 2) == crc) {// CRC 校验成功
		  result = CommSuccess;
		  break;
	      } else {  // CRC 错误，数据报损毁
		  result = CommRxCorrupt;
		  break;
	      }
	  } else { // 移除报头特征字符串之前的无用字符
	      for (uint16_t s = 0; s < rx_length - idx; s++)
		  _rxpacket[s] = _rxpacket[idx + s];
	      rx_length -= idx;
	  }
      }

      if (port->isPacketTimeout() == true) { // 还没有接收足够的字符，看看是否超时了
	  if (rx_length == 0) {
	      result = CommRxTimeout;
	  } else {
	      result = CommRxCorrupt;
	  }
	  break;
      }
      sleepMillSec(0.1); // 睡眠以等待字节传输
  }
  port->setUsing(false);

  if (result == CommSuccess) {
    removeStuffing(_rxpacket);
  }/* else
      BOOST_LOG_TRIVIAL(info) << port->portName() <<"-"<<(int)_txpacket[PktID]
			      <<":" <<(int)(_txpacket[PktErrParam]+_txpacket[PktErrParam+1]*256)<<":" <<(int)(_txpacket[PktErrParam+2]+_txpacket[PktErrParam+3]*256)  <<">"<<readCount;*/

  return result;
}

// 这个函数发送一个指令数据报，并接收响应状态数据报。但是这个函数无法用于 BulkRead / SyncRead 指令
CommErrorCode Protocol2PacketHandler::txRxPacket(PortHandler *port, uint8_t *error) {
  // 发送指令数据报
  CommErrorCode result = txPacket(port);
  if (result != CommSuccess)
    return result;

  // 如果通讯 ID 为 Broadcast_ID，同时不是 BulkRead，SyncRead；或者是 Action 指令，那么无需响应状态数据报
  if ((_txpacket[PktID] == BoardcastID && _txpacket[PktInstruction] != InstBulkRead) ||
     (_txpacket[PktID] == BoardcastID && _txpacket[PktInstruction] != InstSyncRead) ||
     (_txpacket[PktInstruction] == InstAction)) {
    port->setUsing(false);
    return result;
  }

  // 根据指令，设定我们希望的响应数据报长度
  if (_txpacket[PktInstruction] == InstRead) {
    port->packetTimeout((uint16_t)(getHalfWord(&_txpacket[PktErrParam + 2]) + 11));
  } else {
    port->packetTimeout((uint16_t)11); // 普通指令，响应包为最小长度，HEADER0 HEADER1 HEADER2 RESERVED ID LENGTH_L LENGTH_H INST ERROR CRC16_L CRC16_H
  }

  result = rxPacket(port);  // 接收响应数据报
  //if (result == CommSuccess && _txpacket[PktID] != _rxpacket[PktID])  // 检查指令数据报和响应数据报 ID 是否匹配
  //  result = rxPacket(port);

  if (result == CommSuccess && _txpacket[PktID] != BoardcastID) {
    if (error != 0)
      *error = (uint8_t)_rxpacket[PktErrParam];
  }

  return result;
}

// PING 一个伺服，但是不读取该伺服的型号
CommErrorCode Protocol2PacketHandler::ping(PortHandler *port, uint8_t id, uint8_t *error) {
  return ping(port, id, 0, error);
}

// PING 一个伺服，同时读取该伺服的型号
CommErrorCode Protocol2PacketHandler::ping(PortHandler *port, uint8_t id, uint16_t *model_number, uint8_t *error) {
  CommErrorCode result = CommTxFailed;

  if (id >= BoardcastID)
    return CommNotImplement;

  _txpacket[PktID] = id;
  _txpacket[PktLengthLow] = PingCmdParamLen;
  _txpacket[PktLengthHigh] = 0;
  _txpacket[PktInstruction] = InstPing;

  result = txRxPacket(port, error);
  if (result == CommSuccess && model_number != 0)
    *model_number = getHalfWord(&_rxpacket[PktErrParam +1]);
  return result;
}

// 广播型的 PING，这样一个指令可以找到串口上的所有伺服
CommErrorCode Protocol2PacketHandler::broadcastPing(PortHandler *port, std::vector<uint8_t> &id_list) {
  id_list.clear();

  uint16_t rx_length = 0;
  uint16_t wait_length = PING_STATUS_LENGTH * MAX_ID;

  _txpacket[PktID] = BoardcastID;
  _txpacket[PktLengthLow] = PingCmdParamLen;
  _txpacket[PktLengthHigh] = 0;
  _txpacket[PktInstruction] = InstPing;

  CommErrorCode result = txPacket(port);
  if (result != CommSuccess) {
    port->setUsing(false);
    return result;
  }

  port->packetTimeout((uint16_t)(wait_length * 30)); //设定我们等待状态数据报的超时时间
  while(1) {
      rx_length += port->readPort(&_rxpacket[rx_length], wait_length - rx_length);
      if (port->isPacketTimeout() == true)// || rx_length >= wait_length)
	break;
      sleepMillSec(0.1);
  }

  port->setUsing(false);
  if (rx_length == 0)
      return CommRxTimeout;

  while(1) {
      if (rx_length < PingStatusLength)
	  return CommRxCorrupt;

      uint16_t idx = 0;
      for (idx = 0; idx < (rx_length - 2); idx++) { // 首先查找数据报头部特征字符串
	  if (_rxpacket[idx] == 0xFF && _rxpacket[idx+1] == 0xFF && _rxpacket[idx+2] == 0xFD)
	      break;
      }

      if (idx == 0) {  // 找到了数据报头部特征字符串
	  // 从状态数据报提取 CRC16 校验码
	  uint16_t crc = getHalfWord(&_rxpacket[PingStatusLength-2]);

	  if (updateCRC(0, _rxpacket, PingStatusLength - 2) == crc) { // CRC 校验正确
	      result = CommSuccess;

	      id_list.push_back(_rxpacket[PktID]);
	      for (uint8_t s = 0; s < rx_length - PingStatusLength; s++)
		  _rxpacket[s] = _rxpacket[PingStatusLength + s];
	      rx_length -= PingStatusLength;

	      if (rx_length == 0)
		  return result;
	  } else { // CRC 校验失败
	      result = CommRxCorrupt;

	      // remove header (0xFF 0xFF 0xFD)
	      for (uint8_t s = 0; s < rx_length - 3; s++) //删除报头特征字符串，相当于删除了这个数据报
		  _rxpacket[s] = _rxpacket[3 + s];
	      rx_length -= 3;
	  }
    } else { // 删除头部特征字符串之前的所有接收字节
	  for (uint8_t s = 0; s < rx_length - idx; s++)
	      _rxpacket[s] = _rxpacket[idx + s];
	  rx_length -= idx;
    }
  }

  return result;
}

CommErrorCode Protocol2PacketHandler::action(PortHandler *port, uint8_t id) {
  _txpacket[PktID]            = id;
  _txpacket[PktLengthLow]      = ActionCmdParamLen;
  _txpacket[PktLengthHigh]      = 0;
  _txpacket[PktInstruction]   = InstAction;

  return txRxPacket(port, 0);
}

CommErrorCode Protocol2PacketHandler::reboot(PortHandler *port, uint8_t id, uint8_t *error) {
  _txpacket[PktID]            = id;
  _txpacket[PktLengthLow]      = RebootCmdParamLen;
  _txpacket[PktLengthHigh]      = 0;
  _txpacket[PktInstruction]   = InstReboot;

  return txRxPacket(port, error);
}

CommErrorCode Protocol2PacketHandler::factoryReset(PortHandler *port, uint8_t id, uint8_t option, uint8_t *error) {
  _txpacket[PktID]            = id;
  _txpacket[PktLengthLow]      = FactoryResetCmdParamLen;
  _txpacket[PktLengthHigh]      = 0;
  _txpacket[PktInstruction]   = InstFactoryReset;
  _txpacket[PktErrParam]    = option;

  return txRxPacket(port, error);
}

CommErrorCode Protocol2PacketHandler::homing(PortHandler *port, uint8_t id, uint8_t direction, uint8_t *error) {
    _txpacket[PktID]            = id;
    _txpacket[PktLengthLow]      = HomeCmdParamLen;
    _txpacket[PktLengthHigh]      = 0;
    _txpacket[PktInstruction]   = InstHomig;
    _txpacket[PktErrParam]    = direction;

    return txRxPacket(port, error);
}

CommErrorCode Protocol2PacketHandler::readTx(PortHandler *port, uint8_t id, uint16_t address, uint16_t length) {
  if (id >= BoardcastID)
    return CommNotImplement;

  _txpacket[PktID]            = id;
  _txpacket[PktLengthLow]      = ReadCmdCommonParamLen;
  _txpacket[PktLengthHigh]      = 0;
  _txpacket[PktInstruction]   = InstRead;
  packetHalfWord(&_txpacket[PktErrParam + 0], address);
  packetHalfWord(&_txpacket[PktErrParam + 2], length);

  CommErrorCode result = txPacket(port);

  // set packet timeout
  if (result == CommSuccess)
    port->packetTimeout((uint16_t)(length + 11));

  return result;
}

CommErrorCode Protocol2PacketHandler::readRx(PortHandler *port, uint16_t length, uint8_t *data, uint8_t *error) {
  CommErrorCode result = rxPacket(port);
  if (result == CommSuccess) {
    if (error != 0)
      *error = (uint8_t)_rxpacket[PktErrParam];
    for (uint8_t s = 0; s < length; s++)
      data[s] = _rxpacket[PktErrParam + 1 + s];
  }

  return result;
}

CommErrorCode Protocol2PacketHandler::readTxRx(PortHandler *port, uint8_t id, uint16_t address,
                                               uint16_t *length, uint8_t *data, uint8_t *error) {
  if (id >= BoardcastID)
    return CommNotImplement;

  _txpacket[PktID]            = id;
  _txpacket[PktLengthLow]      = ReadCmdCommonParamLen;
  _txpacket[PktLengthHigh]      = 0;
  _txpacket[PktInstruction]   = InstRead;
  packetHalfWord(&_txpacket[PktErrParam + 0], address);
  packetHalfWord(&_txpacket[PktErrParam + 2], *length);

  CommErrorCode result =txRxPacket(port, error);
  if (result == CommSuccess) {
    if (error != 0)
      *error = (uint8_t)_rxpacket[PktErrParam];
    uint16_t length2 = _rxpacket[PktLengthLow] | static_cast<uint16_t>(_rxpacket[PktLengthHigh]<<8);
    length2 -= 4;
    if ( length2> *length )
	length2 = *length;
    for (uint8_t s = 0; s < length2; s++)
      data[s] = _rxpacket[PktErrParam + 1 + s];
    *length = length2;
  }

  return result;
}

CommErrorCode Protocol2PacketHandler::writeTxOnly(PortHandler *port, uint8_t id, uint16_t address, uint16_t length, uint8_t *data) {
  _txpacket[PktID]            = id;
  packetHalfWord(&_txpacket[PktLengthLow], (uint16_t)(length + WriteCmdCommonParamLen));
  _txpacket[PktInstruction]   = InstWrite;
  packetHalfWord(&_txpacket[PktErrParam + 0], address);

  for (uint8_t s = 0; s < length; s++)
    _txpacket[PktErrParam +2+s] = data[s];

  CommErrorCode result = txPacket(port);
  port->setUsing(false);

  return result;
}

CommErrorCode Protocol2PacketHandler::writeTxRx(PortHandler *port, uint8_t id, uint16_t address, uint16_t length, uint8_t *data, uint8_t *error) {
  _txpacket[PktID]            = id;
  packetHalfWord(&_txpacket[PktLengthLow], (uint16_t)(length + WriteCmdCommonParamLen));
  _txpacket[PktInstruction]   = InstWrite;
  packetHalfWord(&_txpacket[PktErrParam + 0], address);

  for (uint8_t s = 0; s < length; s++)
    _txpacket[PktErrParam +2+s] = data[s];

  CommErrorCode result = txRxPacket(port, error);
  return result;
}

CommErrorCode Protocol2PacketHandler::regWriteTxOnly(PortHandler *port, uint8_t id, uint16_t address, uint16_t length, uint8_t *data) {
  _txpacket[PktID]            = id;
  packetHalfWord(&_txpacket[PktLengthLow], (uint16_t)(length + RegWriteCmdCommonParamLen));
  _txpacket[PktInstruction]   = InstRegWrite;
  packetHalfWord(&_txpacket[PktErrParam + 0], address);

  for (uint16_t s = 0; s < length; s++)
    _txpacket[PktErrParam +2+s] = data[s];

  CommErrorCode result = txPacket(port);
  port->setUsing(false);

  return result;
}

CommErrorCode Protocol2PacketHandler::regWriteTxRx(PortHandler *port, uint8_t id, uint16_t address, uint16_t length, uint8_t *data, uint8_t *error) {
  _txpacket[PktID]            = id;
  packetHalfWord(&_txpacket[PktLengthLow], (uint16_t)(length + RegWriteCmdCommonParamLen));
  _txpacket[PktInstruction]   = InstRegWrite;
  packetHalfWord(&_txpacket[PktErrParam + 0], address);

  for (uint16_t s = 0; s < length; s++)
    _txpacket[PktErrParam +2+s] = data[s];

  CommErrorCode result = txRxPacket(port, error);
  return result;
}

CommErrorCode Protocol2PacketHandler::syncReadTx(PortHandler *port, uint16_t start_address, uint16_t data_length, uint8_t *param, uint16_t param_length) {
  _txpacket[PktID]            = BoardcastID;
  packetHalfWord(&_txpacket[PktLengthLow], (uint16_t)(param_length + SyncCmdCommonParamLen));
  _txpacket[PktInstruction]   = InstSyncRead;
  packetHalfWord(&_txpacket[PktErrParam + 0], start_address);
  packetHalfWord(&_txpacket[PktErrParam + 2], data_length);

  for (uint16_t s = 0; s < param_length; s++)
    _txpacket[PktErrParam +4+s] = param[s];

  CommErrorCode result = txPacket(port);
  if (result == CommSuccess)
    port->packetTimeout((uint16_t)((11 + data_length) * param_length));
  return result;
}

CommErrorCode Protocol2PacketHandler::syncWriteTxOnly(PortHandler *port, uint16_t start_address, uint16_t data_length, uint8_t *param, size_t param_length) {
    _txpacket[PktID]            = BoardcastID;
    packetHalfWord(&_txpacket[PktLengthLow], (uint16_t)(param_length + SyncCmdCommonParamLen));
    _txpacket[PktInstruction]   = InstSyncWrite;
    PacketHandler::packetHalfWord(&_txpacket[PktErrParam + 0], start_address);
    PacketHandler::packetHalfWord(&_txpacket[PktErrParam + 2], data_length);

    for (uint16_t s = 0; s < param_length; s++) {
	_txpacket[PktErrParam +4+s] = param[s];
    }

    CommErrorCode result = txRxPacket(port, 0);
    return result;
}

CommErrorCode Protocol2PacketHandler::bulkReadTx(PortHandler *port, uint8_t *param, size_t param_length) {
  _txpacket[PktID]            = BoardcastID;
  packetHalfWord(&_txpacket[PktLengthLow], (uint16_t)(param_length + BulkCmdCommonParamLen));
  _txpacket[PktInstruction]   = InstBulkRead;

  for (size_t s = 0; s < param_length; s++)
    _txpacket[PktErrParam +s] = param[s];

  CommErrorCode result = txPacket(port);
  if (result == CommSuccess) {
    int wait_length = 0;
    for (int i = 0; i < param_length; i += 5)
      wait_length += getHalfWord(&param[i+3]) + 10;
    port->packetTimeout((uint16_t)wait_length);
  }

  return result;
}

CommErrorCode Protocol2PacketHandler::bulkWriteTxOnly(PortHandler *port, uint8_t *param, uint16_t param_length) {
  _txpacket[PktID]            = BoardcastID;
  packetHalfWord(&_txpacket[PktLengthLow], (uint16_t)(param_length + BulkCmdCommonParamLen));
  _txpacket[PktInstruction]   = InstBulkWrite;

  for (uint16_t s = 0; s < param_length; s++)
    _txpacket[PktErrParam +s] = param[s];

  CommErrorCode result = txRxPacket(port, 0);
  return result;
}
