/* Authors: Zhwei123 */

#ifndef TOUCHIDEAS_SDK_PACKETHANDLER_H_
#define TOUCHIDEAS_SDK_PACKETHANDLER_H_

#include <stdio.h>
#include <vector>
#include <string>
#include "port_handler.h"

extern const uint8_t BoardcastID;  // 广播地址ID
extern const uint8_t MaxID;  // 设备可用的最大地址ID
#define MAX_ID              0xFC    // 252

// 这些宏用来在字节（1字节），半字（2字节）和整字（4字节）之间进行转换

inline uint16_t makeWord(uint8_t a, uint8_t b) { return (uint16_t)( a | ((uint16_t)(b << 8))); }
inline uint32_t makeDWord(uint16_t a, uint16_t b) { return (uint32_t)( a | ((uint32_t)(b << 16))); }

inline uint16_t lowWord(uint32_t l) { return ((uint16_t)(l & 0xffff)); }
inline uint16_t highWord(uint32_t l) { return ((uint16_t)((l >> 16) & 0xffff)); }

inline uint8_t lowByte(uint16_t w) { return (uint8_t)(w & 0xff); }
inline uint8_t highByte(uint16_t w) { return (uint8_t)((w >> 8) & 0xff); }

enum  TouchIdeasInstruction { // 这里定义用于 TouchIdeas 的通讯包的命令字类型
    // 在 Dynamixel 1.0 协议中出现的命令
    InstPing =1,
    InstRead =2,
    InstWrite =3,
    InstRegWrite =4,
    InstAction =5,
    InstFactoryReset =6,
    InstSyncWrite =0x83,
    InstBulkRead  =0x92,

    // 在 Dynamixel 2.0 协议中出现的命令
    InstReboot =8,
    InstStatus =0x55,
    InstSyncRead = 0x82,
    InstBulkWrite = 0x93,

    // TouchIdeas 协议添加的命令
    InstHomig =9
};

enum CommErrorCode {  // 通讯操作的返回状态码
    CommSuccess      =0,         // 协议数据报收发成功
    CommPortBusy     = -1000,    // 端口忙，被其它线程占用
    CommTxFailed     = -1001,    // 发送指令数据报失败
    CommRxFailed     = -1002,    // 接收状态数据报失败
    CommTxError      = -2000,    // 不正确的指令数据报
    CommRxWaiting    = -3000,    // 正在等待状态数据报
    CommRxTimeout    = -3001,    // 接收状态数据报超时
    CommRxCorrupt    = -3002,    // 接收状态数据报有错误
    CommNotImplement = -9000,    // 说明当前尚不支持这个操作
    CommCmdEmpty     = -9001     // 命令数据报是空的
};

namespace TouchIdeas485 {

/// @brief 这个类用来定义一个协议数据报的解析和封装接口，不同的协议版本分别对应不同的派生类
class WINDECLSPEC PacketHandler {
 protected:
  PacketHandler() { }

 public:
  /// @brief 这个函数根据给定的版本参数返回一个协议句柄
  /// @return PacketHandler 的对应版本的句柄
  static PacketHandler *packetHandler(double protocol_version = 2.0);

  virtual ~PacketHandler() { }

  // 将一个字节转换为对应的十六进制字符串
  static std::string hex2String(uint8_t hex);

  static void packetHalfWord(uint8_t *data, uint16_t val) {
      data[0] =lowByte(val);
      data[1] =highByte(val);
  }

  static void packetHalfWord(uint8_t *data, int16_t val) {
      data[0] =lowByte(val);
      data[1] =highByte(val);
  }

  static void packetWord(uint8_t *data, int32_t val) {
      data[0] = lowByte(lowWord(val));
      data[1] = highByte(lowWord(val));
      data[2] = lowByte(highWord(val));
      data[3] = highByte(highWord(val));
  }

  static void packetWord(uint8_t *data, uint32_t val) {
      data[0] = lowByte(lowWord(val));
      data[1] = highByte(lowWord(val));
      data[2] = lowByte(highWord(val));
      data[3] = highByte(highWord(val));
  }

  static uint16_t getHalfWord(uint8_t *data) {
          return makeWord(data[0], data[1]);
  }

  static uint32_t getWord(uint8_t *data) {
          return makeDWord(makeWord(data[0], data[1]), makeWord(data[2], data[3]));
  }

  static void sleepMillSec(double msec);  // 睡眠指定的毫秒数，精度为纳秒

  /// @brief 返回当前句柄支持的通讯协议版本
  /// @return 通讯协议版本
  virtual float   protocolVersion() = 0;

  /// @brief 使用这个函数获取通讯错误码对应的描述字符串
  /// @param result 通讯错误码
  /// @return 错误码对应的描述字符串
  virtual const char *txRxResult(CommErrorCode result) = 0;

  /// @brief 根据状态响应包中的硬件错误字，返回对应的描述字符串
  /// @param error 状态响应包中的硬件错误字
  /// @return 状态响应包中的硬件错误字对应的描述字符串
  virtual const char *rxPacketError(uint8_t error) = 0;
  static inline bool isHardError(uint8_t error) { return (error&0x7f); }

  /// @brief 通过指定的串口发送一个命令数据报
  /// @description 这个函数会调用 PortHandler::clearPort() 清空串口的发送缓存区，然后调用 PortHandler::writePort() 发送数据包
  /// @description 只有当串口空闲，并且数据报写入发送缓存区后才会实际发送数据
  /// @param port 发送数据报使用的串口
  /// @param txpacket 要发送的命令数据报
  /// @return 如果串口忙，返回 CommPortBusy；如果数据报长度超过 TXPacketMaxLen 则返回 CommTxError；如果数据报太短，返回 CommTxFailed
  /// @return 发送成功返回 CommSuccess
  virtual CommErrorCode txPacket(PortHandler *port) = 0;

  /// @brief 这个函数会尝试从指定串口接收一个状态数据报
  /// @description 这个函数会持续调用 PortHandler::readPort() 以读取数据报，当 PortHandler::isPacketTimeout() 返回超时时它会中断循环退出；
  /// @description 或者发现数据报损毁，也会中断
  /// @param port 接收数据报使用的串口
  /// @param rxpacket 接收到的状态数据报
  /// @return 如果数据报损坏（没有头部；ID,长度，错误字非法；数据报太短），返回 CommRxCorrupt；如果超时，返回 CommRxTimeout
  /// @return 成功接收，校验失败返回 CommRxFailed；否则返回 CommSuccess
  virtual CommErrorCode rxPacket(PortHandler *port) = 0;

  /// @brief 这个函数从指定串口发送一个命令数据报，然后接收该数据包的响应状态数据报
  /// @description 首先它调用 txPacket() 发送数据报，如果不成功则返回错误码；成功则调用 rxPacket() 接收状态响应
  /// @param port 发送数据报使用的串口
  /// @param txpacket 要发送的命令数据报
  /// @param rxpacket 接收到的状态数据报
  /// @return 如果发送和接收都成功，则返回 CommSuccess；否则返回 txPacket() 或 rxPacket() 返回的错误码
  virtual CommErrorCode txRxPacket(PortHandler *port, uint8_t *error = 0) = 0;

  /// @brief 这个函数 PING 一个伺服，但是不获取其型号
  /// @description 这是一个获取型号的 ping() 函数的一个简化版本，只是设定 model_number 指针为空
  /// @param port 发送数据报使用的串口
  /// @param id 伺服的通讯地址
  /// @param error 伺服的硬件状态字
  /// @return 下面 的 ping() 函数返回的通讯状态码
  virtual CommErrorCode ping(PortHandler *port, uint8_t id, uint8_t *error = 0) = 0;

  /// @brief 这个函数 PING 一个伺服，同时获取该设备的型号
  /// @description 这个函数会构造 InstPing 命令数据报, 然后使用 txRxPacket() 发送数据报接收响应；成功则通过 readTxRx() 函数读取响应中的设备型号
  /// @description 如果 ID 是 BROADCAST_ID，那么会直接退出
  /// @param port 发送数据报使用的串口
  /// @param id 伺服的通讯地址
  /// @param error 伺服的硬件状态字
  /// @return 如果指定的地址是 BROADCAST_ID，则返回 CommNotImplement；发送接收成功，则返回 CommSuccess
  /// @return 其它错误则是由于其调用的 txRxPacket() 和 readTxRx() 所返回
  virtual CommErrorCode ping(PortHandler *port, uint8_t id, uint16_t *model_number, uint8_t *error = 0) = 0;

  /// @brief (只有 2.0 协议支持)，这个函数会 PING 串口上连接的所有设备
  /// @param port 发送数据报使用的串口
  /// @param id_list 这个列表中是 PING 成功的设备 ID 列表
  /// @return 如果 1.0 版本，因为不支持这个指令，返回 CommNotImplement
  virtual CommErrorCode broadcastPing(PortHandler *port, std::vector<uint8_t> &id_list) = 0;

  /// @brief 这个函数发送一个动作指令，和寄存器写指令配合，主要用于需要多个寄存器写入的动作可以在全部指令下发后才同时更新
  /// @description 这个函数会构造 InstAction 命令数据报, 然后使用 txRxPacket() 发送数据报接收响应
  /// @description 在这个命令前，你通常需要通过 regWriteTxOnly()  或者 regWriteTxRx() 先写入寄存器
  /// @param port 发送数据报使用的串口
  /// @param id 伺服的通讯地址
  /// @return 返回 txRxPacket() 函数执行的结果错误码
  virtual CommErrorCode action(PortHandler *port, uint8_t id) = 0;

  /// @brief 这个命令会命令一个伺服重启，这个函数会构造 InstReboot 命令数据报, 然后使用 txRxPacket() 发送数据报接收响应
  /// @description 发送成功，那么伺服会重启，重启时可以看到它的 LED 灯闪烁
  /// @param port 发送数据报使用的串口
  /// @param id 伺服的通讯地址
  /// @param error 伺服的硬件状态字
  /// @return 如果是 1.0 版本的协议，会直接返回 CommNotImplement
  virtual CommErrorCode reboot(PortHandler *port, uint8_t id, uint8_t *error = 0) = 0;

  /// @brief 这个函数请求伺服将 EEPROM 参数回复到出厂设定，这个函数会构造 InstFactoryReset 命令数据报, 然后使用 txRxPacket() 发送数据报接收响应
  /// @description 因为执行后你设定的参数都会丢失，所以请谨慎使用
  /// @param port 发送数据报使用的串口
  /// @param id 伺服的通讯地址
  /// @param option 回复出厂设定的选项，涉及是否回复 ID 和波特率
  /// @param error 伺服的硬件状态字
  /// @return 返回 txRxPacket() 函数执行的结果错误码
  virtual CommErrorCode factoryReset(PortHandler *port, uint8_t id, uint8_t option = 0, uint8_t *error = 0) = 0;

  /// @brief 这个函数会请求伺服进行一次零位操作，以找到其零位，这个函数会构造 InstHomig 命令数据报, 然后使用 txRxPacket() 发送数据报接收响应
  /// @description 因为会产生实际运动，所以要谨慎使用
  /// @param port 发送数据报使用的串口
  /// @param id 伺服的通讯地址
  /// @param direction 零位搜寻的起始方向，0=CW, 1=CCW
  /// @param error 伺服的硬件状态字
  /// @return 返回 txRxPacket() 函数执行的结果错误码
  virtual CommErrorCode homing(PortHandler *port, uint8_t id, uint8_t direction = 0, uint8_t *error = 0) = 0;

  /// @brief 这个函数负责读取指定地址的寄存器，这个函数会构造 InstRead 命令数据报, 然后使用 txRxPacket() 发送数据报接收响应
  /// @description 如果 ID 是 BROADCAST_ID，那么会直接退出
  /// @param port 发送数据报使用的串口
  /// @param id 伺服的通讯地址
  /// @param address 要读取的寄存器开始地址
  /// @param length 要读取的字节数
  /// @return 地址为 BROADCAST_ID，则返回 CommNotImplement；否则返回 txPacket() 函数执行的结果错误码
  virtual CommErrorCode readTx(PortHandler *port, uint8_t id, uint16_t address, uint16_t length) = 0;

  /// @brief 这个函数负责从状态数据报中读取寄存器数据信息，通常前面有发送一个 InstRead 指令数据报。
  /// @param port 发送数据报使用的串口
  /// @param length 要读取的字节数
  /// @param data 从数据报中读取到的数据
  /// @param error 伺服的硬件状态字
  /// @return 返回 rxPacket() 函数执行的结果错误码
  virtual CommErrorCode readRx(PortHandler *port, uint16_t length, uint8_t *data, uint8_t *error = 0) = 0;

  /// @brief 这个函数发送一个 InstRead 指令数据报，然后使用 txRxPacket() 发送数据报接收响应，从中读取寄存器的数据
  /// @description 如果 ID 是 BROADCAST_ID，那么会直接退出
  /// @param port 发送数据报使用的串口
  /// @param id 伺服的通讯地址
  /// @param address 要读取的寄存器开始地址
  /// @param length 要读取的字节数
  /// @param data 从数据报中读取到的数据
  /// @param error 伺服的硬件状态字
  /// @return 地址为 BROADCAST_ID，则返回 CommNotImplement；否则返回 txPacket() 函数执行的结果错误码
  virtual CommErrorCode readTxRx(PortHandler *port, uint8_t id, uint16_t address,
                                 uint16_t *length, uint8_t *data, uint8_t *error = 0) = 0;

  /// @brief 这个函数调用 readTx() 来读取 1 字节的寄存器数据
  /// @param port 发送数据报使用的串口
  /// @param id 伺服的通讯地址
  /// @param address 要读取的寄存器开始地址
  /// @return 返回 readTx() 函数执行的结果错误码
  virtual CommErrorCode read1ByteTx(PortHandler *port, uint8_t id, uint16_t address);

  /// @brief 这个函数调用 readRx() 来从响应数据报汇总读取 1 字节的寄存器数据
  /// @param port 发送数据报使用的串口
  /// @param data 从数据报中读取到的数据
  /// @param error 伺服的硬件状态字
  /// @return 返回 readRx() 函数执行的结果错误码
  virtual CommErrorCode read1ByteRx(PortHandler *port, uint8_t *data, uint8_t *error = 0);

  /// @brief 这个函数调用 readTxRx() 来读取 1 字节的寄存器数据
  /// @param port 发送数据报使用的串口
  /// @param id 伺服的通讯地址
  /// @param address 要读取的寄存器开始地址
  /// @param data 从数据报中读取到的数据
  /// @param error 伺服的硬件状态字
  /// @return 返回 txRxPacket() 函数执行的结果错误码
  virtual CommErrorCode read1ByteTxRx(PortHandler *port, uint8_t id, uint16_t address, uint8_t *data, uint8_t *error = 0);

  /// @brief 这个函数调用 readTx() 来读取 2 字节的寄存器数据
  /// @param port 发送数据报使用的串口
  /// @param id 伺服的通讯地址
  /// @param address 要读取的寄存器开始地址
  /// @return 返回 readTx() 函数执行的结果错误码
  virtual CommErrorCode read2ByteTx(PortHandler *port, uint8_t id, uint16_t address);

  /// @brief 这个函数调用 readRx() 来从响应数据报汇总读取 2 字节的寄存器数据
  /// @param port 发送数据报使用的串口
  /// @param data 从数据报中读取到的数据
  /// @param error 伺服的硬件状态字
  /// @return 返回 readRx() 函数执行的结果错误码
  virtual CommErrorCode read2ByteRx(PortHandler *port, uint16_t *data, uint8_t *error = 0);

  /// @brief 这个函数调用 readTxRx() 来读取 2 字节的寄存器数据
  /// @param port 发送数据报使用的串口
  /// @param id 伺服的通讯地址
  /// @param address 要读取的寄存器开始地址
  /// @param data 从数据报中读取到的数据
  /// @param error 伺服的硬件状态字
  /// @return 返回 txRxPacket() 函数执行的结果错误码
  virtual CommErrorCode read2ByteTxRx(PortHandler *port, uint8_t id, uint16_t address,
                                      uint16_t *data, uint8_t *error = 0);

  /// @brief 这个函数调用 readTx() 来读取 4 字节的寄存器数据
  /// @param port 发送数据报使用的串口
  /// @param id 伺服的通讯地址
  /// @param address 要读取的寄存器开始地址
  /// @return 返回 readTx() 函数执行的结果错误码
  virtual CommErrorCode read4ByteTx(PortHandler *port, uint8_t id, uint16_t address);

  /// @brief 这个函数调用 readRx() 来从响应数据报汇总读取 4 字节的寄存器数据
  /// @param port 发送数据报使用的串口
  /// @param data 从数据报中读取到的数据
  /// @param error 伺服的硬件状态字
  /// @return 返回 readRx() 函数执行的结果错误码
  virtual CommErrorCode read4ByteRx(PortHandler *port, uint32_t *data, uint8_t *error = 0);

  /// @brief 这个函数调用 readTxRx() 来读取 4 字节的寄存器数据
  /// @param port 发送数据报使用的串口
  /// @param id 伺服的通讯地址
  /// @param address 要读取的寄存器开始地址
  /// @param data 从数据报中读取到的数据
  /// @param error 伺服的硬件状态字
  /// @return 返回 txRxPacket() 函数执行的结果错误码
  virtual CommErrorCode read4ByteTxRx(PortHandler *port, uint8_t id, uint16_t address,
                                      uint32_t *data, uint8_t *error = 0);

  /// @brief 这个函数发送一个 InstWrite 命令数据报来写入寄存器，它使用 txPacket() 发送数据报.
  /// @param port 发送数据报使用的串口
  /// @param id 伺服的通讯地址
  /// @param address 要写入的寄存器开始地址
  /// @param length 要写入的字节数
  /// @param data 要写入到寄存器中的数据
  /// @return 返回 txPacket() 函数执行的结果错误码
  virtual CommErrorCode writeTxOnly(PortHandler *port, uint8_t id, uint16_t address, uint16_t length, uint8_t *data) = 0;

  /// @brief 这个函数发送一个 InstWrite 指令数据报，并接收响应数据报，从中读取硬件错误字
  /// @param port 发送数据报使用的串口
  /// @param id 伺服的通讯地址
  /// @param address 要写入的寄存器开始地址
  /// @param length 要写入的字节数
  /// @param data 要写入到寄存器中的数据
  /// @param error 伺服的硬件状态字
  /// @return 返回 txRxPacket() 函数执行的结果错误码
  virtual CommErrorCode writeTxRx(PortHandler *port, uint8_t id, uint16_t address,
                                  uint16_t length, uint8_t *data, uint8_t *error = 0) = 0;

  /// @brief 这个函数使用 writeTxOnly() 写入 1 字节的数据
  /// @param port 发送数据报使用的串口
  /// @param id 伺服的通讯地址
  /// @param address 要写入的寄存器开始地址
  /// @param data 要写入到寄存器中的数据
  /// @return 返回 writeTxOnly() 函数执行的结果错误码
  virtual CommErrorCode write1ByteTxOnly(PortHandler *port, uint8_t id, uint16_t address, uint8_t data);

  /// @brief 这个函数调用 writeTxRx() 来对寄存器写入 1 字节的数据，并接收响应包，读取硬件错误字
  /// @param port 发送数据报使用的串口
  /// @param id 伺服的通讯地址
  /// @param address 要写入的寄存器开始地址
  /// @param data 要写入到寄存器中的数据
  /// @param error 伺服的硬件状态字
  /// @return 返回 writeTxRx() 函数执行的结果错误码
  virtual CommErrorCode write1ByteTxRx(PortHandler *port, uint8_t id, uint16_t address,
                                       uint8_t data, uint8_t *error = 0);

  /// @brief 这个函数使用 writeTxOnly() 写入 2 字节的数据
  /// @param port 发送数据报使用的串口
  /// @param id 伺服的通讯地址
  /// @param address 要写入的寄存器开始地址
  /// @param data 要写入到寄存器中的数据
  /// @return 返回 writeTxOnly() 函数执行的结果错误码
  virtual CommErrorCode write2ByteTxOnly(PortHandler *port, uint8_t id,
                                         uint16_t address, uint16_t data);

  /// @brief 这个函数调用 writeTxRx() 来对寄存器写入 2 字节的数据，并接收响应包，读取硬件错误字
  /// @param port 发送数据报使用的串口
  /// @param id 伺服的通讯地址
  /// @param address 要写入的寄存器开始地址
  /// @param data 要写入到寄存器中的数据
  /// @param error 伺服的硬件状态字
  /// @return 返回 writeTxRx() 函数执行的结果错误码
  virtual CommErrorCode write2ByteTxRx(PortHandler *port, uint8_t id, uint16_t address,
                                       uint16_t data, uint8_t *error = 0);

  /// @brief 这个函数使用 writeTxOnly() 写入 4 字节的数据
  /// @param port 发送数据报使用的串口
  /// @param id 伺服的通讯地址
  /// @param address 要写入的寄存器开始地址
  /// @param data 要写入到寄存器中的数据
  /// @return 返回 writeTxOnly() 函数执行的结果错误码
  virtual CommErrorCode write4ByteTxOnly(PortHandler *port, uint8_t id,
                                         uint16_t address, uint32_t data);

  /// @brief 这个函数调用 writeTxRx() 来对寄存器写入 4 字节的数据，并接收响应包，读取硬件错误字
  /// @param port 发送数据报使用的串口
  /// @param id 伺服的通讯地址
  /// @param address 要写入的寄存器开始地址
  /// @param data 要写入到寄存器中的数据
  /// @param error 伺服的硬件状态字
  /// @return 返回 writeTxRx() 函数执行的结果错误码
  virtual CommErrorCode write4ByteTxRx(PortHandler *port, uint8_t id, uint16_t address,
                                       uint32_t data, uint8_t *error = 0);

  /// @brief 这个函数发送 InstRegWrite 指令数据报，这个命令会先让数据缓存到伺服中，等到 InstAction 指令时才更新对应寄存器
  /// @param port 发送数据报使用的串口
  /// @param id 伺服的通讯地址
  /// @param address 要写入的寄存器开始地址
  /// @param length 要读取的字节数
  /// @param data 要写入到寄存器中的数据
  /// @return 返回 txPacket() 函数执行的结果错误码
  virtual CommErrorCode regWriteTxOnly(PortHandler *port, uint8_t id, uint16_t address, uint16_t length, uint8_t *data) = 0;

  /// @brief 这个函数发送 InstRegWrite 指令数据报，这个命令会先让数据缓存到伺服中，等到 InstAction 指令时才更新对应寄存器，并从状态数据报中读取硬件错误字
  /// @param port 发送数据报使用的串口
  /// @param id 伺服的通讯地址
  /// @param address 要写入的寄存器开始地址
  /// @param length 要读取的字节数
  /// @param data 要写入到寄存器中的数据
  /// @param error 伺服的硬件状态字
  /// @return 返回 txRxPacket() 函数执行的结果错误码
  virtual CommErrorCode regWriteTxRx(PortHandler *port, uint8_t id, uint16_t address, uint16_t length, uint8_t *data, uint8_t *error = 0) = 0;

  /// @brief （只有版本 2.0 支持）这个函数发送 InstSyncRead 指令数据报，调用 txPacket() 发送数据报，被 GroupSyncRead 使用
  /// @param port 发送数据报使用的串口
  /// @param start_address 同步读取的寄存器起始地址
  /// @param data_length 同步读取的数据字节数
  /// @param param 同步读取的 ID 列表，因为同步读取可读取多个伺服，这里列出要读取的伺服通讯 ID 列表
  /// @param param_length 伺服通讯 ID 列表的长度
  /// @return 返回 txPacket() 函数执行的结果错误码
  virtual CommErrorCode syncReadTx(PortHandler *port, uint16_t start_address, uint16_t data_length, uint8_t *param, uint16_t param_length) = 0;

  /// @brief 这个函数发送 InstSyncWrite 指令数据报，调用 txRxPacket() 发送指令，但并没有处理响应数据报，被 GroupSyncWrite 使用
  /// @param port 发送数据报使用的串口
  /// @param start_address 同步读取的寄存器起始地址
  /// @param data_length 同步读取的数据字节数
  /// @param param 同步读取的 ID 列表，因为同步读取可读取多个伺服，这里列出要读取的伺服通讯 ID 列表，后面是写入的数据
  /// @param param_length 伺服通讯 ID 列表的长度*（1+数据字节数）
  /// @return 返回 txRxPacket() 函数执行的结果错误码
  virtual CommErrorCode syncWriteTxOnly(PortHandler *port, uint16_t start_address, uint16_t data_length, uint8_t *param, size_t param_length) = 0;

  /// @brief 这个函数发送 InstBulkRead 指令数据报，使用 txPacket() 发送，被 GroupBulkRead 使用
  /// @param port 发送数据报使用的串口
  /// @param param 块读取也可以一次读取多个伺服，每个伺服除 ID 外，还需要指定起始地址（2字节），数据长度（2字节）
  /// @param param_length 读取命令的参数长度，伺服数*5字节，
  /// @return 返回 txPacket() 函数执行的结果错误码
  virtual CommErrorCode bulkReadTx(PortHandler *port, uint8_t *param, size_t param_length) = 0;

  /// @brief 这个函数发送 InstBulkWrite 指令数据报，使用txRxPacket() 发送，被 GroupBulkWrite 使用
  /// @param port 发送数据报使用的串口
  /// @param param 命令的参数字节，长度参考下面的参数
  /// @param param_length 读取命令的参数长度，伺服数*（5字节+数据长度），
  /// @return 返回 txRxPacket() 函数执行的结果错误码
  virtual CommErrorCode bulkWriteTxOnly(PortHandler *port, uint8_t *param, uint16_t param_length) = 0;
};

}


#endif /* DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PACKETHANDLER_H_ */
