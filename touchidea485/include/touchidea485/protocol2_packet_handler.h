/* Authors: Zhwei123 */

#ifndef TOUCHIDEAS_SDK_PROTOCOL2PACKETHANDLER_H_
#define TOUCHIDEAS_SDK_PROTOCOL2PACKETHANDLER_H_

#include "packet_handler.h"

namespace TouchIdeas485 {

/// @brief 这个类负责 Dynamixel 2.0 协议版本数据报的解析和封装
class WINDECLSPEC Protocol2PacketHandler : public PacketHandler {
public:

    enum PacketItem {   // 这个枚举变量用来定义数据报中各位置的意义
	PktHeader0       =0,
	PktHeader1       =1,
	PktHeader2       =2,
	PktReserved      =3,
	PktID            =4,
	PktLengthLow     =5,
	PktLengthHigh    =6,
	PktInstruction   =7,
	PktErrParam         =8
    };

    enum ErrNumber {     // 当 ErrBitAlert 被置位时，错误字的含义代表硬件错误；否则是下面的含义
	ErrResultFailed  =1,   // 处理指令数据报失败
	ErrInstruction   =2,   // 错误指令
	ErrCRC           =3,   // CRC 校验错误
	ErrDataRange     =4,   // 数据范围错误
	ErrDataLength    =5,   // 数据长度错误
	ErrDataLimit     =6,   // 数据上下限错误
	ErrAccess        =7,   // 数据访问权限错误
	ErrBitAlert      =128  // 这个是硬件错误标志位
    };

    Protocol2PacketHandler();
    virtual ~Protocol2PacketHandler() { }

    /// @brief 返回当前句柄支持的通讯协议版本
    /// @return 通讯协议版本
    float   protocolVersion() { return 2.0; }

    /// @brief 根据状态响应包中的硬件错误字，返回对应的描述字符串
    /// @param error 状态响应包中的硬件错误字
    /// @return 状态响应包中的硬件错误字对应的描述字符串
    const char *txRxResult(CommErrorCode result);

    /// @brief 根据状态响应包中的硬件错误字，返回对应的描述字符串
    /// @param error 状态响应包中的硬件错误字
    /// @return 状态响应包中的硬件错误字对应的描述字符串
    const char *rxPacketError  (uint8_t error);

    /// @brief 通过指定的串口发送一个命令数据报
    /// @description 这个函数会调用 PortHandler::clearPort() 清空串口的发送缓存区，然后调用 PortHandler::writePort() 发送数据包
    /// @description 只有当串口空闲，并且数据报写入发送缓存区后才会实际发送数据
    /// @param port 发送数据报使用的串口
    /// @param txpacket 要发送的命令数据报
    /// @return 如果串口忙，返回 CommPortBusy；如果数据报长度超过 TXPacketMaxLen 则返回 CommTxError；如果数据报太短，返回 CommTxFailed
    /// @return 发送成功返回 CommSuccess
    CommErrorCode txPacket(PortHandler *port);

    /// @brief 这个函数会尝试从指定串口接收一个状态数据报
    /// @description 这个函数会持续调用 PortHandler::readPort() 以读取数据报，当 PortHandler::isPacketTimeout() 返回超时时它会中断循环退出；
    /// @description 或者发现数据报损毁，也会中断
    /// @param port 接收数据报使用的串口
    /// @param rxpacket 接收到的状态数据报
    /// @return 如果数据报损坏（没有头部；ID,长度，错误字非法；数据报太短），返回 CommRxCorrupt；如果超时，返回 CommRxTimeout
    /// @return 成功接收，校验失败返回 CommRxFailed；否则返回 CommSuccess
    CommErrorCode rxPacket(PortHandler *port);

    /// @brief 这个函数从指定串口发送一个命令数据报，然后接收该数据包的响应状态数据报
    /// @description 首先它调用 txPacket() 发送数据报，如果不成功则返回错误码；成功则调用 rxPacket() 接收状态响应
    /// @param port 发送数据报使用的串口
    /// @param txpacket 要发送的命令数据报
    /// @param rxpacket 接收到的状态数据报
    /// @return 如果发送和接收都成功，则返回 CommSuccess；否则返回 txPacket() 或 rxPacket() 返回的错误码
    CommErrorCode txRxPacket(PortHandler *port, uint8_t *error = 0);

    /// @brief 这个函数 PING 一个伺服，但是不获取其型号
    /// @description 这是一个获取型号的 ping() 函数的一个简化版本，只是设定 model_number 指针为空
    /// @param port 发送数据报使用的串口
    /// @param id 伺服的通讯地址
    /// @param error 伺服的硬件状态字
    /// @return ping() 返回的通讯状态码
    CommErrorCode ping(PortHandler *port, uint8_t id, uint8_t *error = 0);

    /// @brief 这个函数 PING 一个伺服，同时获取该设备的型号
    /// @description 这个函数会构造 InstPing 命令数据报, 然后使用 txRxPacket() 发送数据报接收响应；成功则通过 readTxRx() 函数读取响应中的设备型号
    /// @description 如果 ID 是 BoardcastID，那么会直接退出
    /// @param port 发送数据报使用的串口
    /// @param id 伺服的通讯地址
    /// @param error 伺服的硬件状态字
    /// @return 如果指定的地址是 BoardcastID，则返回 CommNotImplement；发送接收成功，则返回 CommSuccess
    /// @return 其它错误则是由于其调用的 txRxPacket() 和 readTxRx() 所返回
    CommErrorCode ping(PortHandler *port, uint8_t id, uint16_t *model_number, uint8_t *error = 0);

    /// @brief (只有 2.0 协议支持)，这个函数会 PING 串口上连接的所有设备
    /// @param port 发送数据报使用的串口
    /// @param id_list 这个列表中是 PING 成功的设备 ID 列表
    /// @return 如果出错，返回对应的错误码；成功则返回 CommSuccess
    CommErrorCode broadcastPing(PortHandler *port, std::vector<uint8_t> &id_list);

    /// @brief 这个函数发送一个动作指令，和寄存器写指令配合，主要用于需要多个寄存器写入的动作可以在全部指令下发后才同时更新
    /// @description 这个函数会构造 InstAction 命令数据报, 然后使用 txRxPacket() 发送数据报接收响应
    /// @description 在这个命令前，你通常需要通过 regWriteTxOnly()  或者 regWriteTxRx() 先写入寄存器
    /// @param port 发送数据报使用的串口
    /// @param id 伺服的通讯地址
    /// @return 返回 txRxPacket() 函数执行的结果错误码
    CommErrorCode action(PortHandler *port, uint8_t id);

    /// @brief 这个命令会命令一个伺服重启，这个函数会构造 InstReboot 命令数据报, 然后使用 txRxPacket() 发送数据报接收响应
    /// @description 发送成功，那么伺服会重启，重启时可以看到它的 LED 灯闪烁
    /// @param port 发送数据报使用的串口
    /// @param id 伺服的通讯地址
    /// @param error 伺服的硬件状态字
    /// @return 返回 txRxPacket() 函数执行的结果错误码
    CommErrorCode reboot(PortHandler *port, uint8_t id, uint8_t *error = 0);

    /// @brief 这个函数请求伺服将 EEPROM 参数回复到出厂设定，这个函数会构造 InstFactoryReset 命令数据报, 然后使用 txRxPacket() 发送数据报接收响应
    /// @description 因为执行后你设定的参数都会丢失，所以请谨慎使用
    /// @param port 发送数据报使用的串口
    /// @param id 伺服的通讯地址
    /// @param option 回复出厂设定的选项，涉及是否回复 ID 和波特率
    /// @param error 伺服的硬件状态字
    /// @return 返回 txRxPacket() 函数执行的结果错误码
    CommErrorCode factoryReset(PortHandler *port, uint8_t id, uint8_t option, uint8_t *error = 0);

    /// @brief 这个函数会请求伺服进行一次零位操作，以找到其零位，这个函数会构造 InstHomig 命令数据报, 然后使用 txRxPacket() 发送数据报接收响应
    /// @description 因为会产生实际运动，所以要谨慎使用
    /// @param port 发送数据报使用的串口
    /// @param id 伺服的通讯地址
    /// @param direction 零位搜寻的起始方向，0=CW, 1=CCW
    /// @param error 伺服的硬件状态字
    /// @return 返回 txRxPacket() 函数执行的结果错误码
    CommErrorCode homing(PortHandler *port, uint8_t id, uint8_t direction = 0, uint8_t *error = 0);

    /// @brief 这个函数负责读取指定地址的寄存器，这个函数会构造 InstRead 命令数据报, 然后使用 txRxPacket() 发送数据报接收响应
    /// @description 如果 ID 是 BoardcastID，那么会直接退出
    /// @param port 发送数据报使用的串口
    /// @param id 伺服的通讯地址
    /// @param address 要读取的寄存器开始地址
    /// @param length 要读取的字节数
    /// @return 地址为 BoardcastID，则返回 CommNotImplement；否则返回 txPacket() 函数执行的结果错误码
    CommErrorCode readTx(PortHandler *port, uint8_t id, uint16_t address, uint16_t length);

    /// @brief 这个函数负责从状态数据报中读取寄存器数据信息，通常前面有发送一个 InstRead 指令数据报。
    /// @param port 发送数据报使用的串口
    /// @param length 要读取的字节数
    /// @param data 从数据报中读取到的数据
    /// @param error 伺服的硬件状态字
    /// @return 返回 rxPacket() 函数执行的结果错误码
    CommErrorCode readRx(PortHandler *port, uint16_t length, uint8_t *data, uint8_t *error = 0);

    /// @brief 这个函数发送一个 InstRead 指令数据报，然后使用 txRxPacket() 发送数据报接收响应，从中读取寄存器的数据
    /// @description 如果 ID 是 BoardcastID，那么会直接退出
    /// @param port 发送数据报使用的串口
    /// @param id 伺服的通讯地址
    /// @param address 要读取的寄存器开始地址
    /// @param length 要读取的字节数
    /// @param data 从数据报中读取到的数据
    /// @param error 伺服的硬件状态字
    /// @return 地址为 BoardcastID，则返回 CommNotImplement；否则返回 txPacket() 函数执行的结果错误码
    CommErrorCode readTxRx(PortHandler *port, uint8_t id, uint16_t address,
                           uint16_t *length, uint8_t *data, uint8_t *error = 0);

    /// @brief 这个函数发送一个 InstWrite 命令数据报来写入寄存器，它使用 txPacket() 发送数据报.
    /// @param port 发送数据报使用的串口
    /// @param id 伺服的通讯地址
    /// @param address 要写入的寄存器开始地址
    /// @param length 要写入的字节数
    /// @param data 要写入到寄存器中的数据
    /// @return 返回 txPacket() 函数执行的结果错误码
    CommErrorCode writeTxOnly(PortHandler *port, uint8_t id, uint16_t address, uint16_t length, uint8_t *data);

    /// @brief 这个函数发送一个 InstWrite 指令数据报，并接收响应数据报，从中读取硬件错误字
    /// @param port 发送数据报使用的串口
    /// @param id 伺服的通讯地址
    /// @param address 要写入的寄存器开始地址
    /// @param length 要写入的字节数
    /// @param data 要写入到寄存器中的数据
    /// @param error 伺服的硬件状态字
    /// @return 返回 txRxPacket() 函数执行的结果错误码
    CommErrorCode writeTxRx(PortHandler *port, uint8_t id, uint16_t address, uint16_t length, uint8_t *data, uint8_t *error = 0);

    /// @brief 这个函数发送 InstRegWrite 指令数据报，这个命令会先让数据缓存到伺服中，等到 InstAction 指令时才更新对应寄存器
    /// @param port 发送数据报使用的串口
    /// @param id 伺服的通讯地址
    /// @param address 要写入的寄存器开始地址
    /// @param length 要读取的字节数
    /// @param data 要写入到寄存器中的数据
    /// @return 返回 txPacket() 函数执行的结果错误码
    CommErrorCode regWriteTxOnly(PortHandler *port, uint8_t id, uint16_t address, uint16_t length, uint8_t *data);

    /// @brief 这个函数发送 InstRegWrite 指令数据报，这个命令会先让数据缓存到伺服中，等到 InstAction 指令时才更新对应寄存器，并从状态数据报中读取硬件错误字
    /// @param port 发送数据报使用的串口
    /// @param id 伺服的通讯地址
    /// @param address 要写入的寄存器开始地址
    /// @param length 要读取的字节数
    /// @param data 要写入到寄存器中的数据
    /// @param error 伺服的硬件状态字
    /// @return 返回 txRxPacket() 函数执行的结果错误码
    CommErrorCode regWriteTxRx(PortHandler *port, uint8_t id, uint16_t address, uint16_t length, uint8_t *data, uint8_t *error = 0);


    /// @brief （只有版本 2.0 支持）这个函数发送 InstSyncRead 指令数据报，调用 txPacket() 发送数据报，被 GroupSyncRead 使用
    /// @param port 发送数据报使用的串口
    /// @param start_address 同步读取的寄存器起始地址
    /// @param data_length 同步读取的数据字节数
    /// @param param 同步读取的 ID 列表，因为同步读取可读取多个伺服，这里列出要读取的伺服通讯 ID 列表
    /// @param param_length 伺服通讯 ID 列表的长度
    /// @return 返回 txPacket() 函数执行的结果错误码
    CommErrorCode syncReadTx(PortHandler *port, uint16_t start_address, uint16_t data_length, uint8_t *param, uint16_t param_length);

    /// @brief 这个函数发送 InstSyncWrite 指令数据报，调用 txRxPacket() 发送指令，但并没有处理响应数据报，被 GroupSyncWrite 使用
    /// @param port 发送数据报使用的串口
    /// @param start_address 同步读取的寄存器起始地址
    /// @param data_length 同步读取的数据字节数
    /// @param param 同步读取的 ID 列表，因为同步读取可读取多个伺服，这里列出要读取的伺服通讯 ID 列表，后面是写入的数据
    /// @param param_length 伺服通讯 ID 列表的长度*（1+数据字节数）
    /// @return 返回 txRxPacket() 函数执行的结果错误码
    CommErrorCode syncWriteTxOnly(PortHandler *port, uint16_t start_address, uint16_t data_length, uint8_t *param, size_t param_length);

    /// @brief 这个函数发送 InstBulkRead 指令数据报，使用 txPacket() 发送，被 GroupBulkRead 使用
    /// @param port 发送数据报使用的串口
    /// @param param 块读取也可以一次读取多个伺服，每个伺服除 ID 外，还需要指定起始地址（2字节），数据长度（2字节）
    /// @param param_length 读取命令的参数长度，伺服数*5字节，
    /// @return 返回 txPacket() 函数执行的结果错误码
    CommErrorCode bulkReadTx(PortHandler *port, uint8_t *param, size_t param_length);

    /// @brief 这个函数发送 InstBulkWrite 指令数据报，使用txRxPacket() 发送，被 GroupBulkWrite 使用
    /// @param port 发送数据报使用的串口
    /// @param param 命令的参数字节，长度参考下面的参数
    /// @param param_length 读取命令的参数长度，伺服数*（5字节+数据长度），
    /// @return 返回 txRxPacket() 函数执行的结果错误码
    CommErrorCode bulkWriteTxOnly(PortHandler *port, uint8_t *param, uint16_t param_length);

protected:
    // 计算数据报的 CRC 校验码
    uint16_t updateCRC(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size);
    void addStuffing(uint8_t *packet);     // 添加填充字节，以避免在数据报内部出现 0xff 0xff 0xfd
    void removeStuffing(uint8_t *packet);  // 移除填充字节

    static const int PingStatusLength = 14;           // 响应 PING 指令的状态数据报

    // 指令数据报公共部分的长度，10 字节：HEADER0 HEADER1 HEADER2 RESERVED ID LEN_L LEN_H INST （...）CRC16_L CRC16_H
    static const int CmdCommonLength  =10;

    // PING 执行指令数据报公共部分的长度，3 字节：INST CRC16_L CRC16_H
    static const int PingCmdParamLen  =3;
    static const int PingStatusLen  =3;

    // 延时写执行指令数据报公共部分的长度，3 字节：INST CRC16_L CRC16_H
    static const int ActionCmdParamLen  =3;

    // 重启指令数据报公共部分的长度，3 字节：INST CRC16_L CRC16_H
    static const int RebootCmdParamLen  =3;

    // 回复出厂设定指令数据报公共部分的长度，4 字节：INST DIRECTION CRC16_L CRC16_H
    static const int FactoryResetCmdParamLen  =4;

    // 归零指令数据报公共部分的长度，4 字节：INST DIRECTION CRC16_L CRC16_H
    static const int HomeCmdParamLen  =4;

    // 读指令数据报的长度，14 字节：HEADER0 HEADER1 HEADER2 RESERVED ID LEN_L LEN_H INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H
    static const int ReadCmdLen  =14;
    // 读指令数据报公共部分的长度，7 字节：INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H
    static const int ReadCmdCommonParamLen  =7;

    // 写指令数据报公共部分的长度，12 字节：HEADER0 HEADER1 HEADER2 RESERVED ID LEN_L LEN_H INST START_ADDR_L START_ADDR_H （...） CRC16_L CRC16_H
    static const int WriteCmdCommonLength  =12;
    // 写指令数据报公共部分的长度，5 字节：INST START_ADDR_L START_ADDR_H CRC16_L CRC16_H
    static const int WriteCmdCommonParamLen  =5;

    // 延时写指令数据报公共部分的长度，12 字节：HEADER0 HEADER1 HEADER2 RESERVED ID LEN_L LEN_H INST START_ADDR_L START_ADDR_H （...） CRC16_L CRC16_H
    static const int RegWriteCmdCommonLength  =12;
    // 延时写指令数据报公共部分的长度，5 字节：INST START_ADDR_L START_ADDR_H CRC16_L CRC16_H
    static const int RegWriteCmdCommonParamLen  =5;

    // 同步读写指令数据报公共部分的长度，14 字节：HEADER0 HEADER1 HEADER2 RESERVED ID LEN_L LEN_H INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H （...） CRC16_L CRC16_H
    static const int SyncCmdCommonLength  =14;
    // 同步读写指令数据报公共部分的长度，7 字节：INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H
    static const int SyncCmdCommonParamLen  =7;

    // 块读写指令数据报公共部分的长度，10 字节：HEADER0 HEADER1 HEADER2 RESERVED ID LEN_L LEN_H INST （...）CRC16_L CRC16_H
    static const int BulkCmdCommonLength  =10;
    // 块读写指令数据报公共部分的长度，3 字节：INST CRC16_L CRC16_H
    static const int BulkCmdCommonParamLen  =3;

    static const int _MaxBufSize =4096;
    uint8_t _txpacket[_MaxBufSize];
    uint8_t _rxpacket[_MaxBufSize];
    uint32_t _packetSN =0;
 };

#define PING_STATUS_LENGTH   14
#define READ_COMMAND_LENGTH  14
#define PING_COMMAND_LENGTH   10
}

#endif /* TOUCHIDEAS_SDK_PROTOCOL2PACKETHANDLER_H_ */
