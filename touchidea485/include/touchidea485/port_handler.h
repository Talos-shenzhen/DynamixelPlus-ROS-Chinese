/* Authors: Zhwei123 */

#ifndef TOUCHIDEAS_SDK_PORTHANDLER_H_
#define TOUCHIDEAS_SDK_PORTHANDLER_H_

#ifdef __linux__
#define WINDECLSPEC
#elif defined(_WIN32) || defined(_WIN64)
#ifdef WINDLLEXPORT
#define WINDECLSPEC __declspec(dllexport)
#else
//#define WINDECLSPEC __declspec(dllimport)
#define WINDECLSPEC
#endif
#endif

#include <stdint.h>
#include <string.h>

namespace TouchIdeas485 {

class WINDECLSPEC PortHandler {
public:
    static const int DefBaudRate_ = 1000000;    // 我们使用的默认波特率
    static const int _LatencyTimer = 8;  // msec (USB latency timer) [was changed from 4 due to the Ubuntu update 16.04.2]

    PortHandler(void): baudrate_(DefBaudRate_),
        packet_start_time_(0.0),
        packet_timeout_(0.0),
        tx_time_per_byte(0.0) {
    }

    // 我们实际使用时，都是通过这个静态函数来申请一个串口句柄
    static PortHandler *portHandler(const char *portName);

    virtual ~PortHandler() { }

    virtual bool openPort() = 0;   //打开串口
    virtual void closePort() = 0;  //关闭串口
    virtual void clearPort() = 0;  //清空串口缓存区

    virtual bool isOpen(void) const { return _isOpen; } // 判断串口是否打开
    virtual bool readPortAttr(void) { return false; }  // 从系统中读取串口的实际状态

    // 设定和获取串口设备名称
    virtual void portName(const char* portName) {
	strcpy(port_name_, portName);
    }
    virtual const char* portName() const {
	return port_name_;
    }

    // 设定和获取串口波特率
    virtual bool baudRate(const int baudrate) = 0;
    virtual int baudRate() = 0;

    // 获取串口接收缓存区可用字节
    virtual int bytesAvailable() = 0;

    // 读写串口接收和发送缓存区
    virtual int readPort(uint8_t *packet, int length) = 0;
    virtual int writePort(uint8_t *packet, int length) = 0;

    // 设定串口接收超时，一种是指定接收数据报的长度；另一种是指定毫秒时间
    virtual double packetTimeout(uint16_t packet_length);
    virtual void packetTimeout(double msec);
    virtual double len2Time(uint16_t packet_length); //根据字节长度计算传输时间

    virtual bool isPacketTimeout();   // 检查接收等待是否超时
    double  getTimeSinceStart();

    bool isUsing(void) const { return isUsing_; }
    void setUsing(bool used) { isUsing_ =used; }
protected:
    virtual double getCurrentTime()=0;

    bool   isUsing_ =false;
    bool   _isOpen =false;

    int     baudrate_;
    char    port_name_[30];

    double  packet_start_time_;
    double  packet_timeout_;
    double  tx_time_per_byte;
};

}


#endif /* TOUCHIDEAS_SDK_PORTHANDLER_H_ */
