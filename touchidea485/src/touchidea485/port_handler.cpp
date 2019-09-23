/* Authors: Zhwei123 */

#if defined(_WIN32) || defined(_WIN64)
#define WINDLLEXPORT
#endif

#include "../../include/touchidea485/port_handler.h"

#ifdef __linux__
  #include "../../include/touchidea485_linux/port_handler_linux.h"
#endif

#if defined(_WIN32) || defined(_WIN64)
  #include "../../include/touchidea485_windows/port_handler_windows.h"
#endif

using namespace TouchIdeas485;

//根据字节长度计算传输时间
double PortHandler::len2Time(uint16_t packet_length) {
    return (tx_time_per_byte * static_cast<double>(packet_length+_LatencyTimer));
}

double PortHandler::packetTimeout(uint16_t packet_length) {
    packet_start_time_  = getCurrentTime();
    packet_timeout_     = len2Time(packet_length) +0.5;
    return packet_timeout_;
}

void PortHandler::packetTimeout(double msec) {
    packet_start_time_  = getCurrentTime();
    packet_timeout_     = msec;
}

bool PortHandler::isPacketTimeout() {
    if(getTimeSinceStart() > packet_timeout_) {
	packet_timeout_ = 0;
	return true;
    }
    return false;
}

double PortHandler::getTimeSinceStart() {
    double time = getCurrentTime() - packet_start_time_;
    if(time < 0.0)
	packet_start_time_ = getCurrentTime();

    return time;
}

// 创建一个串口句柄，我们通过它与 RS485 进行通讯
PortHandler *PortHandler::portHandler(const char *portName) {
#ifdef __linux__
    return (PortHandler *)(new PortHandlerLinux(portName));
#endif

#if defined(_WIN32) || defined(_WIN64)
    return (PortHandler *)(new PortHandlerWindows(portName));
#endif
}
