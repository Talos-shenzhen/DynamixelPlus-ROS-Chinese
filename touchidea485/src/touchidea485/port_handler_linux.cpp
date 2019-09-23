/*******************************************************************************
* Copyright (c) 2016, ROBOTIS CO., LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of ROBOTIS nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/* Author: zerom, Ryu Woon Jung (Leon) */

#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <linux/serial.h>

#include "../../include/touchidea485_linux/port_handler_linux.h"

#define BOOST_LOG_DYN_LINK 1
#include <boost/log/trivial.hpp>

using namespace TouchIdeas485;

PortHandlerLinux::PortHandlerLinux(const char *pName)
  : socket_fd_(-1) {
    portName(pName);
}

bool PortHandlerLinux::openPort() {
    return baudRate(baudrate_);
}

void PortHandlerLinux::closePort() {
    if(socket_fd_ != -1) {
	if (ioctl(socket_fd_, TIOCNXCL ))  // 为了避免其他程序误操作我们的串口，释放对串口的独占
	    BOOST_LOG_TRIVIAL(error) << "[PortHandlerLinux::setupPort]Unlock port failed!" <<port_name_;
	close(socket_fd_);
    }
    socket_fd_ = -1;
    _isOpen =false;
}

void PortHandlerLinux::clearPort() {
    tcflush(socket_fd_, TCIOFLUSH);
}

// TODO: baud number ??
bool PortHandlerLinux::baudRate(const int baudrate) {    
    int baud = getCFlagBaud(baudrate);
    //BOOST_LOG_TRIVIAL(info) << "[PortHandlerLinux::baudRate]"<<baudrate<<","<<baud;
    closePort();

    baudrate_ = baudrate;
    if(baud <= 0) { // custom baudrate
	setupPort(B38400);
	_isOpen = setCustomBaudrate(baudrate);
    } else {
	_isOpen = setupPort(baud);
    }
    return _isOpen;
}

int PortHandlerLinux::baudRate() {
    return baudrate_;
}

int PortHandlerLinux::bytesAvailable() {
    int bytes_available;
    ioctl(socket_fd_, FIONREAD, &bytes_available);
    return bytes_available;
}

int PortHandlerLinux::readPort(uint8_t *packet, int length) {
    return read(socket_fd_, packet, length);
}

int PortHandlerLinux::writePort(uint8_t *packet, int length) {
    return write(socket_fd_, packet, length);
}

double PortHandlerLinux::getCurrentTime() {
    struct timespec tv;
    clock_gettime( CLOCK_REALTIME, &tv);
    return ((double)tv.tv_sec*1000.0 + (double)tv.tv_nsec*0.001*0.001);
}

// 从系统中读取串口的实际状态
bool PortHandlerLinux::readPortAttr(void) {
    struct termios term; //term用于存储获得的终端参数信息
    int err;
    if(socket_fd_ < 0) {
	BOOST_LOG_TRIVIAL(error) << "[PortHandlerLinux::readPortAttr] port is Closed!";
	_isOpen = false;
	return false;
    }

    //获得标准输入的终端参数，将获得的信息保存在term变量中
    if(tcgetattr(socket_fd_,&term)==-1){
	BOOST_LOG_TRIVIAL(error) << "[PortHandlerLinux::readPortAttr]Cannot get port description!";
	return false;
    }

    // 读取串口相关参数
    int baud =cfgetospeed(&term);
    int reqBaud = getCFlagBaud(baudrate_);
    if ( baud != reqBaud )  {
	BOOST_LOG_TRIVIAL(info) << "[PortHandlerLinux::readPortAttr]"<< cfgetispeed(&term) <<"," << cfgetospeed(&term);
	baudRate(baudrate_);
    }
    return true;
}
bool PortHandlerLinux::setupPort(int cflag_baud) {
    struct termios newtio;

    socket_fd_ = open(port_name_, O_RDWR|O_NOCTTY|O_NONBLOCK);
    if(socket_fd_ < 0) {
	BOOST_LOG_TRIVIAL(error) << "[PortHandlerLinux::setupPort]Open port failed!" <<errno;
	return false;
    }
//    if (ioctl(socket_fd_, TIOCEXCL)) {  // 为了避免其他程序误操作我们的串口，我们要求独占串口
//	BOOST_LOG_TRIVIAL(error) << "[PortHandlerLinux::setupPort]Lock port failed!" <<errno;
//    }

    bzero(&newtio, sizeof(newtio)); // clear struct for new port settings

    newtio.c_cflag = cflag_baud | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag      = 0;
    newtio.c_lflag      = 0;
    newtio.c_cc[VTIME]  = 0;
    newtio.c_cc[VMIN]   = 0;

    // clean the buffer and activate the settings for the port
    tcflush(socket_fd_, TCIFLUSH);
    tcsetattr(socket_fd_, TCSANOW, &newtio);

    tx_time_per_byte = (1000.0 / (double)baudrate_) * 12.0;
    return true;
}

bool PortHandlerLinux::setCustomBaudrate(int speed) {
    if ( speed == baudrate_ )
        return true;
    if( socket_fd_ == -1 )
        return false;
  // try to set a custom divisor
  struct serial_struct ss;
  if(ioctl(socket_fd_, TIOCGSERIAL, &ss) != 0) {
    printf("[PortHandlerLinux::SetCustomBaudrate] TIOCGSERIAL failed!\n");
    return false;
  }

  ss.flags = (ss.flags & ~ASYNC_SPD_MASK) | ASYNC_SPD_CUST;
  ss.custom_divisor = (ss.baud_base + (speed / 2)) / speed;
  int closest_speed = ss.baud_base / ss.custom_divisor;

  if(closest_speed < speed * 98 / 100 || closest_speed > speed * 102 / 100) {
    printf("[PortHandlerLinux::SetCustomBaudrate] Cannot set speed to %d, closest is %d \n", speed, closest_speed);
    return false;
  }

  if(ioctl(socket_fd_, TIOCSSERIAL, &ss) < 0) {
    printf("[PortHandlerLinux::SetCustomBaudrate] TIOCSSERIAL failed!\n");
    return false;
  }

  tx_time_per_byte = (1000.0 / (double)speed) * 12.0;
  return true;
}

int PortHandlerLinux::getCFlagBaud(int baudrate) {
  switch(baudrate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 500000:
      return B500000;
    case 576000:
      return B576000;
    case 921600:
      return B921600;
    case 1000000:
      return B1000000;
    case 1152000:
      return B1152000;
    case 1500000:
      return B1500000;
    case 2000000:
      return B2000000;
    case 2500000:
      return B2500000;
    case 3000000:
      return B3000000;
    case 3500000:
      return B3500000;
    case 4000000:
      return B4000000;
    default:
      return -1;
  }
}
