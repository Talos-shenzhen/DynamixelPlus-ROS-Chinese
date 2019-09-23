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

#if defined(__linux__)
#include "../../include/touchidea485/protocol1_packet_handler.h"
#elif defined(__APPLE__)
#include "protocol1_packet_handler.h"
#elif defined(_WIN32) || defined(_WIN64)
#define WINDLLEXPORT
#include "../../include/touchidea485/protocol1_packet_handler.h"
#elif defined(ARDUINO) || defined(__OPENCR__) || defined(__OPENCM904__)
#include "../../include/touchidea485/protocol1_packet_handler.h"
#endif

#include <string.h>
#include <stdlib.h>

///////////////// for Protocol 1.0 Packet /////////////////
#define PKT_HEADER0             0
#define PKT_HEADER1             1
#define PKT_ID                  2
#define PKT_LENGTH              3
#define PKT_INSTRUCTION         4
#define PKT_ERROR               4
#define PKT_PARAMETER0          5

///////////////// Protocol 1.0 Error bit /////////////////
#define ERRBIT_VOLTAGE          1       // Supplied voltage is out of the range (operating volatage set in the control table)
#define ERRBIT_ANGLE            2       // Goal position is written out of the range (from CW angle limit to CCW angle limit)
#define ERRBIT_OVERHEAT         4       // Temperature is out of the range (operating temperature set in the control table)
#define ERRBIT_RANGE            8       // Command(setting value) is out of the range for use.
#define ERRBIT_CHECKSUM         16      // Instruction packet checksum is incorrect.
#define ERRBIT_OVERLOAD         32      // The current load cannot be controlled by the set torque.
#define ERRBIT_INSTRUCTION      64      // Undefined instruction or delivering the action command without the reg_write command.

using namespace TouchIdeas485;

Protocol1PacketHandler *Protocol1PacketHandler::unique_instance_ = new Protocol1PacketHandler();

Protocol1PacketHandler::Protocol1PacketHandler() { }

const char *Protocol1PacketHandler::txRxResult(CommErrorCode result) {
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

    default:
      return "";
  }
}

const char *Protocol1PacketHandler::rxPacketError(uint8_t error) {
  if (error & ERRBIT_VOLTAGE)
    return "[RxPacketError] Input voltage error!";

  if (error & ERRBIT_ANGLE)
    return "[RxPacketError] Angle limit error!";

  if (error & ERRBIT_OVERHEAT)
    return "[RxPacketError] Overheat error!";

  if (error & ERRBIT_RANGE)
    return "[RxPacketError] Out of range error!";

  if (error & ERRBIT_CHECKSUM)
    return "[RxPacketError] Checksum error!";

  if (error & ERRBIT_OVERLOAD)
    return "[RxPacketError] Overload error!";

  if (error & ERRBIT_INSTRUCTION)
    return "[RxPacketError] Instruction code error!";

  return "";
}

CommErrorCode Protocol1PacketHandler::txPacket(PortHandler *port) {
  uint8_t checksum               = 0;
  uint8_t total_packet_length    = _txpacket[PKT_LENGTH] + 4; // 4: HEADER0 HEADER1 ID LENGTH
  uint8_t written_packet_length  = 0;

  if ( port->isUsing() )
    return CommPortBusy;
  port->setUsing(true);

  // check max packet length
  if (total_packet_length > _MaxBufSize) {
    port->setUsing(false);
    return CommTxError;
  }

  // make packet header
  _txpacket[PKT_HEADER0]   = 0xFF;
  _txpacket[PKT_HEADER1]   = 0xFF;

  // add a checksum to the packet
  for (int idx = 2; idx < total_packet_length - 1; idx++)   // except header, checksum
    checksum += _txpacket[idx];
  _txpacket[total_packet_length - 1] = ~checksum;

  // tx packet
  port->clearPort();
  written_packet_length = port->writePort(_txpacket, total_packet_length);
  if (total_packet_length != written_packet_length) {
    port->setUsing(false);
    return CommTxFailed;
  }

  return CommSuccess;
}

CommErrorCode Protocol1PacketHandler::rxPacket(PortHandler *port) {
  CommErrorCode     result         = CommTxFailed;

  uint8_t checksum       = 0;
  uint8_t rx_length      = 0;
  uint8_t wait_length    = 6;    // minimum length (HEADER0 HEADER1 ID LENGTH ERROR CHKSUM)

  while(true) {
    rx_length += port->readPort(&_rxpacket[rx_length], wait_length - rx_length);
    if (rx_length >= wait_length) {
      uint8_t idx = 0;

      // find packet header
      for (idx = 0; idx < (rx_length - 1); idx++) {
	if (_rxpacket[idx] == 0xFF && _rxpacket[idx+1] == 0xFF)
          break;
      }

      if (idx == 0)  { // found at the beginning of the packet
	if (_rxpacket[PKT_ID] > 0xFD ||                  // unavailable ID
	   _rxpacket[PKT_LENGTH] > _MaxBufSize ||   // unavailable Length
	   _rxpacket[PKT_ERROR] >= 0x64)  {               // unavailable Error
            // remove the first byte in the packet
            for (uint8_t s = 0; s < rx_length - 1; s++)
		_rxpacket[s] = _rxpacket[1 + s];
            //memcpy(&rxpacket[0], &rxpacket[idx], rx_length - idx);
            rx_length -= 1;
            continue;
        }

        // re-calculate the exact length of the rx packet
	if (wait_length != _rxpacket[PKT_LENGTH] + PKT_LENGTH + 1) {
	  wait_length = _rxpacket[PKT_LENGTH] + PKT_LENGTH + 1;
          continue;
        }

        if (rx_length < wait_length) {
          // check timeout
          if (port->isPacketTimeout() == true) {
            if (rx_length == 0) {
              result = CommRxTimeout;
            } else {
              result = CommRxCorrupt;
            }
            break;
          } else {
            continue;
          }
        }

        // calculate checksum
        for (int i = 2; i < wait_length - 1; i++)   // except header, checksum
	  checksum += _rxpacket[i];
        checksum = ~checksum;

        // verify checksum
	if (_rxpacket[wait_length - 1] == checksum) {
          result = CommSuccess;
        } else {
          result = CommRxCorrupt;
        }
        break;
      } else {
        // remove unnecessary packets
        for (uint8_t s = 0; s < rx_length - idx; s++)
	  _rxpacket[s] = _rxpacket[idx + s];
        //memcpy(&rxpacket[0], &rxpacket[idx], rx_length - idx);
        rx_length -= idx;
      }
    } else {
      // check timeout
      if (port->isPacketTimeout() == true) {
        if (rx_length == 0) {
          result = CommRxTimeout;
        } else {
          result = CommRxCorrupt;
        }
        break;
      }
    }
  }
  port->setUsing(false);

  return result;
}

// NOT for BulkRead instruction
CommErrorCode Protocol1PacketHandler::txRxPacket(PortHandler *port, uint8_t *error) {
    // tx packet
    CommErrorCode result = txPacket(port);
    if (result != CommSuccess)
      return result;

    // (ID == Broadcast ID && NOT BulkRead) == no need to wait for status packet
    // (Instruction == action) == no need to wait for status packet
    if ((_txpacket[PKT_ID] == BoardcastID && _txpacket[PKT_INSTRUCTION] != InstBulkRead) ||
       (_txpacket[PKT_INSTRUCTION] == InstAction)) {
        port->setUsing(false);
        return result;
    }

    // set packet timeout
    if (_txpacket[PKT_INSTRUCTION] == InstRead) {
      port->packetTimeout((uint16_t)(_txpacket[PKT_PARAMETER0+1] + 6));
    } else {
      port->packetTimeout((uint16_t)6);
    }

    // rx packet
    result = rxPacket(port);
    // check txpacket ID == rxpacket ID
    if (_txpacket[PKT_ID] != _rxpacket[PKT_ID])
      result = rxPacket(port);

    if (result == CommSuccess && _txpacket[PKT_ID] != BoardcastID) {
      if (error != 0)
	*error = (uint8_t)_rxpacket[PKT_ERROR];
    }
    return result;
}

CommErrorCode Protocol1PacketHandler::ping(PortHandler *port, uint8_t id, uint8_t *error) {
    return ping(port, id, 0, error);
}

CommErrorCode Protocol1PacketHandler::ping(PortHandler *port, uint8_t id, uint16_t *model_number, uint8_t *error) {
    if (id >= BoardcastID)
        return CommNotImplement;

    _txpacket[PKT_ID]            = id;
    _txpacket[PKT_LENGTH]        = 2;
    _txpacket[PKT_INSTRUCTION]   = InstPing;

    CommErrorCode result = txRxPacket(port, error);
    if (result == CommSuccess && model_number != 0) {
        uint8_t data_read[2] = {0};
	uint16_t len =2;
	result = readTxRx(port, id, 0, &len, data_read);  // Address 0 : Model Number
        if (result == CommSuccess) *model_number = makeWord(data_read[0], data_read[1]);
    }

    return result;
}

CommErrorCode Protocol1PacketHandler::broadcastPing(PortHandler *port, std::vector<uint8_t> &id_list) {
    (void)port;
    (void)id_list;
    return CommNotImplement;
}

CommErrorCode Protocol1PacketHandler::action(PortHandler *port, uint8_t id) {
    _txpacket[PKT_ID] = id;
    _txpacket[PKT_LENGTH] = 2;
    _txpacket[PKT_INSTRUCTION] = InstAction;

    return txRxPacket(port, 0);
}

CommErrorCode Protocol1PacketHandler::reboot(PortHandler *port, uint8_t id, uint8_t *error) {
    (void)port;
    (void)id;
    (void)error;
    return CommNotImplement;
}

CommErrorCode Protocol1PacketHandler::factoryReset(PortHandler *port, uint8_t id, uint8_t option, uint8_t *error) {
    (void)option;

    _txpacket[PKT_ID] = id;
    _txpacket[PKT_LENGTH] = 2;
    _txpacket[PKT_INSTRUCTION] = InstFactoryReset;

    return txRxPacket(port, error);
}

CommErrorCode Protocol1PacketHandler::homing(PortHandler *port, uint8_t id, uint8_t direction, uint8_t *error) {
    (void)port;
    (void)id;
    (void)direction;
    (void)error;
    return CommNotImplement;
}

CommErrorCode Protocol1PacketHandler::readTx(PortHandler *port, uint8_t id, uint16_t address, uint16_t length) {
    if (id >= BoardcastID)
      return CommNotImplement;

    _txpacket[PKT_ID] = id;
    _txpacket[PKT_LENGTH] = 4;
    _txpacket[PKT_INSTRUCTION] = InstRead;
    _txpacket[PKT_PARAMETER0+0] = (uint8_t)address;
    _txpacket[PKT_PARAMETER0+1] = (uint8_t)length;

    CommErrorCode result = txPacket(port);

    if (result == CommSuccess) // set packet timeout
      port->packetTimeout((uint16_t)(length+6));

    return result;
}

CommErrorCode Protocol1PacketHandler::readRx(PortHandler *port, uint16_t length, uint8_t *data, uint8_t *error) {
    CommErrorCode result = rxPacket(port);
    if (result == CommSuccess) {
        if (error != 0) {
	    *error = (uint8_t)_rxpacket[PKT_ERROR];
        }
        for (uint8_t s = 0; s < length; s++) {
	    data[s] = _rxpacket[PKT_PARAMETER0 + s];
        }
    }
    return result;
}

CommErrorCode Protocol1PacketHandler::readTxRx(PortHandler *port, uint8_t id, uint16_t address,
                                               uint16_t *length, uint8_t *data, uint8_t *error) {
    if (id >= BoardcastID)
        return CommNotImplement;

    _txpacket[PKT_ID] = id;
    _txpacket[PKT_LENGTH] = 4;
    _txpacket[PKT_INSTRUCTION] = InstRead;
    _txpacket[PKT_PARAMETER0+0] = (uint8_t)address;
    _txpacket[PKT_PARAMETER0+1] = (uint8_t)(*length);

    CommErrorCode result = txRxPacket(port, error);
    if (result == CommSuccess) {
        if (error != 0) {
	    *error = (uint8_t)_rxpacket[PKT_ERROR];
        }
	for (uint8_t s = 0; s < *length; s++) {
	    data[s] = _rxpacket[PKT_PARAMETER0 + s];
        }
    }
    return result;
}

CommErrorCode Protocol1PacketHandler::writeTxOnly(PortHandler *port, uint8_t id, uint16_t address, uint16_t length, uint8_t *data) {
    _txpacket[PKT_ID] = id;
    _txpacket[PKT_LENGTH] = length+3;
    _txpacket[PKT_INSTRUCTION] = InstWrite;
    _txpacket[PKT_PARAMETER0] = (uint8_t)address;

    for (uint8_t s = 0; s < length; s++)
	_txpacket[PKT_PARAMETER0+1+s] = data[s];

    CommErrorCode result = txPacket(port);
    port->setUsing(false);
    return result;
}

CommErrorCode Protocol1PacketHandler::writeTxRx(PortHandler *port, uint8_t id, uint16_t address, uint16_t length, uint8_t *data, uint8_t *error) {
    _txpacket[PKT_ID] = id;
    _txpacket[PKT_LENGTH] = length+3;
    _txpacket[PKT_INSTRUCTION] = InstWrite;
    _txpacket[PKT_PARAMETER0] = (uint8_t)address;

    for (uint8_t s = 0; s < length; s++)
	_txpacket[PKT_PARAMETER0+1+s] = data[s];

    CommErrorCode result = txRxPacket(port, error);
    return result;
}

CommErrorCode Protocol1PacketHandler::regWriteTxOnly(PortHandler *port, uint8_t id, uint16_t address, uint16_t length, uint8_t *data) {
    _txpacket[PKT_ID] = id;
    _txpacket[PKT_LENGTH] = length+3;
    _txpacket[PKT_INSTRUCTION] = InstRegWrite;
    _txpacket[PKT_PARAMETER0] = (uint8_t)address;

    for (uint8_t s = 0; s < length; s++)
	_txpacket[PKT_PARAMETER0+1+s] = data[s];

    CommErrorCode result = txPacket(port);
    port->setUsing(false);
    return result;
}

CommErrorCode Protocol1PacketHandler::regWriteTxRx(PortHandler *port, uint8_t id, uint16_t address, uint16_t length, uint8_t *data, uint8_t *error) {
    _txpacket[PKT_ID]            = id;
    _txpacket[PKT_LENGTH]        = length+3;
    _txpacket[PKT_INSTRUCTION]   = InstRegWrite;
    _txpacket[PKT_PARAMETER0]    = (uint8_t)address;

    for (uint8_t s = 0; s < length; s++)
	_txpacket[PKT_PARAMETER0+1+s] = data[s];

    CommErrorCode result = txRxPacket(port, error);
    return result;
}

CommErrorCode Protocol1PacketHandler::syncReadTx(PortHandler *port, uint16_t start_address, uint16_t data_length, uint8_t *param, uint16_t param_length) {
    (void)port;
    (void)start_address;
    (void)data_length;
    (void)param;
    (void)param_length;
    return CommNotImplement;
}

CommErrorCode Protocol1PacketHandler::syncWriteTxOnly(PortHandler *port, uint16_t start_address, uint16_t data_length, uint8_t *param, size_t param_length) {
    // 8: HEADER0 HEADER1 ID LEN INST START_ADDR DATA_LEN ... CHKSUM
    _txpacket[PKT_ID]            = BoardcastID;
    _txpacket[PKT_LENGTH]        = (uint8_t)param_length + 4; // 4: INST START_ADDR DATA_LEN ... CHKSUM
    _txpacket[PKT_INSTRUCTION]   = InstSyncWrite;
    _txpacket[PKT_PARAMETER0+0]  = (uint8_t)start_address;
    _txpacket[PKT_PARAMETER0+1]  = (uint8_t)data_length;

    for (uint8_t s = 0; s < param_length; s++)
	_txpacket[PKT_PARAMETER0+2+s] = param[s];

    CommErrorCode result = txRxPacket(port, 0);
    return result;
}

CommErrorCode Protocol1PacketHandler::bulkReadTx(PortHandler *port, uint8_t *param, size_t param_length) {
    // 7: HEADER0 HEADER1 ID LEN INST 0x00 ... CHKSUM
    _txpacket[PKT_ID]            = BoardcastID;
    _txpacket[PKT_LENGTH]        = (uint8_t)param_length + 3; // 3: INST 0x00 ... CHKSUM
    _txpacket[PKT_INSTRUCTION]   = InstBulkRead;
    _txpacket[PKT_PARAMETER0+0]  = 0x00;

    for (uint8_t s = 0; s < param_length; s++)
	_txpacket[PKT_PARAMETER0+1+s] = param[s];

    CommErrorCode result = txPacket(port);
    if (result == CommSuccess) {
        int wait_length = 0;
        for (int i = 0; i < param_length; i += 3)
            wait_length += param[i] + 7;
        port->packetTimeout((uint16_t)wait_length);
    }
    return result;
}

CommErrorCode Protocol1PacketHandler::bulkWriteTxOnly(PortHandler *port, uint8_t *param, uint16_t param_length) {
    (void)port;
    (void)param;
    (void)param_length;
    return CommNotImplement;
}
