/* Authors: Zhwei123 */

#if defined(_WIN32) || defined(_WIN64)
#define WINDLLEXPORT
#endif

#include <stdio.h>
#include <algorithm>
#include "../../include/touchidea485/group_bulk_read.h"

using namespace TouchIdeas485;

GroupBulkRead::GroupBulkRead(PortHandler *port, PacketHandler *ph)
  : port_(port), ph_(ph), last_result_(false),
    is_param_changed_(false), param_(0) {
  clearParam();
}

void GroupBulkRead::makeParam() {
  if (id_list_.size() == 0)
    return;

  if (param_ != 0)
    delete[] param_;
  param_ = 0;

  if (ph_->protocolVersion() == 1.0) { // 1.0 版本一个节点占用 3 字节描述
    param_ = new uint8_t[id_list_.size() * 3];  // ID(1) + ADDR(1) + LENGTH(1)
  } else {   // 2.0 版本一个节点占用 5 字节描述
    param_ = new uint8_t[id_list_.size() * 5];  // ID(1) + ADDR(2) + LENGTH(2)
  }

  int idx = 0;
  for (unsigned int i = 0; i < id_list_.size(); i++) {
    uint8_t id = id_list_[i];
    if (ph_->protocolVersion() == 1.0) {
      param_[idx++] = (uint8_t)length_list_[id];    // LEN
      param_[idx++] = id;                           // ID
      param_[idx++] = (uint8_t)address_list_[id];   // ADDR
    } else {    // 2.0
      param_[idx++] = id;                               // ID
      param_[idx++] = lowByte(address_list_[id]);    // ADDR_L
      param_[idx++] = highByte(address_list_[id]);    // ADDR_H
      param_[idx++] = lowByte(length_list_[id]);     // LEN_L
      param_[idx++] = highByte(length_list_[id]);     // LEN_H
    }
  }
}

bool GroupBulkRead::addParam(uint8_t id, uint16_t start_address, uint16_t data_length) {
  if (std::find(id_list_.begin(), id_list_.end(), id) != id_list_.end())   // 检查通讯 ID 是否已经存在
    return false;

  id_list_.push_back(id);
  length_list_[id]    = data_length;
  address_list_[id]   = start_address;
  data_list_[id]      = new uint8_t[data_length];

  is_param_changed_   = true;
  return true;
}

void GroupBulkRead::removeParam(uint8_t id) {
  std::vector<uint8_t>::iterator it = std::find(id_list_.begin(), id_list_.end(), id);
  if (it == id_list_.end())    // 这个通讯 ID 不在列表中存在
    return;

  id_list_.erase(it);
  address_list_.erase(id);
  length_list_.erase(id);
  delete[] data_list_[id];
  data_list_.erase(id);

  is_param_changed_   = true;
}

void GroupBulkRead::clearParam() {
  if (id_list_.size() == 0)
    return;

  for (unsigned int i = 0; i < id_list_.size(); i++)
    delete[] data_list_[id_list_[i]];

  id_list_.clear();
  address_list_.clear();
  length_list_.clear();
  data_list_.clear();
  if (param_ != 0)
    delete[] param_;
  param_ = 0;
}

CommErrorCode GroupBulkRead::txPacket() {
  if (id_list_.size() == 0)
    return CommCmdEmpty;

  if (is_param_changed_ == true)
    makeParam();

  if (ph_->protocolVersion() == 1.0) {
    return ph_->bulkReadTx(port_, param_, id_list_.size() * 3);
  } else  {   // 2.0
    return ph_->bulkReadTx(port_, param_, id_list_.size() * 5);
  }
}

CommErrorCode GroupBulkRead::rxPacket() {
  size_t cnt = id_list_.size();
  CommErrorCode result = CommRxFailed;

  last_result_ = false;

  if (cnt == 0)
    return CommCmdEmpty;

  for (size_t i = 0; i < cnt; i++) {
    uint8_t id = id_list_[i];

    result = ph_->readRx(port_, length_list_[id], data_list_[id]);
    if (result != CommSuccess)
      return result;
  }

  if (result == CommSuccess)
    last_result_ = true;

  return result;
}

CommErrorCode GroupBulkRead::txRxPacket() {
  CommErrorCode result = txPacket();
  if (result != CommSuccess)
    return result;

  return rxPacket();
}

bool GroupBulkRead::isAvailable(uint8_t id, uint16_t address, uint16_t data_length) {
  if (last_result_ == false || data_list_.find(id) == data_list_.end())
    return false;

  uint16_t start_addr = address_list_[id];
  if (address < start_addr || start_addr + length_list_[id] - data_length < address)
    return false;

  return true;
}

uint32_t GroupBulkRead::getData(uint8_t id, uint16_t address, uint16_t data_length) {
  if (isAvailable(id, address, data_length) == false)
    return 0;

  uint16_t start_addr = address_list_[id];

  switch(data_length) {
    case 1:
      return data_list_[id][address - start_addr];
    case 2:
      return makeWord(data_list_[id][address - start_addr], data_list_[id][address - start_addr + 1]);
    case 4:
      return makeDWord(makeWord(data_list_[id][address - start_addr + 0], data_list_[id][address - start_addr + 1]),
                           makeWord(data_list_[id][address - start_addr + 2], data_list_[id][address - start_addr + 3]));
    default:
      return 0;
  }
}
