/* Authors: Zhwei123 */

#if defined(_WIN32) || defined(_WIN64)
#define WINDLLEXPORT
#endif

#define BOOST_LOG_DYN_LINK 1
#include <boost/log/trivial.hpp>

#include <algorithm>
#include "../../include/touchidea485/group_sync_write.h"

using namespace TouchIdeas485;

GroupSyncWrite::GroupSyncWrite(PortHandler *port, PacketHandler *ph, uint16_t start_address, uint16_t data_length)
  : port_(port), ph_(ph), start_address_(start_address),
    data_length_(data_length), is_param_changed_(false), param_(nullptr) {
  clearParam();
}

void GroupSyncWrite::makeParam(uint8_t sn) {
  if (id_list_.size() == 0) return;

  if (param_ != 0)
    delete[] param_;
  param_ = new uint8_t[id_list_.size() * (1 + data_length_)]; // ID(1) + DATA(data_length)

  int idx = 0;
  param_[idx++] = sn;
  for (unsigned int i = 0; i < id_list_.size(); i++) {
    uint8_t id = id_list_[i];
    if (data_list_[id] ==nullptr)
      return;

    param_[idx++] = id;
    for (int c = 0; c < data_length_; c++)
      param_[idx++] = (data_list_[id])[c];
  }
}

// 为一个伺服的同步写入指令设定数据，注意数据长度要与同步指令初始化长度匹配
bool GroupSyncWrite::addParam(uint8_t id, uint8_t *data) {
  if (std::find(id_list_.begin(), id_list_.end(), id) != id_list_.end())   // id already exist
    return false;

  id_list_.push_back(id);
  data_list_[id]    = new uint8_t[data_length_];
  for (int c = 0; c < data_length_; c++)
    data_list_[id][c] = data[c];

  is_param_changed_   = true;
  return true;
}

void GroupSyncWrite::removeParam(uint8_t id) {
  std::vector<uint8_t>::iterator it = std::find(id_list_.begin(), id_list_.end(), id);
  if (it == id_list_.end())    // NOT exist
    return;

  id_list_.erase(it);
  delete[] data_list_[id];
  data_list_.erase(id);

  is_param_changed_   = true;
}

bool GroupSyncWrite::changeParam(uint8_t id, uint8_t *data) {
  std::vector<uint8_t>::iterator it = std::find(id_list_.begin(), id_list_.end(), id);
  if (it == id_list_.end())    // NOT exist
    return false;

  delete[] data_list_[id];
  data_list_[id]    = new uint8_t[data_length_];
  for (int c = 0; c < data_length_; c++)
    data_list_[id][c] = data[c];

  is_param_changed_   = true;
  return true;
}

void GroupSyncWrite::clearParam() {
  if (id_list_.size() == 0)
    return;

  for (unsigned int i = 0; i < id_list_.size(); i++)
    delete[] data_list_[id_list_[i]];

  id_list_.clear();
  data_list_.clear();
  if (param_ !=nullptr)
    delete[] param_;
  param_ =nullptr;
}

CommErrorCode GroupSyncWrite::txPacket(uint8_t sn) {
  if (id_list_.size() == 0)
    return CommCmdEmpty;

  if (is_param_changed_ == true)
    makeParam(sn);

  return ph_->syncWriteTxOnly(port_, start_address_, data_length_, param_, id_list_.size() * (1 + data_length_)+1);
}
