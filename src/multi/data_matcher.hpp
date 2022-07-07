#pragma once

#include <cmath>
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "data_struct.hpp"

#include "cyber/common/log.h"
#include "cyber/cyber.h"

namespace omnisense::redetection {

template <typename T0, typename T1 = NullType, typename T2 = NullType,
          typename T3 = NullType>
using MultiSourceMatch_CallBack =
    std::function<void(std::vector<BasicMsgPtr<T0, T1, T2, T3>>&)>;

template <typename T0, typename T1 = NullType, typename T2 = NullType,
          typename T3 = NullType>
bool check_input(const Validation& config,
                 const BasicMsgPtr<T0, T1, T2, T3>& input_frame) {
  if (input_frame == nullptr) {
    AWARN << "Input is nullptr";
    return false;
  }
  if (!input_frame->is_frame_valid(config)) {
    AWARN << "Input is invalid, detail is " << input_frame->printf_frame();
    return false;
  }
  return true;
}

template <typename T0, typename T1 = NullType, typename T2 = NullType,
          typename T3 = NullType>
class MultiSourceMatcherBase {
 public:
  MultiSourceMatcherBase() = delete;
  MultiSourceMatcherBase(const Validation& config,
                         MultiSourceMatch_CallBack<T0, T1, T2, T3> func)
      : config_(config), thread_poll_(1), callback_func_(func) {
    process_thread_ = std::thread([this]() { this->process(); });
    input_buff_.resize(config_.source_num);
  }
  virtual ~MultiSourceMatcherBase() {
    thread_poll_ = 0;
    std::unique_lock<std::mutex> lock(deque_count_mtx_);
    deque_.push_back(nullptr);
    deque_count_cv_.notify_all();
    lock.unlock();
    if (process_thread_.joinable()) {
      process_thread_.join();
    }
  }
  void add_frame(const BasicMsgPtr<T0, T1, T2, T3>& input) {
    std::unique_lock<std::mutex> lock(this->deque_count_mtx_);
    ADEBUG << "[ADDR] {Packaged into deque }" << input.get();
    this->deque_.emplace_back(input);
    this->deque_count_cv_.notify_one();
    // lock.unlock();
  }

 protected:
  void process() {
    while (thread_poll_) {
      BasicMsgPtr<T0, T1, T2, T3> input_frame = update();
      if (input_frame == nullptr) {
        continue;
      }
      std::vector<BasicMsgPtr<T0, T1, T2, T3>> matched_list;
      // matched_list.reserve(config_.source_num);
      ADEBUG << "[ADDR] {Packaged try match}" << input_frame.get();
      process_impl(input_frame, matched_list);
      if (matched_list.size() == config_.source_num) {
        ADEBUG << "Find enough match message, now to trigger";
        callback_func_(matched_list);
      }
    }
  }

  BasicMsgPtr<T0, T1, T2, T3> update() {
    std::unique_lock<std::mutex> lock(this->deque_count_mtx_);
    while (deque_.empty()) {
      deque_count_cv_.wait(lock);
    }
    assert(!deque_.empty());
    BasicMsgPtr<T0, T1, T2, T3> latest = deque_.front();
    deque_.pop_front();
    ADEBUG << "[ADDR] {Packaged pop last   }" << latest.get();
    return latest;
  }

  virtual void process_impl(
      const BasicMsgPtr<T0, T1, T2, T3>& input_frame,
      std::vector<BasicMsgPtr<T0, T1, T2, T3>>& matched_list) {}

  std::string printf_input_buff() const {
    std::string input_buff_info_str = "input_buff_ details are:";
    for (size_t i = 0; i < input_buff_.size(); ++i) {
      const auto& vec = input_buff_[i];
      input_buff_info_str += "\n Template type = " + std::to_string(i + 1) +
                             ":" +
                             "Size = " + std::to_string(input_buff_[i].size());
      for (size_t j = 0; j < vec.size(); ++j) {
        if (vec[j] != nullptr) {
          input_buff_info_str +=
              "(" + std::to_string(vec[j]->frame_id(i)) + "), ";
        } else {
          input_buff_info_str += "nullptr, ";
        }
      }
    }
    return input_buff_info_str;
  }

  const Validation& config_;
  std::mutex deque_count_mtx_;
  std::condition_variable deque_count_cv_;
  volatile int thread_poll_;
  MultiSourceMatch_CallBack<T0, T1, T2, T3> callback_func_;
  std::thread process_thread_;
  std::deque<BasicMsgPtr<T0, T1, T2, T3>> deque_;
  std::vector<std::deque<BasicMsgPtr<T0, T1, T2, T3>>> input_buff_;
};

// find match
// find_matched_with_timestamp
template <typename T0, typename T1 = NullType, typename T2 = NullType,
          typename T3 = NullType>
void find_matched_with_idx(
    std::vector<BasicMsgPtr<T0, T1, T2, T3>>& matched_list,
    const std::vector<std::deque<BasicMsgPtr<T0, T1, T2, T3>>>& input_buff,
    unsigned long index, const Validation& config, FrameType curr) {
  for (size_t i = 0; i < input_buff.size(); ++i) {
    if (int(i) == curr) {
      continue;
    }
    for (size_t j = 0; j < input_buff[i].size(); ++j) {
      if (input_buff[i][j]->frame_id(i) == index) {
        matched_list.emplace_back(input_buff[i][j]);
        break;
      }
    }
  }
}

// erase_before timestamp
// erase_before idx
template <typename T0, typename T1 = NullType, typename T2 = NullType,
          typename T3 = NullType>
void erase_before(
    std::vector<std::deque<BasicMsgPtr<T0, T1, T2, T3>>>& input_buff,
    unsigned long index) {
  for (size_t i = 0; i < input_buff.size(); ++i) {
    while (input_buff[i].size() && input_buff[i][0]->frame_id(i) <= index) {
      input_buff[i].pop_front();
    }
  }
}

// insert_into_buff
template <typename T0, typename T1 = NullType, typename T2 = NullType,
          typename T3 = NullType>
void insert_into_buff(
    std::vector<std::deque<BasicMsgPtr<T0, T1, T2, T3>>>& input_buff,
    const BasicMsgPtr<T0, T1, T2, T3>& input_frame, const Validation& config,
    const FrameType& curr) {
  if (curr < input_buff.size()) {
    input_buff[curr].emplace_back(input_frame);
    while (input_buff[curr].size() > config.history_path) {
      input_buff[curr].pop_front();
    }
  }
  ADEBUG << "Inserted";
}

template <typename T0, typename T1 = NullType, typename T2 = NullType,
          typename T3 = NullType>
class MatchStrategyI : public MultiSourceMatcherBase<T0, T1, T2, T3> {
 public:
  MatchStrategyI() = delete;
  MatchStrategyI(const Validation& config,
                 MultiSourceMatch_CallBack<T0, T1, T2, T3> func)
      : MultiSourceMatcherBase<T0, T1, T2, T3>(config, func) {}

 private:
  void process_impl(
      const BasicMsgPtr<T0, T1, T2, T3>& input_frame,
      std::vector<BasicMsgPtr<T0, T1, T2, T3>>& matched_list) override {
    // check input
    if (!check_input(this->config_, input_frame)) {
      return;
    }
    // clear output
    matched_list.clear();
    ADEBUG << "[ADDR] {Packaged matching   }" << input_frame.get();

    // find all matched
    FrameType current_type = input_frame->ttype;
    assert(current_type != Others);
    unsigned long base_idx = input_frame->frame_id(current_type);
    ADEBUG << "[IMPL] {Try to find matching element in buffer} " << current_type
          << " , " << base_idx;
    find_matched_with_idx(matched_list, this->input_buff_, base_idx,
                          this->config_, current_type);
    ADEBUG << "Pure matched " << matched_list.size();
    matched_list.emplace_back(input_frame);
    ADEBUG << "Find match size" << matched_list.size() << "?"
          << this->config_.source_num;
    if (matched_list.size() == this->config_.source_num) {
      // if all matched, put into output, clear those before matched
      ADEBUG << "[IMPL] {Found matched and now erase out of date}";
      erase_before(this->input_buff_, base_idx);
    } else {
      // else put into input_buff
      matched_list.clear();
      ADEBUG << "[IMPL] {No matched found and insert into buffer}";
      insert_into_buff(this->input_buff_, input_frame, this->config_,
                       current_type);
    }
  }
};

}  // namespace omnisense::redetection