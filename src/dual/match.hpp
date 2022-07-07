#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

namespace omnisense::match {

template <typename T0, typename T1>
using TMatch_CallBack =
    std::function<void(const std::shared_ptr<T0>&, const std::shared_ptr<T1>&)>;

template <typename T0, typename T1>
struct MatchPair {
  std::shared_ptr<T0> msg0 = nullptr;
  std::shared_ptr<T1> msg1 = nullptr;
};

template <typename T0, typename T1, typename T3>
class TMatch {
 public:
  TMatch() = delete;
  TMatch(size_t buffer_size, TMatch_CallBack<T0, T1> func)
      : m_buffer_size_(buffer_size), m_func_(func) {}
  ~TMatch() {}

  void add_msg0(const std::shared_ptr<T0>& msg0) {
    T3 target = msg0->frame_id();
    MatchPair<T0, T1> res;
    res.msg0 = msg0;
    std::unique_lock<std::mutex> ul(this->buff_mtx_);
    auto iter = m_msg1_buffer_.find(target);
    if (iter != m_msg1_buffer_.end()) {
      res.msg1 = iter->second;
    }
    if (res.msg1 == nullptr) {
      m_msg0_buffer_.insert({target, msg0});
    }
    ul.unlock();
    if (res.msg1 != nullptr) {
      m_func_(res.msg0, res.msg1);
    }
  }
  void add_msg1(const std::shared_ptr<T1>& msg1) {
    T3 target = msg1->frame_id();
    MatchPair<T0, T1> res;
    res.msg1 = msg1;
    std::unique_lock<std::mutex> ul(this->buff_mtx_);
    auto iter = m_msg0_buffer_.find(target);
    if (iter != m_msg0_buffer_.end()) {
      res.msg0 = iter->second;
    }
    if (res.msg0 == nullptr) {
      m_msg1_buffer_.insert({target, msg1});
    }
    ul.unlock();
    if (res.msg0 != nullptr) {
      m_func_(res.msg0, res.msg1);
    }
  }

 private:
  size_t m_buffer_size_;
  TMatch_CallBack<T0, T1> m_func_;
  std::mutex buff_mtx_;
  std::unordered_map<T3, std::shared_ptr<T0>> m_msg0_buffer_;
  std::unordered_map<T3, std::shared_ptr<T1>> m_msg1_buffer_;
};

}  // namespace omnisense::match