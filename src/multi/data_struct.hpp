#pragma once

#include "math.h"

#include "modules/omnisense/common/proto/inno_box.pb.h"
#include "modules/omnisense/common/proto/innovusion.pb.h"
#include "modules/omnisense/common/proto/pose.pb.h"

#include "cyber/common/log.h"
#include "cyber/cyber.h"
using omnisense::box::Boundaries;
using omnisense::box::Boxes;
using omnisense::drivers::innovusion::PointCloud2;
using omnisense::drivers::innovusion::PointXYZITL;
using omnisense::drivers::innovusion::RoadSurface;

#define ERROR_SOURCE_ID 0U
#define PSUDO_SOURCE_ID (std::numeric_limits<unsigned int>::max)()
#define ERROR_TIMESTAMP 0UL
#define ERROR_FRAME_ID (std::numeric_limits<unsigned long>::max)()
#define REAL_FRAME_ID 0UL

namespace omnisense::redetection {

enum FrameType {
  AILabel = 0,
  AIBoxes = 1,
  Ground = 2,
  Lane = 3,
  MSPointCloud = 4,
  Others = 7
};

struct Validation {
  size_t source_num;
  size_t history_path;
  Validation() = default;
  Validation(size_t num, int path)
      : source_num(num), history_path(std::max(std::min(10, path), 5)) {}
  Validation(const Validation& other)
      : source_num(other.source_num), history_path(other.history_path) {}
};

class NullType {};

// debug message
template <typename T>
std::string debug_msg(const std::shared_ptr<T>& msg) {
  return "unknown type message";
}

template <>
std::string debug_msg(const std::shared_ptr<NullType>& msg) {
  return "NullType";
}

template <>
std::string debug_msg(const std::shared_ptr<PointCloud2>& msg) {
  return msg == nullptr
             ? "PointCloud2: nullptr"
             : "PointCloud2: source_id: " + std::to_string(msg->source_id()) +
                   ",frame_id: " + std::to_string(msg->idx()) +
                   ",frame_ns_end: " + std::to_string(msg->frame_ns_end()) +
                   ",point_size: " + std::to_string(msg->point_size());
}

template <>
std::string debug_msg(const std::shared_ptr<RoadSurface>& msg) {
  return msg == nullptr
             ? "RoadSurface: nullptr"
             : "RoadSurface: source_id: " + std::to_string(msg->source_id()) +
                   ",frame_id: " + std::to_string(msg->idx()) +
                   ",confidence: " + std::to_string(msg->confidence()) +
                   ",equation: " + std::to_string(msg->param_a()) + "x + " +
                   std::to_string(msg->param_b()) + "y + " +
                   std::to_string(msg->param_c()) + "z + " +
                   std::to_string(msg->param_d()) + " = 0";
}

template <>
std::string debug_msg(const std::shared_ptr<Boundaries>& msg) {
  return msg == nullptr
             ? "Boundaries: nullptr"
             : "Boundaries: source_id: " + std::to_string(msg->source_id()) +
                   ",frame_id: " + std::to_string(msg->idx()) +
                   ",boundaries size: " + std::to_string(msg->boundary_size());
}

template <>
std::string debug_msg(const std::shared_ptr<Boxes>& msg) {
  return msg == nullptr
             ? "Boxes: nullptr"
             : "Boxes: source_id: " + std::to_string(msg->source_id()) +
                   ",frame_id: " + std::to_string(msg->idx()) +
                   ",box size: " + std::to_string(msg->box_size());
}

// (TODO : check message)
template <typename T>
bool is_msg_valid(const std::shared_ptr<T>& msg, const Validation& config) {
  AERROR << "UNKNOW MESSAGE TYPE";
  return false;
}
template <>
bool is_msg_valid(const std::shared_ptr<NullType>& msg,
                  const Validation& config) {
  return true;
}
template <>
bool is_msg_valid(const std::shared_ptr<PointCloud2>& msg,
                  const Validation& config) {
  return true;
}
template <>
bool is_msg_valid(const std::shared_ptr<RoadSurface>& msg,
                  const Validation& config) {
  return true;
}
template <>
bool is_msg_valid(const std::shared_ptr<Boundaries>& msg,
                  const Validation& config) {
  return true;
}
template <>
bool is_msg_valid(const std::shared_ptr<Boxes>& msg, const Validation& config) {
  return true;
}

// Crucial : frame_id or idx
template <typename T>
unsigned long message_frame_id(const std::shared_ptr<T>& msg) {
  AERROR << "UNKNOW MESSAGE TYPE";
  return ERROR_FRAME_ID;
}
template <>
unsigned long message_frame_id(const std::shared_ptr<NullType>& msg) {
  return ERROR_FRAME_ID;
}
template <>
unsigned long message_frame_id(const std::shared_ptr<PointCloud2>& msg) {
  ADEBUG << "[ADDR] {PointCloud check    }" << msg.get();
  return msg == nullptr ? ERROR_FRAME_ID : msg->idx();
}
template <>
unsigned long message_frame_id(const std::shared_ptr<RoadSurface>& msg) {
  ADEBUG << "[ADDR] {Surface check       }" << msg.get();
  return msg == nullptr ? ERROR_FRAME_ID : msg->idx();
}
template <>
unsigned long message_frame_id(const std::shared_ptr<Boundaries>& msg) {
  ADEBUG << "[ADDR] {Boundaries check    }" << msg.get();
  return msg == nullptr ? ERROR_FRAME_ID : msg->idx();
}
template <>
unsigned long message_frame_id(const std::shared_ptr<Boxes>& msg) {
  ADEBUG << "[ADDR] {Boxes check         }" << msg.get();
  return msg == nullptr ? ERROR_FRAME_ID : msg->idx();
}

// // (TODO : source id should be the same all the time)
// template <typename T>
// unsigned int message_source_id(const std::shared_ptr<T>& msg) {
//   AERROR << "UNKNOW MESSAGE TYPE";
//   return ERROR_SOURCE_ID;
// }
// template <>
// unsigned int message_source_id(const std::shared_ptr<NullType>& msg) {
//   return ERROR_SOURCE_ID;
// }
// template <>
// unsigned int message_source_id(const std::shared_ptr<PointCloud2>& msg) {
//   return msg == nullptr ? ERROR_SOURCE_ID : msg->source_id();
// }
// template <>
// unsigned int message_source_id(const std::shared_ptr<RoadSurface>& msg) {
//   return msg == nullptr ? ERROR_SOURCE_ID : msg->source_id();
// }
// template <typename T>
// void set_messsage_source_id(const std::shared_ptr<T>& msg,
//                             unsigned int _source_id) {
//   AERROR << "UNKNOW MESSAGE TYPE";
// }
// template <>
// void set_messsage_source_id(const std::shared_ptr<NullType>& msg,
//                             unsigned int _source_id) {}
// template <>
// void set_messsage_source_id(const std::shared_ptr<PointCloud2>& msg,
//                             unsigned int _source_id) {
//   if (msg != nullptr) {
//     msg->set_source_id(_source_id);
//   }
// }
// template <>
// void set_messsage_source_id(const std::shared_ptr<RoadSurface>& msg,
//                             unsigned int _source_id) {
//   if (msg != nullptr) {
//     msg->set_source_id(_source_id);
//   }
// }

// // Crucial : timestamp
// template <typename T>
// unsigned long message_timestamp(const std::shared_ptr<T>& msg) {
//   AERROR << "UNKNOW MESSAGE TYPE";
//   return ERROR_TIMESTAMP;
// }
// template <>
// unsigned long message_timestamp(const std::shared_ptr<NullType>& msg) {
//   return ERROR_TIMESTAMP;
// }
// template <typename T>
// unsigned long set_sys_time(const std::shared_ptr<T>& frame) {
//   return ERROR_TIMESTAMP;
// }
// template <typename T>
// unsigned long get_sys_time(const std::shared_ptr<T>& frame) {
//   return ERROR_TIMESTAMP;
// }

// SourceFrame
template <typename T0, typename T1 = NullType, typename T2 = NullType,
          typename T3 = NullType>
struct SourceFrame {
  std::shared_ptr<T0> sensor0;
  std::shared_ptr<T1> sensor1;
  std::shared_ptr<T2> sensor2;
  std::shared_ptr<T3> sensor3;
  FrameType ttype;

  SourceFrame()
      : sensor0(nullptr),
        sensor1(nullptr),
        sensor2(nullptr),
        sensor3(nullptr),
        ttype(Others) {}
  SourceFrame(FrameType type, const std::shared_ptr<T0>& _sensor0,
              const std::shared_ptr<T1>& _sensor1 = nullptr,
              const std::shared_ptr<T2>& _sensor2 = nullptr,
              const std::shared_ptr<T3>& _sensor3 = nullptr)
      : sensor0(_sensor0),
        sensor1(_sensor1),
        sensor2(_sensor2),
        sensor3(_sensor3),
        ttype(type) {}
  SourceFrame(const SourceFrame& other)
      : sensor0(other.sensor0),
        sensor1(other.sensor1),
        sensor2(other.sensor2),
        sensor3(other.sensor3),
        ttype(other.ttype) {}

  std::string printf_frame() const {
    return "\n msg0: " + debug_msg(sensor0) + "\n msg1: " + debug_msg(sensor1) +
           "\n msg2: " + debug_msg(sensor2) + "\n msg3: " + debug_msg(sensor3);
  }
  bool is_frame_valid(const Validation& config) const {
    return is_msg_valid(sensor0, config) & is_msg_valid(sensor1, config) &
           is_msg_valid(sensor2, config) & is_msg_valid(sensor3, config);
  }
  //   unsigned int source_id(int i = 0) const {
  //     if (psudo_idx != REAL_FRAME_ID) {
  //       return ERROR_SOURCE_ID;
  //     }
  //     switch (i) {
  //       case 0:
  //         return message_source_id(sensor0);
  //       case 1:
  //         return message_source_id(sensor1);
  //       case 2:
  //         return message_source_id(sensor2);
  //       case 3:
  //         return message_source_id(sensor3);
  //       case -1:
  //         return std::max(
  //             std::max(message_source_id(sensor0),
  //             message_source_id(sensor1)),
  //             std::max(message_source_id(sensor2),
  //             message_source_id(sensor3)));
  //       default:
  //         return ERROR_SOURCE_ID;
  //     }
  //   }
  unsigned long frame_id(int type) const {
    switch (FrameType(type)) {
      case AILabel:
        return message_frame_id(sensor0);
      case AIBoxes:
        return message_frame_id(sensor3);
      case Ground:
        return message_frame_id(sensor1);
      case Lane:
        return message_frame_id(sensor2);
      case MSPointCloud:
        return message_frame_id(sensor0);
      default:
        return ERROR_FRAME_ID;
    }
  }
  //   unsigned long timestamp(int i = 0) const {
  //     if (psudo_idx != REAL_FRAME_ID) {
  //       return ERROR_TIMESTAMP;
  //     }
  //     switch (i) {
  //       case 0:
  //         return message_timestamp(sensor0);
  //       case 1:
  //         return message_timestamp(sensor1);
  //       case 2:
  //         return message_timestamp(sensor2);
  //       case 3:
  //         return message_timestamp(sensor3);
  //       case -1:
  //         return std::max(
  //             std::max(message_timestamp(sensor0),
  //             message_timestamp(sensor1)),
  //             std::max(message_timestamp(sensor2),
  //             message_timestamp(sensor3)));
  //       default:
  //         return ERROR_TIMESTAMP;
  //     }
  //   }
};

template <typename T0, typename T1 = NullType, typename T2 = NullType,
          typename T3 = NullType>
using BasicMsgPtr = std::shared_ptr<SourceFrame<T0, T1, T2, T3>>;

template <typename T0, typename T1 = NullType, typename T2 = NullType,
          typename T3 = NullType>
std::string printf_matched_list(
    const std::vector<BasicMsgPtr<T0, T1, T2, T3>>& matched_list) {
  std::string matched_list_info_str =
      "matched_list size is " + std::to_string(matched_list.size());
  for (size_t i = 0; i < matched_list.size(); ++i) {
    const auto& frame = matched_list[i];
    if (frame != nullptr) {
      matched_list_info_str +=
          "\n[" + std::to_string(i) + "]:" + frame->printf_frame();
    } else {
      matched_list_info_str += "\n[" + std::to_string(i) + "]:nullptr";
    }
  }
  return matched_list_info_str;
}

template <typename T0, typename T1 = NullType, typename T2 = NullType,
          typename T3 = NullType>
std::string matched_timestamp(
    const std::vector<BasicMsgPtr<T0, T1, T2, T3>>& matched_list) {
  unsigned long min_ts_ns = (std::numeric_limits<unsigned long>::max)();
  unsigned long max_ts_ns = 0ul;
  std::string timestamp_info_str = "";
  for (size_t i = 0; i < matched_list.size(); ++i) {
    if (matched_list[i] != nullptr) {
      unsigned long tmp_ts_ns = matched_list[i]->timestamp(0);
      min_ts_ns = std::min(min_ts_ns, tmp_ts_ns);
      max_ts_ns = std::max(max_ts_ns, tmp_ts_ns);
      timestamp_info_str += std::to_string(tmp_ts_ns) + "/";
    }
  }
  timestamp_info_str += std::to_string(max_ts_ns - min_ts_ns);
  return timestamp_info_str;
}

}  // namespace omnisense::redetection