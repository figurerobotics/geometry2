// Copyright 2019, Open Source Robotics Foundation, Inc. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Open Source Robotics Foundation nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <string>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"

namespace tf2::msg {

struct Header {
  uint32_t seq;
  struct Stamp {
    int32_t sec;
    uint32_t nanosec;
  } stamp;
  std::string frame_id;
};

struct Transform {
  struct Rotation {
    double x;
    double y;
    double z;
    double w;
  } rotation;

  struct Translation {
    double x;
    double y;
    double z;
  } translation;
};

struct TransformStamped {
  Header header;
  std::string child_frame_id;
  Transform transform;
};

struct Quaternion {
  double x;
  double y;
  double z;
  double w;
};

struct QuaternionStamped {
  Header header;
  Quaternion quaternion;
};

// is_message
template <typename T>
struct is_message : std::false_type {};

template <>
struct is_message<Transform> : std::true_type {};

template <>
struct is_message<TransformStamped> : std::true_type {};

template <>
struct is_message<Quaternion> : std::true_type {};

template <>
struct is_message<QuaternionStamped> : std::true_type {};

}  // namespace tf2::msg

namespace tf2 {

inline void fromMsg(const msg::Quaternion& in, Quaternion& out) {
  // w at the end in the constructor
  out = Quaternion(in.x, in.y, in.z, in.w);
}

inline msg::Quaternion toMsg(const Quaternion& in) {
  return {
      .x = in.x(),
      .y = in.y(),
      .z = in.z(),
      .w = in.w(),
  };
}

inline void fromMsg(const msg::Transform& in, Transform& out) {
  out = Transform(Quaternion(in.rotation.x, in.rotation.y, in.rotation.z, in.rotation.w),
                  Vector3(in.translation.x, in.translation.y, in.translation.z));
}

inline msg::Transform toMsg(const Transform& in) {
  return {
      .rotation =
          {
              .x = in.getRotation().x(),
              .y = in.getRotation().y(),
              .z = in.getRotation().z(),
              .w = in.getRotation().w(),
          },
      .translation =
          {
              .x = in.getOrigin().x(),
              .y = in.getOrigin().y(),
              .z = in.getOrigin().z(),
          },
  };
}

}  // namespace tf2
