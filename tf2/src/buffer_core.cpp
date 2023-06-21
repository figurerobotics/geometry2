// Copyright 2010, Willow Garage, Inc. All rights reserved.
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
//    * Neither the name of the Willow Garage nor the names of its
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

/** \author Tully Foote */

#include "tf2/buffer_core.h"

#include "tf2/buffer_core.inl"

namespace tf2 {

template <>
bool BufferCore<msg::TransformStamped>::setTransform(const msg::TransformStamped& transform,
                                                     const std::string& authority, bool is_static) {
  tf2::Transform tf2_transform(
      tf2::Quaternion(transform.transform.rotation.x, transform.transform.rotation.y,
                      transform.transform.rotation.z, transform.transform.rotation.w),
      tf2::Vector3(transform.transform.translation.x, transform.transform.translation.y,
                   transform.transform.translation.z));
  TimePoint time_point(std::chrono::nanoseconds(transform.header.stamp.nanosec) +
                       std::chrono::duration_cast<std::chrono::nanoseconds>(
                           std::chrono::seconds(transform.header.stamp.sec)));
  return setTransformImpl(tf2_transform, transform.header.frame_id, transform.child_frame_id,
                          time_point, authority, is_static);
}

template <>
msg::TransformStamped BufferCore<msg::TransformStamped>::lookupTransform(
    const std::string& target_frame, const std::string& source_frame, const TimePoint& time) const {
  tf2::Transform transform;
  TimePoint time_out;
  lookupTransformImpl(target_frame, source_frame, time, transform, time_out);
  msg::TransformStamped msg;
  msg.transform.translation.x = transform.getOrigin().x();
  msg.transform.translation.y = transform.getOrigin().y();
  msg.transform.translation.z = transform.getOrigin().z();
  msg.transform.rotation.x = transform.getRotation().x();
  msg.transform.rotation.y = transform.getRotation().y();
  msg.transform.rotation.z = transform.getRotation().z();
  msg.transform.rotation.w = transform.getRotation().w();
  std::chrono::nanoseconds ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(time_out.time_since_epoch());
  std::chrono::seconds s =
      std::chrono::duration_cast<std::chrono::seconds>(time_out.time_since_epoch());
  msg.header.stamp.sec = static_cast<int32_t>(s.count());
  msg.header.stamp.nanosec = static_cast<uint32_t>(ns.count() % 1000000000ull);
  msg.header.frame_id = target_frame;
  msg.child_frame_id = source_frame;

  return msg;
}

template <>
msg::TransformStamped BufferCore<msg::TransformStamped>::lookupTransform(
    const std::string& target_frame, const TimePoint& target_time, const std::string& source_frame,
    const TimePoint& source_time, const std::string& fixed_frame) const {
  tf2::Transform transform;
  TimePoint time_out;
  lookupTransformImpl(target_frame, target_time, source_frame, source_time, fixed_frame, transform,
                      time_out);
  msg::TransformStamped msg;
  msg.transform.translation.x = transform.getOrigin().x();
  msg.transform.translation.y = transform.getOrigin().y();
  msg.transform.translation.z = transform.getOrigin().z();
  msg.transform.rotation.x = transform.getRotation().x();
  msg.transform.rotation.y = transform.getRotation().y();
  msg.transform.rotation.z = transform.getRotation().z();
  msg.transform.rotation.w = transform.getRotation().w();
  std::chrono::nanoseconds ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(time_out.time_since_epoch());
  std::chrono::seconds s =
      std::chrono::duration_cast<std::chrono::seconds>(time_out.time_since_epoch());
  msg.header.stamp.sec = static_cast<int32_t>(s.count());
  msg.header.stamp.nanosec = static_cast<uint32_t>(ns.count() % 1000000000ull);
  msg.header.frame_id = target_frame;
  msg.child_frame_id = source_frame;

  return msg;
}

// Explicit template specialization
template class BufferCore<msg::TransformStamped>;
}  // namespace tf2
