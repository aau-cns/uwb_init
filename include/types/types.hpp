// Copyright (C) 2021 Martin Scheiber, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <martin.scheiber@aau.at>

#ifndef UAV_INIT_TYPES_HPP_
#define UAV_INIT_TYPES_HPP_

#include "types/buffers.hpp"
#include "types/uwb_anchor.hpp"
#include "types/uwb_data.hpp"

#include <Eigen/Eigen>

namespace uav_init
{

class PositionBufferTimed : public TimedBuffer<Eigen::Vector3d>
{
public:
  using TimedBuffer::init;
  PositionBufferTimed() {};
  void init(const double buffer_size_s)
  {
    init(buffer_size_s, Eigen::Vector3d(0,0,0));
  }
}; // class PositionBufferTimed

class UwbDataBuffer : public DataBuffer<UwbData>
{
public:
  using DataBuffer::init;
  UwbDataBuffer() {};
  void init(const double buffer_size_s)
  {
    init(buffer_size_s, UwbData());
  }
}; // class UwbDataBuffer

class UwbAnchorBuffer : public DataBuffer<UwbAnchor>
{
public:
  using DataBuffer::init;
  UwbAnchorBuffer()
  {
    init(1.0, UwbAnchor());
  }
  void init(const double buffer_size_s)
  {
    init(buffer_size_s, UwbAnchor());
  }
}; // class UwbAnchorBuffer

}  // namespace uav_init

#endif  // UAV_INIT_TYPES_HPP_
