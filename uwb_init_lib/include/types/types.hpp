// Copyright (C) 2021 Martin Scheiber, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.
//
// You can contact the author at <martin.scheiber@aau.at>

#ifndef UAV_INIT_TYPES_HPP_
#define UAV_INIT_TYPES_HPP_

#include <Eigen/Eigen>

#include "types/buffers.hpp"
#include "types/uwb_anchor.hpp"
#include "types/uwb_data.hpp"

namespace UwbInit
{
///
/// \brief The PositionBufferTimed class TimedBuffer buffer handler for Eigen::Vector3d.
///
class PositionBufferTimed : public TimedBuffer<Eigen::Vector3d>
{
public:
  using TimedBuffer::init;
  PositionBufferTimed(){};
  void init(const double buffer_size_s)
  {
    init(buffer_size_s, Eigen::Vector3d(0, 0, 0));
  }
};  // class PositionBufferTimed

///
/// \brief The UwbDataBuffer class DataBuffer buffer handler for UwbData.
///
class UwbDataBuffer : public DataBuffer<UwbData>
{
public:
  using DataBuffer::init;
  UwbDataBuffer(){};
  void init(const double buffer_size_s)
  {
    init(buffer_size_s, UwbData());
  }
};  // class UwbDataBuffer

///
/// \brief The UwbAnchorBuffer class is a DataBuffer buffer handler for UwbAnchor.
///
class UwbAnchorBuffer : public DataBuffer<UwbAnchor>
{
public:
  using DataBuffer::init;
  ///
  /// \brief UwbAnchorBuffer default constructor for the UwbAnchorBuffer. This already sets the buffer size to 1.0 s and
  /// the data type to UwbAnchor.
  ///
  UwbAnchorBuffer()
  {
    init(1.0, UwbAnchor());
  }

  ///
  /// \brief init optional init function with just the buffer size in s to give
  /// \param buffer_size_s buffer size in s (since last update)
  ///
  void init(const double buffer_size_s)
  {
    init(buffer_size_s, UwbAnchor());
  }

  ///
  /// \brief is_initialized checks if the anchor with the given ID has been initialized
  /// \param anchor_id ID of anchor to check
  /// \return true if the anchor id exists and has been initialized
  ///
  const bool is_initialized(const uint anchor_id) const
  {
    return contains_id(anchor_id) && buffer_.at(anchor_id).get_buffer().back().second.initialized;
  }
};  // class UwbAnchorBuffer

}  // namespace UwbInit

#endif  // UAV_INIT_TYPES_HPP_
