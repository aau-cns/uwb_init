// Copyright (C) 2022 Martin Scheiber, Alessandro Fornasier,
// Control of Networked Systems, University of Klagenfurt, Austria.
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
// <alessandro.fornasier@aau.at>

#ifndef UAV_INIT_TYPES_DATA_BUFFER_HPP_
#define UAV_INIT_TYPES_DATA_BUFFER_HPP_

#include <unordered_map>

#include "types/buffers/timed_buffer.hpp"

namespace UwbInit
{
template <typename bufferType>
///
/// \brief The DataBuffer class is an object used for storing data in and ID/TimedBuffer data method.
///
class DataBuffer
{
protected:
  std::unordered_map<uint, TimedBuffer<bufferType>> buffer_;  //!< main buffer variable storing the values with timestamps
                                                              //!< encoded

  double buffer_size_s_{ 0.0 };  //!< buffer size in s. If this is <= 0.0 the buffer is assumed to be infinite in size
  bufferType zero_value_;        //!< zero value to return if no entry can be found

  bool f_is_initialized{ false };  //!< flag to deterime if buffer is initialized

public:
  ///
  /// \brief DataBuffer default constructor for any TimedBuffer
  ///
  /// The timed buffer creates a queue of messages, which are timestamped. The messages are assumed to be added in
  /// order, thus currently no checks on where to add the message is eing made.
  ///
  /// \todo add a feature to be able to add messages regardless of their arrival time (timestamp) in an ordered fashion.
  ///
  DataBuffer(){};

  ///
  /// \brief init initialize the TimedBuffer
  /// \param buffer_size_s buffer size in s
  /// \param zero_value
  ///
  virtual void init(const double buffer_size_s, const bufferType zero_value)
  {
    // setup buffer size in s
    if (buffer_size_s <= 0.0)
    {
        // TODO (gid)
//      ROS_WARN("Initializing infinite position buffer (%f)", buffer_size_s);
      buffer_size_s_ = 0.0;
    }
    else
      buffer_size_s_ = buffer_size_s;

    // set zero value
    zero_value_ = zero_value;

    // self reset
    reset();

    // set initialized flag
    f_is_initialized = true;
  }

  ///
  /// \brief reset resets the buffer
  ///
  void reset()
  {
    buffer_.clear();
  }

  ///
  /// \brief push_back adds the value to the buffer and performs checks on its size
  /// \param data_id ID of the data to be added to buffer
  /// \param timestamp timestamp of value to add
  /// \param value value to add to the buffer
  /// \return true if the addition of the value was successful, otherwise false
  ///
  /// \warning This implementation currently only allows to add entries which arrive in an ordered manner. Values with
  /// timestamps older than the 'newest' entry in the buffer are not added and false is returned.
  ///
  /// \todo Implement addition of values out-of-order.
  ///
  bool push_back(uint data_id, double timestamp, bufferType value)
  {
    // check if buffer is initialized
    if (!f_is_initialized)
    {
        // TODO (gid)
//      ROS_ERROR("DataBuffer not initialized yet.");
      return false;
    }

    // perform check if data_id is already existent
#if (__cplusplus >= 202002L)
    // new function for finding keys in maps in >= c++20
    if (!buffer_.contains(data_id))
    {
#else
    typename std::unordered_map<uint, TimedBuffer<bufferType>>::iterator it = buffer_.find(data_id);
    if (it == buffer_.end())
    {
#endif
      // key not found
        // TODO (gid)
//      ROS_DEBUG_STREAM("DataBuffer: key '" << data_id << "' not found, adding new entry");
      buffer_[data_id] = TimedBuffer<bufferType>();
      buffer_[data_id].init(buffer_size_s_, zero_value_);
    }

    // add value to buffer
    buffer_[data_id].push_back(timestamp, value);
    return true;
  }

  ///
  /// \brief get_closest returns the entry of the buffer closes to the given timestamp
  /// \param timestamp
  /// \return
  ///
  bufferType get_closest(const double timestamp) const
  {
    if (buffer_.empty())
    {
        // TODO (gid)
//      ROS_ERROR("PositionBuffer still empty.");
      return zero_value_;
    }

    // TODO(scm) this can be upgraded to interpolate (extrapolate) the positions if time does not match exactly

    // get closest position vector, where the closest is taken as the measurement which is smaller/equal to the current
    // measurement time iterate from back to front thus
    for (uint i = buffer_.size() - 1; i >= 0; --i)
    {
      if (buffer_.at(i).first <= timestamp)
        return buffer_.at(i).second;
    }

    // in case we have not returned any we do not have a measurement in the buffer anymore
    // TODO (gid)
//    ROS_WARN_STREAM("We do not have any value in the buffer for time " << timestamp << " anymore." << std::endl);
    return buffer_.front().second;
  }

  const std::unordered_map<uint, TimedBuffer<bufferType>>& get_buffer() const
  {
    return buffer_;
  }

  const bool get_buffer_values(const uint data_id, std::deque<std::pair<double, bufferType>>& buffer_values) const
  {
    // perform check if data_id is exists
    if (!contains_id(data_id))
    {
      // key not found
      return false;
    }

    // return values
    buffer_values = buffer_.at(data_id).get_buffer();
    return true;
  }

  ///
  /// \brief contains_id checks if the given id is present in the data buffer
  /// \param data_id id to check for
  /// \return true if it is present
  ///
  const bool contains_id(const uint data_id) const
  {
#if (__cplusplus >= 202002L)
    // new function for finding keys in maps in >= c++20
    if (buffer_.contains(data_id))
    {
#else
    typename std::unordered_map<uint, TimedBuffer<bufferType>>::const_iterator it = buffer_.find(data_id);
    if (it != buffer_.end())
    {
#endif
      // key found
      return true;
    }

    // key not found
    return false;
  }

  ///
  /// \brief is_emtpy checks if the buffer is empty
  /// \return true if the buffer is emtpy
  ///
  const bool is_emtpy() const
  {
    return buffer_.empty();
  }

};  // class DataBuffer
}  // namespace UwbInit

#endif  // UAV_INIT_TYPES_DATA_BUFFER_HPP_
