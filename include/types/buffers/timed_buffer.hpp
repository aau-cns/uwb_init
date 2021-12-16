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

#ifndef UAV_INIT_TYPES_TIMED_BUFFER_HPP_
#define UAV_INIT_TYPES_TIMED_BUFFER_HPP_

#include <ros/ros.h>

#include <deque>

namespace uav_init
{
template <typename bufferType>
///
/// \brief The TimedBuffer class
///
class TimedBuffer
{
protected:
  std::deque<std::pair<double, bufferType>> buffer_;  //!< main buffer variable storing the values with timestamps
                                                      //!< encoded

  double buffer_size_s_{ 0.0 };  //!< buffer size in s. If this is <= 0.0 the buffer is assumed to be infinite in size
  bufferType zero_value_;        //!< zero value to return if no entry can be found

  bool f_is_initialized{ false };  //!< flag to deterime if buffer is initialized

public:
  ///
  /// \brief TimedBuffer default constructor for any TimedBuffer
  ///
  /// The timed buffer creates a queue of messages, which are timestamped. The messages are assumed to be added in
  /// order, thus currently no checks on where to add the message is eing made.
  ///
  /// \todo add a feature to be able to add messages regardless of their arrival time (timestamp) in an ordered fashion.
  ///
  TimedBuffer(){};

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
      ROS_WARN("Initializing infinite position buffer (%f)", buffer_size_s);
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
  /// \param timestamp timestamp of value to add
  /// \param value value to add to the buffer
  /// \return true if the addition of the value was successful, otherwise false
  ///
  /// \warning This implementation currently only allows to add entries which arrive in an ordered manner. Values with
  /// timestamps older than the 'newest' entry in the buffer are not added and false is returned.
  ///
  /// \todo Implement addition of values out-of-order.
  ///
  bool push_back(double timestamp, bufferType value)
  {
    // check if buffer is initialized
    if (!f_is_initialized)
    {
      ROS_ERROR("TimedBuffer not initialized yet.");
      return false;
    }

    // perform checks on buffer time and input time
    if (buffer_.size() > 0)
    {
      // check for timestamp jump
      if (buffer_.back().first > timestamp)
      {
        ROS_ERROR("Timejump in buffer detected, not adding entry.");
        return false;
      }

      // check if buffer is bigger than targeted size in s
      if (buffer_size_s_ > 0.0)
      {
        // delete all entries, whose timestamp is outside of buffer timestamp size
        while (buffer_.size() > 0 && timestamp - buffer_.front().first > buffer_size_s_)
        {
          buffer_.pop_front();
        }
      }
    }

    // add entry to buffer
    buffer_.push_back(std::make_pair(timestamp, value));
    //    ROS_DEBUG_STREAM("TimedBuffer ("<< buffer_.size() << "): added entry at " << timestamp);
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
      ROS_ERROR("PositionBuffer still empty.");
      return zero_value_;
    }

    // TODO(scm) this can be upgraded to interpolate (extrapolate) the positions if time does not match exactly

    // get closest position vector, where the closest is taken as the measurement which is smaller/equal to the current
    // measurement time iterate from back to front thus
    //    for (uint i = buffer_.size() - 1; i >= 0; --i)
    //    {
    //      if (buffer_.at(i).first <= timestamp)
    //        return buffer_.at(i).second;
    //    }
    for (auto it = buffer_.rbegin(); it != buffer_.rend(); ++it)
    {
      if ((*it).first <= timestamp)
        return (*it).second;
    }

    // in case we have not returned any we do not have a measurement in the buffer anymore
    ROS_WARN_STREAM("We do not have any value in the buffer for time " << timestamp << " anymore." << std::endl);
    return buffer_.front().second;
  }

  ///
  /// \brief get_buffer returns the full buffer with all values
  /// \return a std::deque containing std::pairs of buffer values in the form [timestamp, value]
  ///
  const std::deque<std::pair<double, bufferType>>& get_buffer() const
  {
    return buffer_;
  }

  ///
  /// \brief is_emtpy determines if the buffer is still empty
  /// \return
  ///
  const bool is_emtpy() const
  {
    return buffer_.empty();
  }

};  // class TimedBuffer
}  // namespace uav_init

#endif  // UAV_INIT_TYPES_TIMED_BUFFER_HPP_
