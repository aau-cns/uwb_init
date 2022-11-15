// Copyright (C) 2022 Alessandro Fornasier, Giulio Delama and Martin Scheiber,
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
// You can contact the authors at <alessandro.fornasier@aau.at>,
// <giulio.delama@aau.at> and <martin.scheiber@aau.at>

#ifndef TIMED_BUFFER_HPP_
#define TIMED_BUFFER_HPP_

#include <algorithm>
#include <vector>

#include "utils/utils.hpp"

namespace uwb_init
{
template <typename BufferType>
class TimedBuffer
{
public:
  TimedBuffer()
  {
  }

  /**
   * @brief Operator []
   * @param index
   * @return const reference to std::pair<double, BufferType>
   */
  inline const std::pair<double, BufferType>& operator[](size_t index) const
  {
    return buffer_[index];
  }

  /**
   * @brief Get the buffer object
   *
   * @return const reference to std::vector<std::pair<double, BufferType>>
   */
  inline const std::vector<std::pair<double, BufferType>>& get_buffer() const
  {
    return buffer_;
  }

  /**
   *
   * @brief Return the size of the buffer
   * @return Size of buffer
   */
  inline const size_t size() const
  {
    return buffer_.size();
  }

  /**
   * @brief Return true if the buffer is empty
   * @return True if buffer is empty, false otherwise
   */
  [[nodiscard]] inline bool empty() const
  {
    return buffer_.empty();
  }

  /**
   * @brief Push a new element into buffer
   *
   * @param timestamp const reference to double
   * @param elem const reference to BufferType
   */
  inline void push_back(const double& timestamp, const BufferType& elem)
  {
    buffer_.emplace_back(std::make_pair(timestamp, elem));
  }

  /**
   * @brief Push a new element into buffer
   *
   * @param p const reference to std::pair<double, BufferType>
   */
  inline void push_back(const std::pair<double, BufferType>& p)
  {
    buffer_.emplace_back(p);
  }

  /**
   * @brief Clear the buffer
   *
   */
  inline void clear()
  {
    buffer_.clear();
  }

  /**
   * @brief Get the closest element to given timestamp from unsorted TimedBuffer
   *
   * @param timestamp
   * @return BufferType
   */
  inline BufferType get_closest(const double& timestamp) const
  {
    return get_closest_cit(timestamp)->second;
  }

  /**
   * @brief Get an object of BufferType at timestamp.
   * If this object does not exist the function perform linear interpolation of the object
   *
   * @param timestamp
   * @return BufferType
   */
  inline BufferType get_at_timestamp(const double& timestamp) const
  {
    // Check if we have an element at a given timestamp, if not perform linear interpolation
    auto it =
        std::find_if(buffer_.cbegin(), buffer_.cend(),
                     [&timestamp](const std::pair<double, BufferType>& element) { return element.first == timestamp; });
    if (it != buffer_.cend())
    {
      return it->second;
    }
    else
    {
      // Get closest iterator
      auto it = get_closest_cit(timestamp);

      // Check if closest timestamp is before or after timestamp
      if (it->first < timestamp)
      {
        // Get the first element and timestamp
        double t0 = it->first;
        BufferType elem0 = it->second;

        // Increment iterator
        ++it;

        // get the second element and timestamp
        double t1 = it->first;
        BufferType elem1 = it->second;

        return lerp(elem0, elem1, (timestamp - t0) / (t1 - t0));
      }
      else
      {
        // Get the secont element and timestamp
        double t1 = it->first;
        BufferType elem1 = it->second;

        // Decrement iterator
        --it;

        // get the first element and timestamp
        double t0 = it->first;
        BufferType elem0 = it->second;

        return lerp(elem0, elem1, (timestamp - t0) / (t1 - t0));
      }
    }
  }

private:
  /**
   * @brief Get a constant iterator to closest element to given timestamp from unsorted TimedBuffer
   *
   * @param timestamp
   * @return const iterator (std::vector<std::pair<double, BufferType>>::const_iterator)
   */
  inline typename std::vector<std::pair<double, BufferType>>::const_iterator
  get_closest_cit(const double& timestamp) const
  {
    // Check that buffer is not empty
    if (buffer_.empty())
    {
      throw std::range_error("TimedBuffer::get_closest called for empty buffer");
    }

    // retrun iterator to closest element
    return std::min_element(
        buffer_.cbegin(), buffer_.cend(),
        [&timestamp](const std::pair<double, BufferType>& elem_pre, const std::pair<double, BufferType>& elem_post) {
          return std::abs(elem_pre.first - timestamp) < std::abs(elem_post.first - timestamp);
        });
  }

  /// Buffer
  std::vector<std::pair<double, BufferType>> buffer_;
};
}  // namespace uwb_init

#endif  // TIMED_BUFFERS_HPP_
