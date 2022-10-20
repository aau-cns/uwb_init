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
// You can contact the author at <giulio.delama@aau.at>

#ifndef UAV_INIT_TYPES_SIMPLE_BUFFER_HPP_
#define UAV_INIT_TYPES_SIMPLE_BUFFER_HPP_

#include <unordered_map>

namespace UavInit
{
template <typename dataType>
///
/// \brief The DataBuffer class is an object used for storing data
///
class SimpleBuffer
{
protected:
    std::unordered_map<uint, dataType> buffer_;  //!< main buffer variable storing the values

public:
    ///
    /// \brief SimpleBuffer
    ///
    /// The timed buffer creates a queue of messages, which are timestamped. The messages are assumed to be added in
    /// order, thus currently no checks on where to add the message is eing made.
    ///
    SimpleBuffer(){}

    ///
    /// \brief reset resets the buffer
    ///
    inline void reset()
    {
        buffer_.clear();
    }

    ///
    /// \brief set adds the value to the buffer and overwrite element if data_id is already present
    /// \param data_id ID of the data to be added to buffer
    /// \param value value to add to the buffer
    /// \return true if the addition of the value was successful, otherwise false
    ///
    inline void set(const uint data_id, const dataType value)
    {
        // add value to buffer
        buffer_[data_id] = value;
    }

    ///
    /// \brief get returns the value to the buffer
    /// \param data_id ID of the data to be added to buffer
    /// \return dataType value
    ///
    inline const dataType get(const uint data_id)
    {
        return buffer_[data_id];
    }

    ///
    /// \brief contains_id checks if the given id is present in the data buffer
    /// \param data_id id to check for
    /// \return true if it is present
    ///
    inline bool contains_id(const uint data_id) const
    {
#if (__cplusplus >= 202002L)
        // new function for finding keys in maps in >= c++20
        if (buffer_.contains(data_id))
        {
#else
        typename std::unordered_map<uint, dataType>::const_iterator it = buffer_.find(data_id);
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
    inline bool is_emtpy() const
    {
        return buffer_.empty();
    }

    ///
    /// \brief get_buffer returns the iterable buffer
    /// \todo TODO (gid) make the whole class iterable
    ///
    const std::unordered_map<uint, dataType> get_buffer()
    {
        return buffer_;
    }

};  // class DataBuffer
}  // namespace UavInit

#endif  // UAV_INIT_TYPES_DATA_BUFFER_HPP_
