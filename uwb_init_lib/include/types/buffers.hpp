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
// You can contact the authors at <martin.scheiber@aau.at> and
// <giulio.delama@aau.at>

#ifndef UAV_INIT_TYPES_BUFFERS_HPP_
#define UAV_INIT_TYPES_BUFFERS_HPP_

#include <vector>
#include <algorithm>

template <typename bufferType>

struct TimedBuffer
{
    std::vector<std::pair<double, bufferType>> buffer_;  // main buffer variable storing the values with timestamps

    inline bool prev(const double timestamp, bufferType& prev) {
        sort(buffer_.begin(), buffer_.end());

        auto const it = std::lower_bound(buffer_.begin(), buffer_.end(), timestamp,
                                         [](std::pair<double, bufferType> const & lhs, double const & rhs)
                                            { return lhs.first < rhs; });

        if (it == buffer_.end()) {
            return false;
        }

        prev = (*it).second;

        return true;
    }

    inline bool next(const double timestamp, bufferType& next) {
        sort(buffer_.begin(), buffer_.end());

        auto const it = std::upper_bound(buffer_.begin(), buffer_.end(), timestamp,
                                         [](std::pair<double, bufferType> const & lhs, double const & rhs)
                                            { return lhs.first > rhs; });

        if (it == buffer_.end()) {
            return false;
        }

        next = (*it).second;

        return true;
    }

    TimedBuffer(){}

};  // struct TimedBuffer

#endif  // UAV_INIT_TYPES_BUFFERS_HPP_
