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

namespace uav_init
{
typedef TimedBuffer<Eigen::Vector3d> PositionBuffer;
typedef std::unordered_map<size_t, TimedBuffer<UwbData>> UwbDataBuffer;
typedef std::unordered_map<size_t, UwbAnchor> UwbAnchorBuffer;
}  // namespace uav_init

#endif  // UAV_INIT_TYPES_HPP_
