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

#ifndef UAV_INIT_TYPES_UWB_DATA_HPP_
#define UAV_INIT_TYPES_UWB_DATA_HPP_

#include <ros/ros.h>

namespace uav_init
{
struct UwbData
{
  double timestamp{ 0.0 };  //!< timestamp of measurement
  bool valid{ false };      //!< validity flag, determines if distance is valid
  double distance{ -1.0 };  //!< distance between anchor and tag
  uint id{ 0 };             //!< anchor ID

  ///
  /// \brief UwbData advanced constructor for measurement, using ros::Time::now() for timestamp
  /// \param _valid validity flag
  /// \param _distance distance measurement
  ///
  UwbData(bool _valid, double _distance) : valid(_valid), distance(_distance)
  {
    timestamp = ros::Time::now().toSec();
  };

  ///
  /// \brief UwbData default constructor for measruement
  /// \param _timestamp timestamp of measurement
  /// \param _valid validity flag
  /// \param _distance distance measurement
  /// \param _id anchor ID
  ///
  UwbData(double _timestamp, bool _valid, double _distance, u_int16_t _id)
    : timestamp(_timestamp), valid(_valid), distance(_distance), id(_id){};

  UwbData() {};
};  // struct UwbData

}  // namespace uav_init

#endif  // UAV_INIT_TYPES_UWB_DATA_HPP_
