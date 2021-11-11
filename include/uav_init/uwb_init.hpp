// Copyright (C) 2021 Martin Scheiber, Alessandro Fornasier
// Control of Networked Systems, Universitaet Klagenfurt, Austria
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
// <alessandro.fornasier@aau.at>

#ifndef UAV_INIT_UWB_INIT_HPP_
#define UAV_INIT_UWB_INIT_HPP_

#include <ros/ros.h>

#include <Eigen/Dense>
#include <deque>
#include <map>

#include "options/uwb_init_options.hpp"
#include "types/types.hpp"

namespace uav_init
{
///
/// \brief The UwbInitializer class is an object used for UWB data handeling for the purpose of initializing the UWB
/// anchors.
///
class UwbInitializer
{
public:
  ///
  /// \brief UwbInitializer default constructor
  /// \param params parameter/options used for UWB initialization
  ///
  UwbInitializer(UwbInitOptions& params) : params_(params)
  {
    // initialize buffers
    buffer_p_UinG_.init(params_.buffer_size_s);
    uwb_data_buffer_.init(params_.buffer_size_s);
  }

  ///
  /// \brief feed_uwb stores incoming UWB (valid) readings
  /// \param uwb_measurements UwbData vector of measurements
  ///
  /// \todo allow feeding of old(er) measurements
  ///
  void feed_uwb(const std::vector<UwbData> uwb_measurements);

  ///
  /// \brief feed_pose stores incoming positions of the UAV in the global frame
  /// \param uwb_measurements Eigen::Vector3d of positions of the UAV in global frame
  ///
  /// \todo allow feeding of old(er) measurements
  ///
  void feed_pose(const double timestamp, const Eigen::Vector3d p_UinG);

  ///
  /// \brief try_to_initialize_anchors tries to initialize all anchors for which readings exist
  /// \param anchor_buffer
  /// \return true if all anchors were successfully initialized
  ///
  /// This function performs a least-squares initialization using the per anchor measurments given the measurement model
  /// from Blueml et al. It will try to initialize each anchor (validated per ID) individually. If an anchor was already
  /// successfully initialized in the past it is skipped.
  /// It will also return 'true' if all anchors, for which measurements are present were successfully initialized at
  /// some point.
  ///
  /// \cite Bluemel J., Fornasier A., and Weiss S., "Bias Compensated UWB Anchor Initialization using
  /// Information-Theoretic Supported Triangulation Points", 2021 IEEE International Conference on Robotics and
  /// Automation (ICRA21), IEEE, 2021.
  ///
  /// \todo allow initialization of anchors, if condition number is better, also after they have been already
  /// initialized
  ///
  bool try_to_initialize_anchors(UwbAnchorBuffer& anchor_buffer);

protected:
  UwbInitOptions params_;  //!< initializer parameters

  // anchor and measurement handeling
  Eigen::Vector3d cur_p_UinG_;         //!< current position of the UWB module in global frame
  PositionBufferTimed buffer_p_UinG_;  //!< buffer of UWB module positions in global frame

  UwbDataBuffer uwb_data_buffer_;  //!< history of uwb readings in DataBuffer
};

}  // namespace uav_init

#endif  // UAV_INIT_UWB_INIT_HPP_
