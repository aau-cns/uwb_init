// Copyright (C) 2021 Martin Scheiber, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <martin.scheiber@aau.at>

#ifndef UAV_INIT_UWB_OPTIONS_HPP_
#define UAV_INIT_UWB_OPTIONS_HPP_

#include <Eigen/Eigen>

namespace uav_init
{
struct UwbInitOptions
{
  // WRAPPER AND CALIBRATION ==================================================

  /// duration between consecutive checks when initialization is performed in seconds
  double init_check_duration_s{ 5.0 };

  /// translational offset of the UWB module w.r.t. the IMU (or body frame) in meter
  Eigen::Vector3d p_ItoU{ Eigen::VectorXd::Zero(3) };

  /// name of the pose topic used for current UAV position in global frame
  std::string topic_sub_pose{ "pose" };

  /// name of the uwb topic used to receive the UWB measurements
  std::string topic_sub_uwb{ "uwb" };

  /// name of the anchors topic used to publish the UWB anchor positions
  std::string topic_pub_anchors{ "anchors" };

  // UWB INITIALIZER ==========================================================

  /// maximum number of anchors used
  /// \deprecated this will be removed soon
  uint n_anchors{ 0 };

  /// buffer size of all buffers in s
  double buffer_size_s{ 10.0 };

  /// maximum condition number of the LS matrix for valid initialization
  double max_cond_num{ 100.0 };

};  // class UwbInitOptions
}  // namespace uav_init

#endif  // UAV_INIT_UWB_OPTIONS_HPP_
