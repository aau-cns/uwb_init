// Copyright (C) 2021 Martin Scheiber, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <martin.scheiber@aau.at>

#ifndef UAV_INIT_TYPES_UWB_ANCHOR_HPP_
#define UAV_INIT_TYPES_UWB_ANCHOR_HPP_

#include <Eigen/Eigen>

namespace uav_init
{
struct UwbAnchor
{
  Eigen::Vector3d p_AinG{ Eigen::VectorXd::Zero(3) };
  double bias_d{ 0.0 };
  double bias_c{ 0.0 };

  Eigen::Matrix3d cov_p_AinG{ Eigen::MatrixXd::Zero(3, 3) };
  double cov_bias_d{ 0.0 };
  double cov_bias_c{ 0.0 };

};  // struct UwbAnchor
}  // namespace uav_init

#endif  // UAV_INIT_TYPES_UWB_ANCHOR_HPP_
