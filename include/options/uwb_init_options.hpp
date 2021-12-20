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

#ifndef UAV_INIT_UWB_OPTIONS_HPP_
#define UAV_INIT_UWB_OPTIONS_HPP_

#include <ros/ros.h>

#include <Eigen/Eigen>

#include "utils/logging.hpp"

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

  /// name of the waypoint list topic used to publish the next waypoints
  std::string topic_pub_wplist{ "waypoints" };

  void print_wrapper()
  {
    INIT_PRINT_STREAM("Parameter Summary -- Wrapper");
    INIT_PRINT_STREAM("\t- init_check_duration_s:       " << init_check_duration_s);
    INIT_PRINT_STREAM("\t- topic_sub_pose:              " << topic_sub_pose);
    INIT_PRINT_STREAM("\t- topic_sub_uwb:               " << topic_sub_uwb);
    INIT_PRINT_STREAM("\t- topic_pub_anchors:           " << topic_pub_anchors);
    INIT_PRINT_STREAM("\t- topic_pub_wplist:            " << topic_pub_wplist);
    INIT_PRINT_STREAM("\t- p_ItoU:                      " << p_ItoU.transpose());
  }

  // UWB INITIALIZER ==========================================================

  /// maximum number of anchors used
  /// \deprecated this will be removed soon
  uint n_anchors{ 0 };

  /// allow continous initialization although solution for anchor was already found
  bool f_do_continous_init_{ false };

  /// buffer size of all buffers in s
  double buffer_size_s{ 10.0 };

  /// maximum condition number of the LS matrix for valid initialization
  double max_cond_num{ 100.0 };

  /// minimum baseline in meter accepted to add obsevation to the double LS problem
  /// (z1^2 - z2^2 must be grater than meas_baseline_m_)
  double meas_baseline_m_{ 0.05 };

  /// indices baseline used for the double method
  uint meas_baseline_idx_{ 50 };

  /// Value of lamda used for regularization, if lambda = 0 no regularization is applied
  /// (suggested value 100)
  double lamda_{ 100 };

  /// time diference of poses to UWB measurements in s
  double t_pose_diff{ 0.0 };

  void print_initializer()
  {
    INIT_PRINT_STREAM("Parameter Summary -- Initializer");
    INIT_PRINT_STREAM("\t- n_anchors:                   " << n_anchors);
    INIT_PRINT_STREAM("\t- buffer_size_s:               " << buffer_size_s);
    INIT_PRINT_STREAM("\t- max_cond_num:                " << max_cond_num);
    INIT_PRINT_STREAM("\t- f_do_continous_init_:        " << f_do_continous_init_);
    INIT_PRINT_STREAM("\t- lamda_:                      " << lamda_);
    // TODO(alf): Add if condition to print these only if we use double
    INIT_PRINT_STREAM("\t- meas_baseline_m_:            " << meas_baseline_m_);
    INIT_PRINT_STREAM("\t- meas_baseline_idx_:          " << meas_baseline_idx_);
  }

  // WAYPOINT GENERATION ======================================================

  /// maximum distance between two waypoints generated
  double wp_generation_max_distance{ 1.0 };

  /// height of waypoints for initialization purpose
  double wp_height{ 2.0 };

  void print_waypoint()
  {
    INIT_PRINT_STREAM("Parameter Summary -- Initializer");
    INIT_PRINT_STREAM("\t- wp_generation_max_distance:  " << wp_generation_max_distance);
    INIT_PRINT_STREAM("\t- wp_height:                   " << wp_height);
  }

};  // class UwbInitOptions
}  // namespace uav_init

#endif  // UAV_INIT_UWB_OPTIONS_HPP_
