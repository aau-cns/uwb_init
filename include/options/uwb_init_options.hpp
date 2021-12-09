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

  void print_wrapper()
  {
    INIT_PRINT_STREAM("Parameter Summary -- Wrapper");
    INIT_PRINT_STREAM("\t- init_check_duration_s:  " << init_check_duration_s);
    INIT_PRINT_STREAM("\t- topic_sub_pose:         " << topic_sub_pose);
    INIT_PRINT_STREAM("\t- topic_sub_uwb:          " << topic_sub_uwb);
    INIT_PRINT_STREAM("\t- topic_pub_anchors:      " << topic_pub_anchors);
    INIT_PRINT_STREAM("\t- p_ItoU:                 " << p_ItoU.transpose());
  }

  // UWB INITIALIZER ==========================================================

  /// maximum number of anchors used
  /// \deprecated this will be removed soon
  uint n_anchors{ 0 };

  /// buffer size of all buffers in s
  double buffer_size_s{ 10.0 };

  /// maximum condition number of the LS matrix for valid initialization
  double max_cond_num{ 100.0 };

  void print_initializer()
  {
    INIT_PRINT_STREAM("Parameter Summary -- Initializer");
    INIT_PRINT_STREAM("\t- n_anchors:              " << n_anchors);
    INIT_PRINT_STREAM("\t- buffer_size_s:          " << buffer_size_s);
    INIT_PRINT_STREAM("\t- max_cond_num:           " << max_cond_num);
  }

  // WAYPOINT GENERATION ======================================================

  /// maximum distance between two waypoints generated
  double wp_generation_max_distance{ 1.0 };

  /// height of waypoints for initialization purpose
  double wp_height{ 2.0 };

};  // class UwbInitOptions
}  // namespace uav_init

#endif  // UAV_INIT_UWB_OPTIONS_HPP_
