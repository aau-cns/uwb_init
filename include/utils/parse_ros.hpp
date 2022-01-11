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

#ifndef UAV_INIT_UTILS_PARSE_ROS_HPP_
#define UAV_INIT_UTILS_PARSE_ROS_HPP_

#include <ros/ros.h>

#include "options/uwb_init_options.hpp"

namespace uav_init
{
UwbInitOptions parse_ros_nodehandle(ros::NodeHandle& nh)
{
  UwbInitOptions params;

  // WRAPPER AND CALIBRATION ==================================================

  nh.param<double>("init_check_duration", params.init_check_duration_s, params.init_check_duration_s);
  nh.param<std::string>("topic_sub_pose", params.topic_sub_pose, params.topic_sub_pose);
  nh.param<std::string>("topic_sub_uwb", params.topic_sub_uwb, params.topic_sub_uwb);
  nh.param<std::string>("topic_pub_anchors", params.topic_pub_anchors, params.topic_pub_anchors);
  nh.param<std::string>("topic_pub_waypoints", params.topic_pub_wplist, params.topic_pub_wplist);

  std::vector<double> p_ItoU;
  std::vector<double> p_ItoU_default = { 0.0, 0.0, 0.0 };
  nh.param<std::vector<double>>("p_ItoU", p_ItoU, p_ItoU_default);
  params.p_ItoU << p_ItoU.at(0), p_ItoU.at(1), p_ItoU.at(2);

  params.print_wrapper();

  // UWB INITIALIZER ==========================================================

  nh.param<double>("buffer_size_s", params.buffer_size_s, params.buffer_size_s);
  nh.param<double>("max_cond_num", params.max_cond_num, params.max_cond_num);
  nh.param<bool>("do_continous_init", params.f_do_continous_init_, params.f_do_continous_init_);
  nh.param<double>("meas_baseline_m", params.meas_baseline_m_, params.meas_baseline_m_);
  nh.param<double>("reg_lambda", params.lamda_, params.lamda_);
  nh.param<double>("cov_sv_threshold", params.cov_sv_threshold_, params.cov_sv_threshold_);
  nh.param<double>("t_pose_diff_s", params.t_pose_diff, params.t_pose_diff);

  int n_anchors;
  nh.param<int>("n_anchors", n_anchors, params.n_anchors);
  params.n_anchors = static_cast<uint>(n_anchors);

  int meas_baseline_idx;
  nh.param<int>("meas_baseline_idx", meas_baseline_idx, params.meas_baseline_idx_);
  params.meas_baseline_idx_ = static_cast<uint>(meas_baseline_idx);

  std::string init_method, init_variables;


  params.print_initializer();

  // WAYPOINT GENERATION ======================================================

  nh.param<double>("waypoint_max_dist_m", params.wp_generation_max_distance, params.wp_generation_max_distance);
  nh.param<double>("waypoint_height_m", params.wp_height, params.wp_height);

  params.print_waypoint();

  // return
  return params;
}  // UwbInitOptions parse_ros_nodehandle(nh)
}  // namespace uav_init

#endif  // UAV_INIT_UTILS_PARSE_ROS_HPP_
