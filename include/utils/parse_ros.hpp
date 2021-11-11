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

  std::vector<double> p_ItoU;
  std::vector<double> p_ItoU_default = { 0.0, 0.0, 0.0 };
  nh.param<std::vector<double>>("p_ItoU", p_ItoU, p_ItoU_default);
  params.p_ItoU << p_ItoU.at(0), p_ItoU.at(1), p_ItoU.at(2);

  params.print_wrapper();

  // UWB INITIALIZER ==========================================================

  nh.param<double>("buffer_size_s", params.buffer_size_s, params.buffer_size_s);
  nh.param<double>("max_cond_num", params.max_cond_num, params.max_cond_num);

  int n_anchors;
  nh.param<int>("n_anchors", n_anchors, params.n_anchors);
  params.n_anchors = static_cast<uint>(n_anchors);

  params.print_initializer();

  // return
  return params;
}  // UwbInitOptions parse_ros_nodehandle(nh)
}  // namespace uav_init

#endif  // UAV_INIT_UTILS_PARSE_ROS_HPP_
