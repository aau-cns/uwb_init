// Copyright (C) 2022 Alessandro Fornasier,
// Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.
//
// You can contact the author at <alessandro.fornasier@aau.at>

#include "uwb_init_ros.hpp"
#include <ros/ros.h>
#include <Eigen/Eigen>

namespace uwb_init_ros
{
UwbInitRos::UwbInitRos(const ros::NodeHandle& nh, const UwbInitRosOptions& options)
  : nh_(nh)
  , options_(options)
  , uwb_init_(options.level_, options.init_options_, options.ls_solver_options_, options.nls_solver_options_)
{
  // Subscribers
  estimated_pose_sub_ = nh_.subscribe(options_.estimated_pose_topic_, 1, &UwbInitRos::callback_pose, this);
  //   uwb_range_sub_ = nh_.subscribe(options_.uwb_range_topic_, 1, &UwbInitRos::callback_uwb, this);

  // Print topics where we are subscribing to
  ROS_INFO("Subsribing to %s", estimated_pose_sub_.getTopic().c_str());

  // Publishers

  // Print topics where we are publishing on
  // ROS_INFO("Publishing in %s");
}

void UwbInitRos::callback_pose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  // Get pose
  Eigen::Vector3d p_IinG(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  Eigen::Quaterniond q_GI(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y,
                          msg->pose.orientation.z);

  // Compute position of UWB module in Global frame
  Eigen::Vector3d p_UinG = p_IinG + q_GI.toRotationMatrix() * p_IinG;

  // Feed p_UinG
  uwb_init_.feed_position(msg->header.stamp.toSec(), p_UinG);
}
}  // namespace uwb_init_ros
