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

#ifndef UWB_INIT_ROS_H
#define UWB_INIT_ROS_H

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <uwb_init_lib/include/uwb_init.hpp>

#include "utils/options.hpp"

namespace uwb_init_ros
{
class UwbInitRos
{
public:
  /**
   * @brief Constructor
   *
   * @param Ros NodeHandle
   */
  UwbInitRos(const ros::NodeHandle& nh, const UwbInitRosOptions& options);

private:
  /**
   * @brief Pose callback
   *
   * @param Message
   */
  void callback_pose(const geometry_msgs::PoseStamped::ConstPtr& msg);

  /// Ros node handler
  ros::NodeHandle nh_;

  /// Subscribers
  ros::Subscriber uwb_range_sub_;
  ros::Subscriber estimated_pose_sub_;

  /// Options
  UwbInitRosOptions options_;

  /// Publishers

  /// Messages

  /// UWB initializer
  uwb_init::UwbInitializer uwb_init_;
};
}  // namespace uwb_init_ros

#endif  // UWB_INIT_ROS_H
