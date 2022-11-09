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

#ifndef uwb_init_ros_H
#define uwb_init_ros_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <uwb_init>

class UwbInitRos
{
public:
  /**
   * @brief Constructor
   *
   * @param Ros NodeHandle
   */
  UwbInitRos(ros::NodeHandle& nh);

private:
  /**
   * @brief Callbacks
   *
   * @param Message const pointer
   */

  /// Ros node handler
  ros::NodeHandle nh_;

  /// Subscribers
  ros::Subscriber uwb_range_sub_;
  ros::Subscriber estimated_pose_sub_;

  /// Publishers

  /// Messages
};

#endif  // uwb_init_ros_H
