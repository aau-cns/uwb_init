// Copyright (C) 2021 Martin Scheiber,
// Control of Networked Systems, Universitaet Klagenfurt, Austria
//
// You can contact the author at <martin.scheiber@aau.at>
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

#ifndef UAV_INIT_UWB_WRAPPER_HPP_
#define UAV_INIT_UWB_WRAPPER_HPP_

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include <Eigen/Eigen>

#include "utils/colors.hpp"

namespace uav_init
{
class UwbInitWrapper
{
public:
  /**
   * @brief Constructor
   * @param Ros NodeHandle
   */
  UwbInitWrapper(ros::NodeHandle& nh);

private:
  /**
   * @brief Callbacks
   * @param Message const pointer
   */
  void callback_posestamped(const geometry_msgs::PoseStamped::ConstPtr& msg);

  /// Ros node handler
  ros::NodeHandle nh_;

  /// Subscribers
  ros::Subscriber sub_posestamped;

  /// Publishers

  /// Messages
};

}  // namespace uav_init

#endif  // UAV_INIT_UWB_WRAPPER_HPP_
