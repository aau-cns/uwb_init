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
#include <evb1000_driver/TagDistance.h>
#include <ros/ros.h>

#include <Eigen/Eigen>

#include "utils/colors.hpp"
#include "uav_init/uwb_init.hpp"

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
  void cb_posestamped(const geometry_msgs::PoseStamped::ConstPtr& msg);

  void cb_uwbstamped(const evb1000_driver::TagDistanceConstPtr& msg);

  /// Ros node handler
  ros::NodeHandle nh_;

  /// Subscribers
  ros::Subscriber sub_posestamped;
  ros::Subscriber sub_uwbstamped;

  // Initializer
  UwbInitializer uwb_initializer_;

  // Calibration and Parameters
  Eigen::Vector3d p_r_ItoU_;  //!< distance (r) from the IMU(body) to UWB frame expressed in IMU frame
};

}  // namespace uav_init

#endif  // UAV_INIT_UWB_WRAPPER_HPP_
