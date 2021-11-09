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

#include <dynamic_reconfigure/server.h>
#include <evb1000_driver/TagDistance.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include <Eigen/Eigen>

#include "uav_init/uwb_init.hpp"
#include "utils/colors.hpp"
#include "uwb_init_cpp/UwbInitConfig.h"

namespace uav_init
{
class UwbInitWrapper
{
public:
  typedef uwb_init_cpp::UwbInitConfig UwbInitConfig_t;
  typedef dynamic_reconfigure::Server<UwbInitConfig_t> ReconfServer_t;
  /**
   * @brief Constructor
   * @param Ros NodeHandle
   */
  UwbInitWrapper(ros::NodeHandle& nh);

private:

  void cb_posestamped(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void cb_uwbstamped(const evb1000_driver::TagDistanceConstPtr& msg);
  void cb_dynamicconfig(UwbInitConfig_t& config, uint32_t level);

  /// Ros node handler
  ros::NodeHandle nh_;

  /// Subscribers
  ros::Subscriber sub_posestamped;
  ros::Subscriber sub_uwbstamped;

  // Initializer
  UwbInitializer uwb_initializer_;

  // Calibration and Parameters
  Eigen::Vector3d p_r_ItoU_;  //!< distance (r) from the IMU(body) to UWB frame expressed in IMU frame
  double p_min_dist{ 0.2 };   //!< minimal distance for measurements to be added (w.r.t. to last measurement)

  // dynamic reconfigure
  ReconfServer_t reconf_server_;
};

}  // namespace uav_init

#endif  // UAV_INIT_UWB_WRAPPER_HPP_
