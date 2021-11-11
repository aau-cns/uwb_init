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

#ifndef UAV_INIT_UWB_WRAPPER_HPP_
#define UAV_INIT_UWB_WRAPPER_HPP_

#include <dynamic_reconfigure/server.h>
#include <evb1000_driver/TagDistance.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include <Eigen/Eigen>

#include "types/types.hpp"
#include "uav_init/uwb_init.hpp"
#include "utils/colors.hpp"
#include "uwb_init_cpp/UwbInitConfig.h"
#include "options/uwb_init_options.hpp"

namespace uav_init
{
class UwbInitWrapper
{
public:
  /// dynamic reconfigure config typedef
  typedef uwb_init_cpp::UwbInitConfig UwbInitConfig_t;
  /// dynamic recofnigure server typedef
  typedef dynamic_reconfigure::Server<UwbInitConfig_t> ReconfServer_t;

  ///
  /// \brief UwbInitWrapper default constructor for UwbInitWrapper
  /// \param nh ros nodehandle
  ///
  UwbInitWrapper(ros::NodeHandle& nh, UwbInitOptions& params);

  void cb_timerinit(const ros::TimerEvent&);

private:
  void cb_posestamped(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void cb_uwbstamped(const evb1000_driver::TagDistanceConstPtr& msg);
  void cb_dynamicconfig(UwbInitConfig_t& config, uint32_t level);

  ///
  /// \brief perform_initialization tries to initialize all anchors, for which messages were received
  ///
  void perform_initialization();

  // Ros node handler
  ros::NodeHandle nh_;  //!< ROS nodehandle given through constructor

  // UwbInit parameters
  UwbInitOptions params_;  //!< launched parameter options

  // Subscribers
  ros::Subscriber sub_posestamped;  //!< ROS subscriber for poses of IMU (or body) in global frame
  ros::Subscriber sub_uwbstamped;   //!< ROS subscirber for UWB distance measurements

  // Publishers
  ros::Publisher pub_wplist;  //!< ROS publisher for wp list

  // dynamic reconfigure
  ReconfServer_t reconf_server_; //!< dynamic reconfigure server for ROS dynamic reconfigure

  // Initializer
  UwbInitializer uwb_initializer_;        //!< initializer class for UWB modules
  UwbAnchorBuffer anchor_buffer_;         //!< buffer containing the anchor calculated positions
  bool f_all_known_anchors_initialized_;  //!< flag determining if all currently known anchors are initialized

  // timer variables
  ros::Timer init_check_timer_;  //!< timer used to check and perform initialization
};

}  // namespace uav_init

#endif  // UAV_INIT_UWB_WRAPPER_HPP_
