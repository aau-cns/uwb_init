// Copyright (C) 2022 Martin Scheiber, Alessandro Fornasier,
// Control of Networked Systems, University of Klagenfurt, Austria.
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
// <alessandro.fornasier@aau.at>

#ifndef UAV_INIT_UWB_WRAPPER_HPP_
#define UAV_INIT_UWB_WRAPPER_HPP_

#define MDEK_DRIVER 1
#define EVB_DRIVER 2

#define UWB_DRIVER EVB_DRIVER

#ifndef UWB_DRIVER
  #define UWB_DRIVER MDEK_DRIVER
#endif

#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <mission_sequencer/GetStartPose.h>
#include <mission_sequencer/MissionWaypointArray.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <Eigen/Eigen>

#if UWB_DRIVER == EVB_DRIVER
  #include <evb1000_driver/TagDistance.h>
#else
  #include <mdek_uwb_driver/Uwb.h>
#endif

#include "options/uwb_init_options.hpp"
#include "types/types.hpp"
#include "uav_init/uwb_init.hpp"
#include "utils/colors.hpp"
#include "uwb_init_cpp/UwbAnchorArrayStamped.h"
#include "uwb_init_cpp/UwbInitConfig.h"

namespace uav_init
{
///
/// \brief The UwbInitWrapper class is a ROS wrapper for the UwbInitializer
///
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
  /// \param params node parameters set by the launchfile
  ///
  UwbInitWrapper(ros::NodeHandle& nh, UwbInitOptions& params);

  void cbTimerInit(const ros::TimerEvent&);

private:

  void cbPoseStamped(const geometry_msgs::PoseStamped::ConstPtr& msg);

  #if UWB_DRIVER == EVB_DRIVER
    void cbUwbStamped(const evb1000_driver::TagDistanceConstPtr& msg);
  #else
    void cbUwbStamped(const mdek_uwb_driver::UwbConstPtr& msg);
  #endif

  void cbDynamicConfig(UwbInitConfig_t& config, uint32_t level);

  bool cbSrvInit(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  ///
  /// \brief perform_initialization tries to initialize all anchors, for which messages were received
  ///
  void perform_initialization();

  ///
  /// \brief calculate_waypoints calculates a list of waypoints to fly to such that the anchors are initialized better
  ///
  void calculate_waypoints();

  ///
  /// \brief resetWrapper method to reset all buffers of the wrapper and init
  ///
  void resetWrapper();

  // Ros node handler
  ros::NodeHandle nh_;  //!< ROS nodehandle given through constructor

  // UwbInit parameters
  UwbInitOptions params_;  //!< launched parameter options

  // Subscribers
  ros::Subscriber sub_posestamped;  //!< ROS subscriber for poses of IMU (or body) in global frame
  ros::Subscriber sub_uwbstamped;   //!< ROS subscirber for UWB distance measurements

  // Publishers
  ros::Publisher pub_anchor;  //!< ROS publisher for anchor position and biases
  ros::Publisher pub_wplist;  //!< ROS publisher for wp list

  // Service Clients
  ros::ServiceClient srvc_sequencer_get_start_pose_;  //!< ROS service client to get the start pose for navigation

  // Service Servers
  ros::ServiceServer srvs_start_init_;  //!< ROS service server to start the initialization phase

  // publishing variables
  uint pub_anchor_seq_{ 0 };                 //!< sequence number of published anchor msgs
  uint pub_waypoint_seq_{ 0 };               //!< sequence number of published waypoint list msgs
  ros::Time pub_stamp_{ ros::Time::now() };  //!< timestamp used when publishing
  bool anchor_publisher_switch_{false};      //!< switch to allow publication of initialized anchors

  // dynamic reconfigure
  ReconfServer_t reconf_server_;  //!< dynamic reconfigure server for ROS dynamic reconfigure

  // Initializer
  UwbInitializer uwb_initializer_;                 //!< initializer class for UWB modules
  UwbAnchorBuffer anchor_buffer_;                  //!< buffer containing the anchor calculated positions
  bool f_all_known_anchors_initialized_{ false };  //!< flag determining if all currently known anchors are initialized
  bool f_in_initialization_phase_{ false };  //!< flag determinig if initialization is currently allowed to be performed

  // waypoint publisher
  Eigen::Vector3d cur_p_IinG_;                             //!< current position of the vehicle in the global frame
  mission_sequencer::MissionWaypointArray cur_waypoints_;  //!< current/next waypoints for the mission_sequencer
  Randomizer randomizer_{ 0, 10 };                         //!< struct used to get random numbers

  // timer variables
  ros::Timer init_check_timer_;  //!< timer used to check and perform initialization
};

}  // namespace uav_init

#endif  // UAV_INIT_UWB_WRAPPER_HPP_
