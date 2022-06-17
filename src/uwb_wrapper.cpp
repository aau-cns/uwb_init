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
// You can contact the author at <martin.scheiber@aau.at>,
// <alessandro.fornasier@aau.at>

#include <mission_sequencer/MissionWaypoint.h>

#include "uwb_wrapper.hpp"
#include "utils/logging.hpp"
#include "utils/utils.h"

namespace uav_init
{

UwbInitWrapper::UwbInitWrapper(ros::NodeHandle& nh, UwbInitOptions& params)
  : nh_(nh), params_(params), uwb_initializer_(params_), f_in_initialization_phase_(params_.b_auto_init)
{
  // subscribers
  sub_posestamped = nh.subscribe(params_.topic_sub_pose, 1, &UwbInitWrapper::cbPoseStamped, this);
  sub_uwbstamped = nh.subscribe(params_.topic_sub_uwb, 1, &UwbInitWrapper::cbUwbStamped, this);

  // publishers
  pub_anchor = nh.advertise<uwb_init_cpp::UwbAnchorArrayStamped>(params_.topic_pub_anchors, 1);
  pub_wplist = nh.advertise<mission_sequencer::MissionWaypointArray>(params_.topic_pub_wplist, 1);
  anchor_publisher_switch_ = !params_.publish_only_when_all_initialized;

  // service clients
  srvc_sequencer_get_start_pose_ =
      nh_.serviceClient<mission_sequencer::GetStartPose>(params_.service_ms_get_start_pose);

  // service servers
  srvs_start_init_ = nh_.advertiseService(params_.service_start_init, &UwbInitWrapper::cbSrvInit, this);

  // set up dynamic parameters
  ReconfServer_t::CallbackType f = boost::bind(&UwbInitWrapper::cbDynamicConfig, this, _1, _2);
  reconf_server_.setCallback(f);

  // print topic information
  uav_init::print_all_topics();

  // init anchor buffer
  anchor_buffer_.init(params_.buffer_size_s);

  // setup timer
  init_check_timer_ =
      nh.createTimer(ros::Duration(params_.init_check_duration_s), &uav_init::UwbInitWrapper::cbTimerInit, this);

  // setup waypoint list
  //  cur_waypoints_.action = mission_sequencer::MissionWaypointArray::CLEAR;
  cur_waypoints_.action = mission_sequencer::MissionWaypointArray::INSERT;
  cur_waypoints_.idx = 0;
  cur_waypoints_.waypoints.clear();

}  // UwbInitWrapper::UwbInitWrapper(...)


void UwbInitWrapper::resetWrapper()
{
  // set init flag to false
  f_in_initialization_phase_ = false;

  // reset buffers
  anchor_buffer_.reset();

  // reset initializer
  uwb_initializer_.reset();
}


void UwbInitWrapper::perform_initialization()
{
  // perform initialization here
  if (uwb_initializer_.try_to_initialize_anchors(anchor_buffer_))
  {
    INIT_INFO_STREAM("All UWB anchors successfully initialized");
    f_all_known_anchors_initialized_ = true;
    anchor_publisher_switch_ = true;
  }
  else
  {
    INIT_ERROR_STREAM("Could not initialize all UWB anchors");
  }

  if (anchor_buffer_.get_buffer().empty())
  {
    INIT_DEBUG_STREAM("No anchors yet...");
    return;
  }
  else
  {
    // output result
    INIT_INFO_STREAM("Initialization Result:");
#if (__cplusplus >= 201703L)
    for (const auto& [anchor_id, anchor_values] : anchor_buffer_.get_buffer())
    {
#else
    for (const auto& kv : anchor_buffer_.get_buffer())
    {
      const auto anchor_id = kv.first;
      const auto anchor_values = kv.second;
#endif
      const auto anchor_value = anchor_values.get_buffer().back().second;
      INIT_INFO_STREAM("\tAnchor " << anchor_id << ": " << anchor_value.initialized);
      INIT_INFO_STREAM("\t\tpos   : " << anchor_value.p_AinG.transpose());
      INIT_INFO_STREAM("\t\td_bias: " << anchor_value.bias_d);
      INIT_INFO_STREAM("\t\tc_bias: " << anchor_value.bias_c << std::endl);
    }
  }
}  // void UwbInitWrapper::perform_initialization()


void UwbInitWrapper::calculate_waypoints()
{
  // clear current waypoint list
  cur_waypoints_.waypoints.clear();

  // check if anchor buffer is empty
  if (anchor_buffer_.get_buffer().empty())
  {
    INIT_DEBUG_STREAM("No anchors to calculate waypoints yet.");
    return;
  }

  // try to get navigation height
  double zero_height = 0.0, zero_yaw = 0.0;
  if (srvc_sequencer_get_start_pose_.exists())
  {
    // checked if service exists, if not we will not add anything here
    mission_sequencer::GetStartPose srv;

    // call service
    if (srvc_sequencer_get_start_pose_.call(srv))
    {
      zero_height = srv.response.start_wp.z;
      zero_yaw = srv.response.start_wp.yaw;
      INIT_DEBUG_STREAM("Got getStartPose service response from mission sequencer with height "
                        << zero_height << " and yaw " << zero_yaw << ".");
    }
    else
    {
      INIT_WARN_STREAM("Did not retreive valid Pose from '" << params_.service_ms_get_start_pose
                                                            << ". Setting local takoff height to " << zero_height
                                                            << ".");
    }
  }
  else
    INIT_WARN_STREAM("Could not connect to getStartPose service");

  if (f_all_known_anchors_initialized_)
  {
    INIT_DEBUG_STREAM("All anchors are already initialized, no need to calculate WPs!");
    INIT_WARN_STREAM("All anchors are initialized, going to origin!");

    // add origin waypoint
    mission_sequencer::MissionWaypoint wp;
    wp.x = 0.0;
    wp.y = 0.0;
    wp.z = params_.wp_height + zero_height;
    wp.yaw = zero_yaw;
    wp.holdtime = params_.wp_holdtime;
    cur_waypoints_.waypoints.push_back(wp);

    // set CLEAR command
    cur_waypoints_.action = mission_sequencer::MissionWaypointArray::CLEAR;
    return;
  }

  // get pose for each uninitialized anchor
  std::vector<Eigen::Vector3d> pos_uninitialized;
  pos_uninitialized.push_back(cur_p_IinG_);  // push current pose for trajectory gen later
  for (const auto& anchor : anchor_buffer_.get_buffer())
  {
    UwbAnchor anchor_value = anchor.second.get_buffer().back().second;
    if (!anchor_value.initialized)
      pos_uninitialized.push_back(anchor_value.p_AinG +
                                  params_.wp_rand_offset / 5.0 *
                                      Eigen::Vector3d(randomizer_.get_randi() - 5.0, randomizer_.get_randi() - 5.0, 0));
  }

  // generate trajectory through all poses
  // METHOD 1 just fly to all of them in straight lines
  if (pos_uninitialized.size() > 1)
  {
    for (uint i = 0; i < pos_uninitialized.size() - 1; ++i)
    {
      // calculate distance between wps
      Eigen::Vector2d dist(pos_uninitialized.at(i + 1).x() - pos_uninitialized.at(i).x(),
                           pos_uninitialized.at(i + 1).y() - pos_uninitialized.at(i).y());
      // calculate number of wps needed
      double num_wps = std::ceil(dist.norm() / params_.wp_generation_max_distance);

      // in case num_wps is 0, we are ontop of the current wps, so do not add anything
      for (uint j = 0; j < (uint)num_wps; ++j)
      {
        mission_sequencer::MissionWaypoint wp;
        wp.z = params_.wp_height + zero_height;
        wp.yaw = zero_yaw;

        Eigen::Vector2d wpxy = (j * dist.norm() / num_wps) * dist.normalized();
        wp.x = pos_uninitialized.at(i).x() + wpxy.x();
        wp.y = pos_uninitialized.at(i).y() + wpxy.y();
        wp.holdtime = params_.wp_holdtime;

        cur_waypoints_.waypoints.push_back(wp);
      }
    }

    // push final value of unintialized wp
    mission_sequencer::MissionWaypoint wp;
    wp.z = params_.wp_height + zero_height;
    wp.yaw = zero_yaw;
    wp.x = pos_uninitialized.at(pos_uninitialized.size() - 1).x();
    wp.y = pos_uninitialized.at(pos_uninitialized.size() - 1).y();
    wp.holdtime = params_.wp_holdtime;
    cur_waypoints_.waypoints.push_back(wp);
  }
  else
  {
    INIT_DEBUG_STREAM("All anchors initialized, no more waypoints needed");
    return;
  }

}  // void UwbInitWrapper::calculate_waypoints()


void UwbInitWrapper::cbPoseStamped(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  // check if initialization is currently allowed
  if (!f_in_initialization_phase_)
    return;

  // calculate position of UWB module in G frame
  cur_p_IinG_ = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  Eigen::Quaterniond q_IinG(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y,
                            msg->pose.orientation.z);
  Eigen::Vector3d p_UinG = cur_p_IinG_ + q_IinG.toRotationMatrix() * params_.p_ItoU;

  // feed current pose to initializer
  uwb_initializer_.feed_pose(msg->header.stamp.toSec(), p_UinG);

  // update publishing timestamp to use latest pose time
  pub_stamp_ = msg->header.stamp;
}  // void UwbInitWrapper::cb_posestamped(...)


#if UWB_DRIVER == EVB_DRIVER
  void UwbInitWrapper::cbUwbStamped(const evb1000_driver::TagDistanceConstPtr& msg)
  {
    // check if initialization is currently allowed
    if (!f_in_initialization_phase_)
      return;

    // Convert measurement to correct format
    double time = msg->header.stamp.toSec();
    std::vector<UwbData> uwb_ranges;  // anchor_id, validity_flag, distance measurement

    // Fill the uwb data structure
    // NOTE(alf): No ID information is provided with the measurements therefore assign ordered IDs
    uint n = msg->valid.size();
    for (uint i = 0; i < n; ++i)
    {
      uwb_ranges.push_back(UwbData(time, msg->valid[i], msg->distance[i], i));

      // check if new anchor was added and reset flag if the id is not known yet
      f_all_known_anchors_initialized_ &= !anchor_buffer_.contains_id(i);
    }

    // feed measurements to initializer
    uwb_initializer_.feed_uwb(uwb_ranges);
  }  // void UwbInitWrapper::cb_uwbstamped(...)
#else
  void UwbInitWrapper::cbUwbStamped(const mdek_uwb_driver::UwbConstPtr& msg)
  {
    // check if initialization is currently allowed
    if (!f_in_initialization_phase_)
      return;

    // Convert measurement to correct format
    double time = msg->header.stamp.toSec();
    std::vector<UwbData> uwb_ranges;  // anchor_id, validity_flag, distance measurement

    // Fill the uwb data structure
    for (const auto &it : msg->ranges)
    {
      // Check if the id is valid (contains only numbers)
      if (!containsChar(it.id)) {

        // Convert string id to size_t
        std::stringstream sstream(it.id);
        uint id;
        sstream >> id;

        // Fill vector
        // TODO(alf): Check if there is a way to infer validy based on information given
        uwb_ranges.emplace_back(UwbData(time, true, it.distance, static_cast<u_int16_t>(id)));

        // check if new anchor was added and reset flag if the id is not known yet
        f_all_known_anchors_initialized_ &= !anchor_buffer_.contains_id(id);

      } else {
        INIT_WARN_STREAM("Received UWB message with characters within the id field");
      }

    }

    // feed measurements to initializer
    uwb_initializer_.feed_uwb(uwb_ranges);
  }  // void UwbInitWrapper::cb_uwbstamped(...)
#endif


void UwbInitWrapper::cbDynamicConfig(UwbInitConfig_t& config, uint32_t /*level*/)
{
  if (config.calculate)
  {
    // perform anchor claculation
    perform_initialization();

    config.calculate = false;
  }
}  // void UwbInitWrapper::cb_dynamicconfig(...)


void UwbInitWrapper::cbTimerInit(const ros::TimerEvent&)
{
  INIT_DEBUG_STREAM("UwbInitWrapper: timer event for initalization triggered");
  // check if initialization is currently allowed
  if (!f_in_initialization_phase_)
  {
    INIT_DEBUG_STREAM("initialization currently disallowed");
    return;
  }

  // only perform initialization if not all known anchors have been initialized
  if (!f_all_known_anchors_initialized_)
  {
    // perform init
    perform_initialization();

    // calc waypoints
    calculate_waypoints();

    // Define pub time
    /// \todo TODO(scm): make rostime now pub param
    //    ros::Time pub_time = ros::Time::now();
    ros::Time pub_time = pub_stamp_;

    // publish result if allowed
    if(anchor_publisher_switch_)
    {
      uwb_init_cpp::UwbAnchorArrayStamped msg_anchors;
      msg_anchors.header.stamp = pub_time;
      msg_anchors.header.frame_id = "global";
      msg_anchors.header.seq = pub_anchor_seq_++;

      // retreive anchor data
      for (const auto& anchor : anchor_buffer_.get_buffer())
      {
        uwb_init_cpp::UwbAnchor msg_anchor;
        UwbAnchor data_anchor = anchor.second.get_buffer().back().second;

        msg_anchor.id = data_anchor.id;
        msg_anchor.position.x = data_anchor.p_AinG.x();
        msg_anchor.position.y = data_anchor.p_AinG.y();
        msg_anchor.position.z = data_anchor.p_AinG.z();
        msg_anchor.bias_distance = data_anchor.bias_d;
        msg_anchor.bias_const = data_anchor.bias_c;
        msg_anchor.initialized = data_anchor.initialized;

        msg_anchors.anchors.push_back(msg_anchor);
      }

      // set size and publish
      msg_anchors.size = msg_anchors.anchors.size();
      pub_anchor.publish(msg_anchors);
    }

    // publish waypoint data
    if (!cur_waypoints_.waypoints.empty())
    {
      cur_waypoints_.header.stamp = pub_time;
      cur_waypoints_.header.seq = pub_waypoint_seq_++;

      if (f_all_known_anchors_initialized_)
      {
        cur_waypoints_.header.frame_id = "local";
        cur_waypoints_.is_global = false;
        pub_wplist.publish(cur_waypoints_);

        // tell the autonomy that the next time it receives a mission complete it should land
        /// \todo TODO(scm): maybe make this string for the parameter a parameter?
        ros::param::set("/autonomy/hover_after_mission_completion", false);
        bool test_val = true;
        ros::param::get("/autonomy/hover_after_mission_completion", test_val);
        INIT_DEBUG_STREAM("Set '/autonomy/hover_after_mission_completion' parameter to " << test_val);
      }
      else
      {
        cur_waypoints_.header.frame_id = "global";
        cur_waypoints_.is_global = true;
        pub_wplist.publish(cur_waypoints_);
      }
    }
  }  // if (!f_all_known_anchors_initialized_)
  else
  {
    // exit if flag is set
    if (params_.b_exit_when_initialized)
    {
      INIT_INFO_STREAM("concluded initialization!");
      INIT_ERROR_STREAM("exiting successfully . . .");
      std::exit(EXIT_SUCCESS);
    }
  }
}  // void UwbInitWrapper::cb_timerinit(...)


bool UwbInitWrapper::cbSrvInit(std_srvs::Empty::Request& /*req*/, std_srvs::Empty::Response& /*res*/)
{
  INIT_DEBUG_STREAM("startService called: starting/reseting initialization");

  // reset buffers (in case of new initialization call
  resetWrapper();

  // allow initialization
  f_in_initialization_phase_ = true;
  return f_in_initialization_phase_;
}

}  // namespace uav_init
