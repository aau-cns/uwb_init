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

#include "uwb_wrapper.hpp"

#include <mission_sequencer/MissionWaypoint.h>

#include "utils/logging.hpp"

namespace uav_init
{
UwbInitWrapper::UwbInitWrapper(ros::NodeHandle& nh, UwbInitOptions& params)
  : nh_(nh), params_(params), uwb_initializer_(params_)
{
  // subscribers
  sub_posestamped = nh.subscribe(params_.topic_sub_pose, 1, &UwbInitWrapper::cb_posestamped, this);
  sub_uwbstamped = nh.subscribe(params_.topic_sub_uwb, 1, &UwbInitWrapper::cb_uwbstamped, this);

  // publishers
  pub_anchor = nh.advertise<uwb_init_cpp::UwbAnchorArrayStamped>(params_.topic_pub_anchors, 1);
  pub_wplist = nh.advertise<mission_sequencer::MissionWaypointArray>(params_.topic_pub_wplist, 1);

  // set up dynamic parameters
  ReconfServer_t::CallbackType f = boost::bind(&UwbInitWrapper::cb_dynamicconfig, this, _1, _2);
  reconf_server_.setCallback(f);

  // print topic information
  uav_init::print_all_topics();

  // init anchor buffer
  anchor_buffer_.init(params_.buffer_size_s);

  // setup timer
  init_check_timer_ =
      nh.createTimer(ros::Duration(params_.init_check_duration_s), &uav_init::UwbInitWrapper::cb_timerinit, this);

  // setup waypoint list
  cur_waypoints_.action = mission_sequencer::MissionWaypointArray::CLEAR;
  cur_waypoints_.waypoints.clear();

}  // UwbInitWrapper::UwbInitWrapper(...)

void UwbInitWrapper::perform_initialization()
{
  // perform initialization here
  if (uwb_initializer_.try_to_initialize_anchors(anchor_buffer_))
  {
    INIT_INFO_STREAM("All UWB anchors successfully initialized");
    f_all_known_anchors_initialized_ = true;
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
  else if (f_all_known_anchors_initialized_)
  {
    INIT_DEBUG_STREAM("All anchors are already initialized, no need to calculate WPs!");
    INIT_WARN_STREAM("All anchors are initialized, going to origin!");

    mission_sequencer::MissionWaypoint wp;
    wp.x = 0.0;
    wp.y = 0.0;
    wp.z = params_.wp_height;
    wp.yaw = 0.0;
    wp.holdtime = 0.5;

    cur_waypoints_.waypoints.push_back(wp);
    return;
  }

  // get pose for each uninitialized anchor
  std::vector<Eigen::Vector3d> pos_uninitialized;
  pos_uninitialized.push_back(cur_p_IinG_);  // push current pose for trajectory gen later
  for (const auto& anchor : anchor_buffer_.get_buffer())
  {
    UwbAnchor anchor_value = anchor.second.get_buffer().back().second;
    if (!anchor_value.initialized)
      pos_uninitialized.push_back(anchor_value.p_AinG);
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
        wp.z = params_.wp_height;
        wp.yaw = 0.0;

        Eigen::Vector2d wpxy = (j * dist.norm() / num_wps) * dist.normalized();
        wp.x = pos_uninitialized.at(i).x() + wpxy.x();
        wp.y = pos_uninitialized.at(i).y() + wpxy.y();

        /// \todo TODO(scm): make the WP holdtime a parameter
        wp.holdtime = 0.5;

        cur_waypoints_.waypoints.push_back(wp);
      }
    }

    // push final value of unintialized wp
    mission_sequencer::MissionWaypoint wp;
    wp.z = params_.wp_height;
    wp.yaw = 0.0;
    wp.x = pos_uninitialized.at(pos_uninitialized.size()-1).x();
    wp.y = pos_uninitialized.at(pos_uninitialized.size()-1).y() ;
    wp.holdtime = 0.5;
    cur_waypoints_.waypoints.push_back(wp);
  }
  else
  {
    INIT_DEBUG_STREAM("All anchors initialized, no more waypoints needed");
    return;
  }

}  // namespace uav_init

void UwbInitWrapper::cb_posestamped(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
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

void UwbInitWrapper::cb_uwbstamped(const evb1000_driver::TagDistanceConstPtr& msg)
{
  // Convert measurement to correct format
  double time = msg->header.stamp.toSec();
  std::vector<UwbData> uwb_ranges;  // anchor_id (0,1,2,3,...), validity_flag, distance measurement

  // Fill the map with the
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

void UwbInitWrapper::cb_dynamicconfig(UwbInitConfig_t& config, uint32_t /*level*/)
{
  if (config.calculate)
  {
    // perform anchor claculation
    perform_initialization();

    config.calculate = false;
  }
}  // void UwbInitWrapper::cb_dynamicconfig(...)

void UwbInitWrapper::cb_timerinit(const ros::TimerEvent&)
{
  INIT_DEBUG_STREAM("UwbInitWrapper: timer event for initalization triggered");
  // only perform initialization if not all known anchors have been initialized
  if (!f_all_known_anchors_initialized_)
  {
    // perform init
    perform_initialization();

    // calc waypoints
    calculate_waypoints();

    // publish result
    uwb_init_cpp::UwbAnchorArrayStamped msg_anchors;
    /// \todo TODO(scm): make rostime now pub param
//    ros::Time pub_time = ros::Time::now();
    ros::Time pub_time = pub_stamp_;
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
        bool test_val = true; ros::param::get("/autonomy/hover_after_mission_completion", test_val);
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
}  // void UwbInitWrapper::cb_timerinit(...)

}  // namespace uav_init
