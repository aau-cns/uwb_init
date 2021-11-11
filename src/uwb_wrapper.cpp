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

void UwbInitWrapper::cb_posestamped(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  // calculate position of UWB module in G frame
  Eigen::Vector3d p_UinG(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  Eigen::Quaterniond q_UinG(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y,
                            msg->pose.orientation.z);
  p_UinG = p_UinG + q_UinG.toRotationMatrix() * params_.p_ItoU;

  // feed current pose to initializer
  uwb_initializer_.feed_pose(msg->header.stamp.toSec(), p_UinG);
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

void UwbInitWrapper::cb_dynamicconfig(UwbInitConfig_t& config, uint32_t level)
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
  perform_initialization();

  // publish result
  uwb_init_cpp::UwbAnchorArrayStamped msg_anchors;
  msg_anchors.header.stamp = ros::Time::now();
  msg_anchors.header.frame_id = "global";
  msg_anchors.header.seq = pub_anchor_seq_++;

  // retreive anchor data
  for (const auto& anchor : anchor_buffer_.get_buffer())
  {
    uwb_init_cpp::UwbAnchor msg_anchor;
    UwbAnchor data_anchor = anchor.second.get_buffer().back().second;

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
}  // void UwbInitWrapper::cb_timerinit(...)

}  // namespace uav_init
