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

#include "uwb_wrapper.hpp"

namespace uav_init
{
UwbInitWrapper::UwbInitWrapper(ros::NodeHandle& nh) : nh_(nh)
{
  // read parameters
  std::vector<double> r_ItoU;
  std::vector<double> r_ItoU_default = { 0.0, 0.0, 0.0 };
  nh.param<std::vector<double>>("r_ItoU", r_ItoU, r_ItoU_default);
  p_r_ItoU_ << r_ItoU.at(0), r_ItoU.at(1), r_ItoU.at(2);

  int n_anchors;
  if (!nh.param<int>("n_anchors", n_anchors, 0))
  {
    ROS_ERROR_STREAM("No number of anchors in use give. Exiting...");
    std::exit(EXIT_FAILURE);
  }

  if (!nh.param<double>("buffer_size_s", p_buffer_size_s_, 10.0))
  {
    ROS_WARN_STREAM("No buffer size give, using 10.0s");
  }

  // subscribers
  sub_posestamped = nh.subscribe("pose", 1, &UwbInitWrapper::cb_posestamped, this);
  sub_uwbstamped = nh.subscribe("uwb", 1, &UwbInitWrapper::cb_uwbstamped, this);

  // publishers

  // set up dynamic parameters
  ReconfServer_t::CallbackType f = boost::bind(&UwbInitWrapper::cb_dynamicconfig, this, _1, _2);
  reconf_server_.setCallback(f);

  // Print topics where we are subscribing to
  std::cout << std::endl;
  std::cout << "Subscribing: " << sub_posestamped.getTopic().c_str() << std::endl;
  std::cout << "Subscribing: " << sub_uwbstamped.getTopic().c_str() << std::endl;

  // create initializer
  uwb_initializer_ = UwbInitializer(n_anchors);

  // init anchor buffer
  anchor_buffer_.init(p_buffer_size_s_);
}

void UwbInitWrapper::perform_initialization()
{
  // perform initialization here
  if (uwb_initializer_.try_to_initialize_anchors(anchor_buffer_))
  {
    ROS_INFO_STREAM("All UWB anchors successfully initialized");
    f_all_known_anchors_initialized_ = true;
  }
  else
  {
    ROS_ERROR_STREAM("Could not initialize all UWB anchors");
  }

  if (anchor_buffer_.get_buffer().empty())
  {
    ROS_DEBUG_STREAM("No anchors yet...");
    return;
  }
  else
  {
    // output result
    ROS_INFO_STREAM("Initialization Result:");
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
      ROS_INFO_STREAM("\tAnchor " << anchor_id << ": " << anchor_value.initialized);
      ROS_INFO_STREAM("\t\tpos   : " << anchor_value.p_AinG.transpose());
      ROS_INFO_STREAM("\t\td_bias: " << anchor_value.bias_d);
      ROS_INFO_STREAM("\t\tc_bias: " << anchor_value.bias_c << std::endl);
    }
  }
}

void UwbInitWrapper::cb_posestamped(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  // calculate position of UWB module in G frame
  Eigen::Vector3d p_UinG(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  Eigen::Quaterniond q_UinG(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y,
                            msg->pose.orientation.z);
  p_UinG = p_UinG + q_UinG.toRotationMatrix() * p_r_ItoU_;

  // feed current pose to initializer
  uwb_initializer_.feed_pose(msg->header.stamp.toSec(), p_UinG);
}

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
}

void UwbInitWrapper::cb_dynamicconfig(UwbInitConfig_t& config, uint32_t level)
{
  if (config.calculate)
  {
    // perform anchor claculation
    perform_initialization();

    config.calculate = false;
  }
}

}  // namespace uav_init
