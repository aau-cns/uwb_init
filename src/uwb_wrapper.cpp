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
    return;
  }

  // Subscribers
  sub_posestamped = nh.subscribe("pose", 1, &UwbInitWrapper::cb_posestamped, this);
  sub_uwbstamped = nh.subscribe("uwb", 1, &UwbInitWrapper::cb_uwbstamped, this);

  // set up dynamic parameters
  ReconfServer_t::CallbackType f = boost::bind(&UwbInitWrapper::cb_dynamicconfig, this, _1, _2);
  reconf_server_.setCallback(f);

  // Print topics where we are subscribing to
  std::cout << std::endl;
  std::cout << "Subscribing: " << sub_posestamped.getTopic().c_str() << std::endl;

  // create initializer
  uwb_initializer_ = UwbInitializer(n_anchors);
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
  u_int n = msg->valid.size();
  for (u_int i = 0; i < n; ++i)
  {
    uwb_ranges.push_back(UwbData(time, msg->valid[i], msg->distance[i], i));
  }

  // feed measurements to initializer
  uwb_initializer_.feed_uwb(uwb_ranges);
}

void UwbInitWrapper::cb_dynamicconfig(UwbInitConfig_t& config, uint32_t level)
{
  if (config.calculate)
  {
    std::map<size_t, Eigen::Vector3d> p_AinG;
    double distance_bias, const_bias, distance_bias_cov, const_bias_cov;
    std::map<size_t, Eigen::Matrix3d> A_covs;

    // perform anchor claculation
    uwb_initializer_.try_to_initialize_anchors(p_AinG, distance_bias, const_bias, A_covs, distance_bias_cov,
                                               const_bias_cov);

    // output result
    ROS_INFO_STREAM("Result:" << std::endl);
#if (__cplusplus == 201703L)
    for (const auto& [key, pos] : p_AinG)
    {
#else
    for (const auto& kv : p_AinG)
    {
      const auto key = kv.first;
      const auto pos = kv.second;
#endif
      ROS_INFO_STREAM("\tPos A" << key << ": " << pos.transpose() << std::endl);
    }
    ROS_INFO_STREAM("\td_bias:" << distance_bias << std::endl);
    ROS_INFO_STREAM("\tc_bias:" << const_bias << std::endl);

    config.calculate = false;
  }
}

}  // namespace uav_init
