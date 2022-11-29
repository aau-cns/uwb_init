// Copyright (C) 2022 Alessandro Fornasier, Giulio Delama
// Control of Networked Systems, University of Klagenfurt, Austria.
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
//
// You can contact the author at <alessandro.fornasier@aau.at> and
// <giulio.delama@aau.at>

#include <ros/ros.h>
#include <Eigen/Eigen>

#include "uwb_init_ros.hpp"

namespace uwb_init_ros
{
UwbInitRos::UwbInitRos(const ros::NodeHandle& nh, UwbInitRosOptions&& options)
  : nh_(nh)
  , options_(std::move(options))
  , uwb_init_(options_.level_, std::move(options_.init_options_), std::move(options_.ls_solver_options_), std::move(options_.nls_solver_options_), std::move(options_.planner_options_))
{
  // Subscribers
  estimated_pose_sub_ = nh_.subscribe(options_.estimated_pose_topic_, 1, &UwbInitRos::callbackPose, this);
  uwb_range_sub_ = nh_.subscribe(options_.uwb_range_topic_, 1, &UwbInitRos::callbackUwbRanges, this);

  // Print topics where we are subscribing to
  ROS_INFO("Subsribing to %s", estimated_pose_sub_.getTopic().c_str());
  ROS_INFO("Subsribing to %s", uwb_range_sub_.getTopic().c_str());

  // Publishers

  // Print topics where we are publishing on
  // ROS_INFO("Publishing in %s");

  // Services
  start_srv_ = nh_.advertiseService(options_.service_start_, &UwbInitRos::callbackServiceStart, this);
  reset_srv_ = nh_.advertiseService(options_.service_reset_, &UwbInitRos::callbackServiceReset, this);
  init_srv_ = nh_.advertiseService(options_.service_init_, &UwbInitRos::callbackServiceInit, this);
  wps_srv_ = nh_.advertiseService(options_.service_wps_, &UwbInitRos::callbackServiceWps, this);
  refine_srv_ = nh_.advertiseService(options_.service_refine_, &UwbInitRos::callbackServiceRefine, this);
}

void UwbInitRos::callbackPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  // Get pose
  Eigen::Vector3d p_IinG(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  Eigen::Quaterniond q_GI(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y,
                          msg->pose.orientation.z);

  // Compute position of UWB module in Global frame
  p_UinG_ = p_IinG + q_GI.toRotationMatrix() * options_.p_UinI_;

  // Feed p_UinG
  if (collect_measurements_)
  {
    uwb_init_.feed_position(msg->header.stamp.toSec(), p_UinG_);
  }
}

void UwbInitRos::callbackUwbRanges(const mdek_uwb_driver::UwbConstPtr& msg)
{
  // Parse message into correct data structure
  std::vector<uwb_init::UwbData> data;

  // Fill data
  for (const auto& it : msg->ranges)
  {
    // Check if the id is valid (contains only numbers)
    if (!containsChar(it.id))
    {
      // Convert id
      std::stringstream sstream(it.id);
      uint id;
      sstream >> id;

      // Fill vector
      data.emplace_back(uwb_init::UwbData(true, it.distance, id));
    }
    else
    {
      ROS_WARN("Received UWB message containing characters in the id field. Measurement discarded");
    }
  }

  // Feed measurements
  if (collect_measurements_)
  {
    uwb_init_.feed_uwb(msg->header.stamp.toSec(), data);
  }
}

bool UwbInitRos::callbackServiceStart(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  ROS_INFO("Start service called.");
  collect_measurements_ = true;
  return true;
}

bool UwbInitRos::callbackServiceReset(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  ROS_INFO("Reset service called.");
  uwb_init_.reset();
  collect_measurements_ = false;
  return true;
}

bool UwbInitRos::callbackServiceInit(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  ROS_INFO("Inizialization service called.");
  return initializeAnchors();
}

bool UwbInitRos::callbackServiceWps(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  ROS_INFO("Refinement service called.");
  return computeWaypoints();
}

bool UwbInitRos::callbackServiceRefine(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  ROS_INFO("Refinement service called.");
  return refineAnchors();
}

bool UwbInitRos::initializeAnchors()
{
  ROS_INFO("Performing anchor inizalization...");

  // Initialize anchors
  if (!uwb_init_.init_anchors())
  {
    ROS_WARN("Initialization of anchor failed. Please collect additional data and repeat.");

    if (!collect_measurements_)
    {
      ROS_WARN_STREAM("Call " << options_.service_start_ << " to start collecting measurements.");
    }
    return false;
  }

  // Display anchors that have been sucessufully initialized
  for (const auto& it : uwb_init_.get_ls_solutions())
  {
    ROS_INFO("Anchor [%d] succesfully initialized at [%f, %f, %f]", it.first, it.second.anchor_.p_AinG_.x(),
             it.second.anchor_.p_AinG_.y(), it.second.anchor_.p_AinG_.z());
  }

  collect_measurements_ = false;

  return true;
}

bool UwbInitRos::computeWaypoints()
{
  ROS_INFO("Computing optimal waypoints...");

  // Compute waypoints passing last registerd UWB tag position
  if (!uwb_init_.compute_waypoints(p_UinG_))
  {
    ROS_WARN("Optimal waipoints computation. Please make sure that the UWB anchors have been correctly initialized.");
    return false;
  }

  // Display waypoints
  for (const auto& it : uwb_init_.get_waypoints())
  {
    ROS_INFO("Waypoint at [%f, %f, %f]", it.x_, it.y_, it.z_);
  }

  return true;
}

bool UwbInitRos::refineAnchors()
{
  ROS_INFO("Performing anchor refinement...");

  // Refine anchors
  if (!uwb_init_.refine_anchors())
  {
    ROS_WARN("Refinement of anchor failed. Please collect additional data and repeat.");
    return false;
  }

  // Display anchors that have been sucessufully refined
  for (const auto& it : uwb_init_.get_nls_solutions())
  {
    ROS_INFO("Anchor [%d] succesfully refined at [%f, %f, %f]", it.first, it.second.anchor_.p_AinG_.x(),
             it.second.anchor_.p_AinG_.y(), it.second.anchor_.p_AinG_.z());
  }

  return true;
}

}  // namespace uwb_init_ros
