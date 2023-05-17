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

#include "uwb_init_ros.hpp"

#include <ros/ros.h>

#include <Eigen/Eigen>

namespace uwb_init_ros
{
UwbInitRos::UwbInitRos(const ros::NodeHandle& nh, UwbInitRosOptions&& options)
  : nh_(nh)
  , options_(std::move(options))
  , uwb_init_(options_.level_, std::move(options_.init_options_), std::move(options_.ls_solver_options_),
              std::move(options_.nls_solver_options_), std::move(options_.planner_options_))
{
  // Subscribers
  estimated_pose_sub_ = nh_.subscribe(options_.estimated_pose_topic_, 1, &UwbInitRos::callbackPoseWithCov, this);
  uwb_range_sub_ = nh_.subscribe(options_.uwb_range_topic_, 1, &UwbInitRos::callbackUwbRanges, this);

  // Publishers
  uwb_anchors_pub_ = nh_.advertise<uwb_init_ros::UwbAnchorArrayStamped>(options_.uwb_anchors_topic_, 1);
  waypoints_pub_ = nh_.advertise<mission_sequencer::MissionWaypointArray>(options_.waypoints_topic_, 1);

  // Print topics where we are subscribing to
  ROS_INFO("Subsribing to %s", estimated_pose_sub_.getTopic().c_str());
  ROS_INFO("Subsribing to %s", uwb_range_sub_.getTopic().c_str());

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

void UwbInitRos::callbackPoseWithCov(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  // Get pose
  Eigen::Vector3d p_IinG(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  Eigen::Quaterniond q_GI(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                          msg->pose.pose.orientation.z);

  // Compute position of UWB module in Global frame
  p_UinG_ = p_IinG + q_GI.toRotationMatrix() * options_.p_UinI_;

  // Feed p_UinG
  if (collect_measurements_)
  {
    uwb_init_.feed_position(msg->header.stamp.toSec(), p_UinG_);
  }
}

void UwbInitRos::callbackTransform(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
  // Get pose
  Eigen::Vector3d p_IinG(msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z);
  Eigen::Quaterniond q_GI(msg->transform.rotation.w, msg->transform.rotation.x, msg->transform.rotation.y,
                          msg->transform.rotation.z);

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
  // [TODO] Check if this needs to be done
  uwb_init_.clear_buffers();
  uwb_init_.clear_solutions_except_ls();
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
  ROS_INFO("Waypoints generation service called.");
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

  // Add a check for the number of anchors initialized and ask if the user wants to continue
  if (uwb_init_.get_ls_solutions().size() < options_.min_num_anchors_)
  {
    ROS_WARN("Number of anchors initialized is less than the minimum required. Please collect additional data and "
             "repeat.");
    return false;
  }

  // Stop collecting measurements
  ROS_INFO("Anchor initialization completed. Stopping data collection.");
  collect_measurements_ = false;

  // Display and publish anchors that have been sucessufully initialized
  uwb_init_ros::UwbAnchorArrayStamped anchors_msg_;

  if (!uwb_init_.refine_anchors() || (uwb_init_.get_nls_solutions().size() < options_.min_num_anchors_))
  {
    ROS_WARN("Refinement of anchor failed. Publishing linear solution.");

    for (const auto& it : uwb_init_.get_ls_solutions())
    {
      ROS_INFO("Anchor [%d]: p_AinG = [%f, %f, %f] | const_bias = %f", it.first, it.second.anchor_.p_AinG_.x(),
               it.second.anchor_.p_AinG_.y(), it.second.anchor_.p_AinG_.z(), it.second.gamma_);

      // Anchor message
      uwb_init_ros::UwbAnchor anchor;
      anchor.id = it.first;
      anchor.position.x = it.second.anchor_.p_AinG_.x();
      anchor.position.y = it.second.anchor_.p_AinG_.y();
      anchor.position.z = it.second.anchor_.p_AinG_.z();
      anchor.gamma = it.second.gamma_;
      anchor.beta = 1.0;

      // Covariance
      Eigen::MatrixXd cov = Eigen::MatrixXd::Zero(5, 5);
      cov.block(0, 0, 4, 4) = it.second.cov_;
      cov(4, 4) = 0.1;  // Initial guess

      // Store upper simmetric part only
      int index = 0;
      for (int i = 0; i < cov.rows(); i++)
      {
        for (int j = i; j < cov.cols(); j++)
        {
          anchor.covariance.at(index++) = cov(i, j);
        }
      }

      anchors_msg_.anchors.push_back(anchor);
    }

    ++anchors_msg_.header.seq;
    anchors_msg_.header.stamp = ros::Time::now();
    anchors_msg_.header.frame_id = "global";
  }
  else
  {
    ROS_INFO("Refinement of anchor successful. Publishing refined solution.");

    for (const auto& it : uwb_init_.get_nls_solutions())
    {
      ROS_INFO("Anchor [%d]: p_AinG = [%f, %f, %f] | const_bias = %f | dist_bias = %f", it.first,
               it.second.anchor_.p_AinG_.x(), it.second.anchor_.p_AinG_.y(), it.second.anchor_.p_AinG_.z(),
               it.second.gamma_, it.second.beta_);

      // Anchor message
      uwb_init_ros::UwbAnchor anchor;
      anchor.id = it.first;
      anchor.position.x = it.second.anchor_.p_AinG_.x();
      anchor.position.y = it.second.anchor_.p_AinG_.y();
      anchor.position.z = it.second.anchor_.p_AinG_.z();
      anchor.gamma = it.second.gamma_;
      anchor.beta = it.second.beta_;

      // Store covarinace upper simmetric part only
      int index = 0;
      for (int i = 0; i < it.second.cov_.rows(); i++)
      {
        for (int j = i; j < it.second.cov_.cols(); j++)
        {
          anchor.covariance.at(index++) = it.second.cov_(i, j);
        }
      }

      anchors_msg_.anchors.push_back(anchor);
    }

    ++anchors_msg_.header.seq;
    anchors_msg_.header.stamp = ros::Time::now();
    anchors_msg_.header.frame_id = "global";
  }

  // [TODO] Disabled, this shhould be done by parameter option
  // uwb_anchors_pub_.publish(anchors_msg_);

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

  // Display and publish waypoints
  mission_sequencer::MissionWaypointArray waypoints_msg_;
  for (const auto& it : uwb_init_.get_waypoints())
  {
    ROS_INFO("Waypoint at [%f, %f, %f]", it.x_, it.y_, it.z_);

    mission_sequencer::MissionWaypoint wp;
    wp.x = it.x_;
    wp.y = it.y_;
    wp.z = it.z_;
    wp.yaw = options_.wp_yaw_;
    wp.holdtime = options_.wp_holdtime_;
    waypoints_msg_.waypoints.push_back(wp);
  }

  ++waypoints_msg_.header.seq;
  waypoints_msg_.header.stamp = ros::Time::now();
  waypoints_msg_.header.frame_id = "local";
  waypoints_msg_.is_global = false;  // Set to false to use local coordinates
  waypoints_msg_.action = mission_sequencer::MissionWaypointArray::CLEAR;

  waypoints_pub_.publish(waypoints_msg_);

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

  // Display and publish anchors that have been sucessufully refined
  uwb_init_ros::UwbAnchorArrayStamped anchors_msg_;
  for (const auto& it : uwb_init_.get_nls_solutions())
  {
    ROS_INFO("Anchor [%d] succesfully refined at [%f, %f, %f]", it.first, it.second.anchor_.p_AinG_.x(),
             it.second.anchor_.p_AinG_.y(), it.second.anchor_.p_AinG_.z());

    // Anchor message
    uwb_init_ros::UwbAnchor anchor;
    anchor.id = it.first;
    anchor.position.x = it.second.anchor_.p_AinG_.x();
    anchor.position.y = it.second.anchor_.p_AinG_.y();
    anchor.position.z = it.second.anchor_.p_AinG_.z();
    anchor.gamma = it.second.gamma_;
    anchor.beta = it.second.beta_;

    // Store covarinace upper simmetric part only
    int index = 0;
    for (int i = 0; i < it.second.cov_.rows(); i++)
    {
      for (int j = i; j < it.second.cov_.cols(); j++)
      {
        anchor.covariance.at(index++) = it.second.cov_(i, j);
      }
    }

    anchors_msg_.anchors.push_back(anchor);
  }

  ++anchors_msg_.header.seq;
  anchors_msg_.header.stamp = ros::Time::now();
  anchors_msg_.header.frame_id = "global";

  uwb_anchors_pub_.publish(anchors_msg_);

  return true;
}

}  // namespace uwb_init_ros
