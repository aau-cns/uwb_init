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

#ifndef UWB_INIT_ROS_H
#define UWB_INIT_ROS_H

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <geometry_msgs/PoseStamped.h>
#include <mdek_uwb_driver/Uwb.h>
#include <std_srvs/Empty.h>
#include <uwb_init_lib/include/uwb_init.hpp>

#include "options.hpp"
#include "utilities.hpp"

namespace uwb_init_ros
{
class UwbInitRos
{
public:
  /**
   * @brief Constructor
   *
   * @param nh ROS NodeHandle
   */
  UwbInitRos(const ros::NodeHandle& nh, const UwbInitRosOptions& options);

private:
  /**
   * @brief Pose callback
   *
   * @param msg
   */
  void callbackPose(const geometry_msgs::PoseStamped::ConstPtr& msg);

  /**
   * @brief uwb ranges callback
   *
   * @param msg
   */
  void callbackUwbRanges(const mdek_uwb_driver::UwbConstPtr& msg);

  /**
   * @brief Starting service callback
   *
   * @param req request
   * @param res response
   */
  bool callbackServiceStart(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  /**
   * @brief Reset service callback
   *
   * @param req request
   * @param res response
   */
  bool callbackServiceReset(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  /**
   * @brief Initialization service callback
   *
   * @param req request
   * @param res response
   * @return true if at least one anchor has been correctly initialized
   */
  bool callbackServiceInit(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  /**
   * @brief Refine service callback
   *
   * @param req request
   * @param res response
   * @return true if at least one anchor has been correctly refined
   */
  bool callbackServiceRefine(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  /**
   * @brief Initialize anchors with data collected so far
   *
   * @return true if at least one anchor has been correctly initialized and refined
   */
  [[nodiscard]] bool refineAnchors();

  /**
   * @brief Refine anchors with data collected so far
   *
   * @return true if at least one anchor has been correctly refined
   */
  [[nodiscard]] bool initializeAnchors();

  // Flags
  bool fstart_collect_measurements_ = false;

  /// Ros node handler
  ros::NodeHandle nh_;

  /// Subscribers
  ros::Subscriber uwb_range_sub_;
  ros::Subscriber estimated_pose_sub_;

  /// Options
  UwbInitRosOptions options_;

  /// Publishers

  /// Messages

  /// Service Servers
  ros::ServiceServer start_srv_;
  ros::ServiceServer reset_srv_;
  ros::ServiceServer init_srv_;
  ros::ServiceServer refine_srv_;

  /// UWB initializer
  uwb_init::UwbInitializer uwb_init_;
};
}  // namespace uwb_init_ros

#endif  // UWB_INIT_ROS_H
