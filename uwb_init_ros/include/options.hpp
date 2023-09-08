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

#ifndef UWB_INIT_ROS_OPTIONS_H
#define UWB_INIT_ROS_OPTIONS_H

#include "logger/logger.hpp"
#include "options/ls_solver_options.hpp"
#include "options/nls_solver_options.hpp"
#include "options/planner_options.hpp"
#include "options/uwb_init_options.hpp"

namespace uwb_init_ros
{
struct UwbInitRosOptions
{
  /// Logger level
  uwb_init::LoggerLevel level_;

  /// Lib options
  std::shared_ptr<uwb_init::UwbInitOptions> init_options_ = nullptr;
  std::unique_ptr<uwb_init::LsSolverOptions> ls_solver_options_ = nullptr;
  std::unique_ptr<uwb_init::NlsSolverOptions> nls_solver_options_ = nullptr;
  std::unique_ptr<uwb_init::PlannerOptions> planner_options_ = nullptr;

  /// ROS options
  std::string estimated_pose_cov_topic_{""};
  std::string estimated_pose_topic_{""};
  std::string estimated_transform_topic_{""};

  std::string uwb_range_topic_;
  std::string uwb_twr_topic_;
  uint uwb_ref_id_{0};  //!< ID of the UWB device (will be set automatically by the twr msg)

  std::string service_start_;
  std::string service_reset_;
  std::string service_init_;
  std::string service_wps_;
  std::string service_refine_;
  std::string uwb_anchors_topic_;
  std::string waypoints_topic_;
  std::string frame_id_anchors_;    //!< frame ID of the anchors, by default global
  std::string frame_id_waypoints_;  //!< frame ID of the waypoints used, can differ to global if vision flight e.g.

  /// Anchors initialization options
  bool publish_first_solution_;
  std::string anchors_file_path_;
  bool publish_anchors_tf_;
  double uwb_min_range_;
  double uwb_max_range_;

  /// Position of the UWB module expressed in IMU frame
  Eigen::Vector3d p_UinI_;

  /// IDs that are rejected
  std::vector<size_t> uwb_id_black_list;

  double wp_yaw_;
  double wp_holdtime_;

  // Waypoint Options
  uint wp_nav_type_;  //!< see mission_sequencer::WaypointArrayStamped for further information

  /// Constructors (implicitly deleted copy constructor)
  UwbInitRosOptions() = default;
  UwbInitRosOptions(UwbInitRosOptions&&) = default;
};
}  // namespace uwb_init_ros

#endif  // UWB_INIT_ROS_OPTIONS_H
