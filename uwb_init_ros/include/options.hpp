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
  std::string estimated_pose_topic_;
  std::string uwb_range_topic_;
  std::string service_start_;
  std::string service_reset_;
  std::string service_init_;
  std::string service_wps_;
  std::string service_refine_;

  /// Position of the UWB module expressed in IMU frame
  Eigen::Vector3d p_UinI_;

  /**
   * @brief Copy constructor
   */
  UwbInitRosOptions(const UwbInitRosOptions& other) = delete;

  /**
   * @brief Move constructor
   */
  UwbInitRosOptions(UwbInitRosOptions&& other) = default;
//  {
//    level_ = std::move(other.level_);
//    init_options_ = std::move(other.init_options_);
//    ls_solver_options_ = std::move(other.ls_solver_options_);
//    nls_solver_options_ = std::move(other.nls_solver_options_);
//    planner_options_ = std::move(other.planner_options_);
//    estimated_pose_topic_ = other.estimated_pose_topic_);
//    uwb_range_topic_ = std::move(other.uwb_range_topic_);
//    service_start_ = std::move(other.service_start_);
//    service_reset_ = std::move(other.service_reset_);
//    service_init_ = std::move(other.service_init_);
//    service_wps_ = std::move(other.service_wps_);
//    service_refine_ = std::move(other.service_refine_);
//  }

    /**
     * @brief Assignment operator overloading
     * @param other (const reference to UwbInitRosOptions)
     * @return UwbInitRosOptions&
     */
    UwbInitRosOptions& operator=(const UwbInitRosOptions& other) = delete;

    /**
     * @brief Assignment operator overloading
     * @param other (rvalue reference)
     * @return UwbInitRosOptions&
     */
    UwbInitRosOptions& operator=(UwbInitRosOptions&& other) = default;

};
}  // namespace uwb_init_ros

#endif  // UWB_INIT_ROS_OPTIONS_H
