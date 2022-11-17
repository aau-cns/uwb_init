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

namespace uwb_init_ros
{
struct UwbInitRosOptions
{
  /// Logger level
  uwb_init::LoggerLevel level_;

  /// Lib options
  uwb_init::UwbInitOptions init_options_;
  uwb_init::LsSolverOptions ls_solver_options_;
  uwb_init::NlsSolverOptions nls_solver_options_;

  /// ROS options
  std::string estimated_pose_topic_;
  std::string uwb_range_topic_;
  std::string service_init_;

  /// Position of the UWB module expressed in IMU frame
  Eigen::Vector3d p_UinI_;
};
}  // namespace uwb_init_ros

#endif  // UWB_INIT_ROS_OPTIONS_H
