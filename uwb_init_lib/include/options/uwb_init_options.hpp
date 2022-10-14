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

#ifndef UAV_INIT_UWB_OPTIONS_HPP_
#define UAV_INIT_UWB_OPTIONS_HPP_

#include <Eigen/Eigen>

#include "utils/logger.hpp"

namespace UwbInit
{
///
/// \brief The UwbInitOptions struct is an object containing all 'static' parameters used.
///
struct UwbInitOptions
{
  ///
  /// \brief The InitMethod enum describes the method used for initialization
  ///
  enum class InitMethod
  {
    SINGLE,  //!< use only one measurement to construct LLS matrix
    DOUBLE,  //!< use a pair of measurements to construct LLS matrix
  };

  ///
  /// \brief The InitVariables enum describes the type of variables to initialize
  ///
  enum class InitVariables
  {
    FULL_BIAS,   //!< use all variables, position, distance bias, and constant bias
    NO_BIAS,     //!< use no bias for initialization, i.e. position only
    CONST_BIAS,  //!< only use constant bias and position in initialization
    DIST_BIAS,   //!< only use distance bias and position in initialization
  };

  friend inline std::ostream& operator<<(std::ostream& os, InitMethod method)
  {
    switch (method)
    {
      case InitMethod::SINGLE:
        return os << "SINGLE";
      case InitMethod::DOUBLE:
        return os << "DOUBLE";
    }

    return os;
  }

  friend inline std::ostream& operator<<(std::ostream& os, InitVariables variable)
  {
    switch (variable)
    {
      case InitVariables::FULL_BIAS:
        return os << "FULL_BIAS";
      case InitVariables::NO_BIAS:
        return os << "NO_BIAS";
      case InitVariables::CONST_BIAS:
        return os << "CONST_BIAS";
      case InitVariables::DIST_BIAS:
        return os << "DIST_BIAS";
    }

    return os;
  }

  // WRAPPER AND CALIBRATION ==================================================

  /// duration between consecutive checks when initialization is performed in seconds
  double init_check_duration_s{ 5.0 };

  /// translational offset of the UWB module w.r.t. the IMU (or body frame) in meter
  Eigen::Vector3d p_ItoU{ Eigen::VectorXd::Zero(3) };

  /// name of the pose topic used for current UAV position in global frame
  std::string topic_sub_pose{ "pose" };

  /// name of the uwb topic used to receive the UWB measurements
  std::string topic_sub_uwb{ "uwb" };

  /// name of the anchors topic used to publish the UWB anchor positions
  std::string topic_pub_anchors{ "anchors" };

  /// name of the waypoint list topic used to publish the next waypoints
  std::string topic_pub_wplist{ "waypoints" };

  /// name of the get start pose service used to derive start pose for navigation
  std::string service_ms_get_start_pose{ "/mission_sequencer/getStartPose" };

  /// name of the get start pose service used to derive start pose for navigation
  std::string service_start_init{ "/uwb_init/start" };

  /// Flag to allow publsihing of initialized anchors only if all anchors have been initialized
  bool publish_only_when_all_initialized{false};

  void print_wrapper()
  {
// TODO (gid)
//      logger_->info("Parameter Summary -- Wrapper");
//      logger_->info("\t- init_check_duration_s:               " << init_check_duration_s);
//      logger_->info("\t- topic_sub_pose:                      " << topic_sub_pose);
//      logger_->info("\t- topic_sub_uwb:                       " << topic_sub_uwb);
//      logger_->info("\t- topic_pub_anchors:                   " << topic_pub_anchors);
//      logger_->info("\t- topic_pub_wplist:                    " << topic_pub_wplist);
//      logger_->info("\t- service_ms_get_start_pose:           " << service_ms_get_start_pose);
//      logger_->info("\t- service_start_init:                  " << service_start_init);
//      logger_->info("\t- publish_only_when_all_initialized:   " << publish_only_when_all_initialized);
//      logger_->info("\t- p_ItoU:                              " << p_ItoU.transpose());
  }

  // UWB INITIALIZER ==========================================================

  /// allow continous initialization although solution for anchor was already found
  bool b_do_continous_init{ false };

  /// buffer size of all buffers in s
  double buffer_size_s{ 10.0 };

  /// maximum condition number of the LS matrix for valid initialization
  double max_cond_num{ 100.0 };

  /// minimum baseline in meter accepted to add obsevation to the double LS problem
  /// (z1^2 - z2^2 must be grater than meas_baseline_m_)
  double meas_baseline_m_{ 0.05 };

  /// indices baseline used for the double method
  uint meas_baseline_idx_{ 50 };

  /// Value of lamda used for regularization, if lambda = 0 no regularization is applied
  /// (suggested value 1000)
  double lamda{ 1000 };

  /// Threshold on norm of LS solution covariance's singular values (used to accept the LS solution)
  double cov_sv_threshold{ 1e-3 };

  /// time diference of poses to UWB measurements in s
  double t_pose_diff{ 0.0 };

  /// determines the method to use for initialization \see UwbInit::UwbInitOptions::InitMethod
  InitMethod init_method{ InitMethod::DOUBLE };

  /// determines the variables to initialize in initialization routine \see UwbInit::UwbInitOptions::InitVariables
  InitVariables init_variables{ InitVariables::FULL_BIAS };

  /// perform regularization (in general this should be avoided)
  bool b_perform_regularization{ false };

  /// determines if the constant bias of the anchors should be regularized to zero
  /// in general this should be avoided
  bool b_regularize_k{ false };

  /// determines if the distance bias of the anchors should be regularized to one
  /// in general this should be avoided
  bool b_regularize_b{ false };

  /// determines if the z component of the anchors should be added to the regularization
  /// turn this off if the anchors are not placed on the ground (of the estimation frame)
  bool b_regularize_z{ false };

  /// flag to determine if the initializer should exit once it has initialized all known anchors
  /// turn this off if new anchors are added on the fly without prior knowledge on number of anchors
  bool b_exit_when_initialized{ false };

  /// flag to determine if initialization should automatically be started
  bool b_auto_init{ true };

  void print_initializer()
  {
    // TODO (gid)

    if (init_method == InitMethod::DOUBLE)
    {
      // todo (gid)
    }
  }

  // WAYPOINT GENERATION ======================================================

  /// maximum distance between two waypoints generated
  double wp_generation_max_distance{ 1.0 };

  /// height of waypoints for initialization purpose
  double wp_height{ 2.0 };

  /// random offset added to waypoint generation (for both entries --> not a circle but rectangle addition)
  double wp_rand_offset{ 0.0 };

  /// holdtime at waypoints
  double wp_holdtime{ 0.5 };

  void print_waypoint()
  {
    // TODO (gid)
  }

};  // class UwbInitOptions
}  // namespace UwbInit

#endif  // UAV_INIT_UWB_OPTIONS_HPP_
