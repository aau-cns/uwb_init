// Copyright (C) 2021 Martin Scheiber, Alessandro Fornasier, Giulio Delama
// Control of Networked Systems, Universitaet Klagenfurt, Austria
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
// You can contact the authors at <martin.scheiber@aau.at>,
// <alessandro.fornasier@aau.at> and <giulio.delama@aau.at>

#ifndef UWB_INIT_HPP_
#define UWB_INIT_HPP_

#include <Eigen/Dense>
#include <chrono>
#include <deque>
#include <map>

#include "logger/logger.hpp"
#include "options/uwb_init_options.hpp"
#include "options/nls_solver_options.hpp"
#include "utils/data_structs.hpp"
#include "utils/ls_solver.hpp"
#include "utils/nls_solver.hpp"

namespace uwb_init
{
///
/// \brief The UwbInitializer class is the main object used for UWB data handling
/// for the purpose of initializing the UWB anchors.
///
class UwbInitializer
{
public:
  ///
  /// \brief UwbInitializer default constructor
  /// \param params parameter/options used for UWB initialization
  ///
  UwbInitializer(const LoggerLevel& level = LoggerLevel::FULL);
  UwbInitializer(const UwbInitOptions init_params_, const LoggerLevel& level = LoggerLevel::FULL);

  ///
  /// \brief set_option
  ///
  void set_init_method_single();
  void set_init_method_double();
  void set_init_unbiased();
  void set_init_const_bias();

  ///
  /// \brief get_option
  ///
  std::string const get_init_method() const;
  std::string const get_init_variables() const;

  ///
  /// \brief clears all the data buffers
  ///
  void clear_buffers();

  ///
  /// \brief clear clears all the solutions
  ///
  void clear_solutions();

  ///
  /// \brief reset resets the initializer and all its buffers
  ///
  void reset();

  ///
  /// \brief feed_uwb stores incoming UWB (valid) readings
  /// \param uwb_measurements UwbData vector of measurements
  ///
  void feed_uwb(const double timestamp, const std::vector<UwbData> uwb_measurements);  
  void feed_uwb(const double timestamp, const UwbData uwb_measurement);

  ///
  /// \brief feed_pose stores incoming positions of the UAV in the global frame
  /// \param timestamp timestamp of pose
  /// \param p_UinG position to add to buffer
  ///
  void feed_pose(const double timestamp, const Eigen::Vector3d p_UinG);

  ///
  /// \brief init_anchors tries to initialize all anchors for which readings exist
  /// \param anchor_buffer
  /// \return true if all anchors were successfully initialized
  ///
  /// This function performs a least-squares initialization using the per anchor measurments given the measurement model
  /// from \cite Blueml2021. It will try to initialize each anchor (validated per ID) individually. If an anchor was
  /// already successfully initialized in the past it is skipped. It will also return 'true' if all anchors, for which
  /// measurements are present were successfully initialized at some point.
  ///
  bool init_anchors();

  ///
  /// \todo TODO (gid) planner for waypoint generation to refine anchors
  ///

  ///
  /// \brief refine_anchors refines the already initialized anchors via optimal waypoints for non-
  /// linear least squares optimization
  ///
  bool refine_anchors();

  // Shared pointer to logger
  std::shared_ptr<Logger> logger_ = nullptr;

  // Shared pointer to least squares solver
  std::shared_ptr<LsSolver> ls_solver_ = nullptr;

  // Shared pointer to nonlinear least squares solver
  std::shared_ptr<NlsSolver> nls_solver_ = nullptr;

private:
  // Initializer parameters
  UwbInitOptions init_params_;

  // Anchor and measurement handling
  PositionBuffer p_UinG_buffer_;    //!< buffer of UWB module positions in global frame
  UwbDataBuffer uwb_data_buffer_;  //!< history of uwb readings in DataBuffer

  // Solutions handling
  LSSolutions ls_sols_;
  NLSSolutions nls_sols_;
};

}  // namespace uwb_init

#endif  // UWB_INIT_HPP_
