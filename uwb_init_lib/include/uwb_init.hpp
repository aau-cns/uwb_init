// Copyright (C) 2021 Giulio Delama, Alessandro Fornasier, Martin Scheiber
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
// You can contact the authors at <giulio.delama@aau.at>,
// <alessandro.fornasier@aau.at>, and <martin.scheiber@aau.at>

#ifndef UWB_INIT_HPP_
#define UWB_INIT_HPP_

#include <Eigen/Dense>
#include <chrono>
#include <deque>
#include <map>

#include "logger/logger.hpp"
#include "options/nls_solver_options.hpp"
#include "options/uwb_init_options.hpp"
#include "solvers/linear/ls_solver.hpp"
#include "solvers/nonlinear/nls_solver.hpp"
#include "utils/data_structs.hpp"

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
  /// \param level logging level
  /// \param init_params Initializer options
  ///
  UwbInitializer(const LoggerLevel& level = LoggerLevel::FULL, const UwbInitOptions& init_params_ = UwbInitOptions());

  ///
  /// \brief Set option
  ///
  void set_bias_type(const BiasType& type);
  void set_init_method(const InitMethod& method);

  ///
  /// \brief Get solution of the leas square formulation of the initialization problem
  ///
  /// \return the actual solution of the least square problem as a constant reference
  /// to LSSOlution.
  ///
  const LSSolutions& get_ls_solutions() const;

  ///
  /// \brief Get solution of the nonlinear least square formulation of the
  /// initialization problem
  ///
  /// \return the actual solution of the least square problem as a constant reference
  /// to NLSSOlution.
  ///
  const NLSSolutions& get_nls_solutions() const;

  ///
  /// \brief clears all the data buffers
  ///
  void clear_buffers();

  ///
  /// \brief clear clears all the solutions
  ///
  void clear_solutions();

  ///
  /// \brief reset resets the initializer and clean all its buffers
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
  // TODO(alf) no discard ?

  ///
  /// \todo TODO (gid) planner for waypoint generation to refine anchors
  ///

  ///
  /// \brief refine_anchors refines the already initialized anchors via optimal waypoints for non-
  /// linear least squares optimization
  ///
  bool refine_anchors();

private:
  // Shared pointer to logger
  std::shared_ptr<Logger> logger_ = nullptr;

  // Initializer parameters
  UwbInitOptions init_params_;

  // TODO(alf) why shared ptr? --> not need of it
  // Shared pointer to least squares solver
  std::shared_ptr<LsSolver> ls_solver_ = nullptr;

  // Shared pointer to nonlinear least squares solver
  std::shared_ptr<NlsSolver> nls_solver_ = nullptr;

  // Anchor and measurement handling
  PositionBuffer p_UinG_buffer_;   //!< buffer of UWB module positions in global frame
  UwbDataBuffer uwb_data_buffer_;  //!< history of uwb readings in DataBuffer

  // Solutions handling
  // TODO(alf) check value of uninitialized map
  LSSolutions ls_sols_;
  NLSSolutions nls_sols_;
};

}  // namespace uwb_init

#endif  // UWB_INIT_HPP_
