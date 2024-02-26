// Copyright (C) 2022 Alessandro Fornasier, Giulio Delama and Martin Scheiber.
// Control of Networked Systems, University of Klagenfurt, Austria.
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
// You can contact the authors at <alessandro.fornasier@aau.at>,
// <giulio.delama@aau.at> and <martin.scheiber@aau.at>

#ifndef UWB_INIT_HPP_
#define UWB_INIT_HPP_

#include <Eigen/Dense>
#include <chrono>
#include <deque>
#include <map>

#include "logger/logger.hpp"
#include "options/nls_solver_options.hpp"
#include "options/uwb_init_options.hpp"
#include "planners/wps_gen.hpp"
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
  /// \param init_options Initializer options
  /// \param ls_solver_options least square solver options
  /// \param nls_solver_options nonlinear least square solver options
  /// \param planner_options planner options
  ///
  UwbInitializer(const LoggerLevel& level, std::shared_ptr<UwbInitOptions>&& init_options,
                 std::unique_ptr<LsSolverOptions>&& ls_solver_options,
                 std::unique_ptr<NlsSolverOptions>&& nls_solver_options,
                 std::unique_ptr<PlannerOptions>&& planner_options);

  ///
  /// \brief Set option
  ///
  void set_bias_type(const BiasType& type);
  void set_init_method(const InitMethod& method);

  ///
  /// \brief Get solution of the least square formulation of the initialization problem
  ///
  /// \return the actual solution of the least square problem as a constant reference
  /// to LSSolutions.
  ///
  const LSSolutions& get_ls_solutions() const;

  ///
  /// \brief Get solution of the nonlinear least square formulation of the
  /// initialization problem
  ///
  /// \return the actual solution of the least square problem as a constant reference
  /// to NLSSolutions.
  ///
  const NLSSolutions& get_nls_solutions() const;

  ///
  /// \brief Get the refined solution of the nonlinear least square formulation of the
  /// initialization problem
  ///
  /// \return the actual solution of the least square problem as a constant reference
  /// to NLSSolutions.
  ///
  const NLSSolutions& get_refined_solutions() const;

  ///
  /// \brief Get optimal waypoints
  ///
  /// \return the optimal waypoints computed by generate_waypoints as a constant reference
  /// to Waypoints.
  ///
  const Waypoints& get_waypoints() const;

  ///
  /// \brief clears all the data buffers
  ///
  void clear_buffers();

  ///
  /// \brief clear clears all the solutions
  ///
  void clear_solutions();
  void clear_solutions_except_ls();

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
  /// \brief feed_position stores incoming positions of the UAV in the global frame
  /// \param timestamp timestamp of pose
  /// \param p_UinG position to add to buffer
  ///
  void feed_position(const double timestamp, const Eigen::Vector3d p_UinG, uint const Tag_ID=0);

  ///
  /// \brief init_anchors tries to initialize all anchors for which readings exist
  /// \param anchor_buffer
  /// \return true if at least one anchor is successfully initialized, false otherwise
  ///
  /// This function performs a least-squares initialization using the per anchor measurments given the measurement model
  /// from \cite Blueml2021. It will try to initialize each anchor (validated per ID) individually. If an anchor was
  /// already successfully initialized in the past it is skipped. It will also return 'true' if all anchors, for which
  /// measurements are present were successfully initialized at some point.
  ///
  [[nodiscard]] bool init_anchors();

  ///
  /// \brief compute_waypoints computes the optimal waypoints given the initial guess for the uwb anchors position
  /// and the current uwb tag position
  /// \return true if waypoints have been correctly computed, false otherwise
  ///
  [[nodiscard]] bool compute_waypoints(const Eigen::Vector3d pos_k);

  ///
  /// \brief refine_anchors refines the already initialized anchors via optimal waypoints for non-
  /// linear least squares optimization
  /// \return true if at least one anchor is successfully refined,
  /// false if there are no solutions of the nonlinear initialization problem
  ///
  [[nodiscard]] bool refine_anchors();

private:
  // Shared pointer to logger
  std::shared_ptr<Logger> logger_ = nullptr;

  // Initializer parameters
  std::shared_ptr<UwbInitOptions> init_options_ = nullptr;

  // Least squares solver
  LsSolver ls_solver_;

  // Nonlinear least squares solver
  NlsSolver nls_solver_;

  // Optimal Waypoints Generator (Planner)
  OptWpsGenerator planner_;

  // Anchor and measurement handling
  // map<Tag_ID, Hist<p_TinG>>>
  PositionBufferDict_t p_UinG_buffer_;   //!< buffer of UWB module positions in global frame
  // map<Anchor_ID, map<Tag_ID, Hist<UwbData>>>
  UwbDataBufferDict_t uwb_data_buffer_;  //!< history of uwb readings in DataBuffer

  // Solutions handling
  LSSolutions ls_sols_;
  NLSSolutions nls_sols_;
  NLSSolutions refined_sols_;

  // Optimal Waypoints
  Waypoints opt_wps_;


  LSSolution to_LSSolution(Eigen::VectorXd const& lsSolution, Eigen::MatrixXd const& lsCov, size_t const ID_Anchor, std::vector<size_t> const& ID_Tags);
  NLSSolution to_NLSSolution(Eigen::VectorXd const& nlsSolution, Eigen::MatrixXd const& nlsCov, size_t const ID_Anchor, std::vector<size_t> const& ID_Tags);

};

}  // namespace uwb_init

#endif  // UWB_INIT_HPP_
