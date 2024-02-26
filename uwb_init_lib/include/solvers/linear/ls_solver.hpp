// Copyright (C) 2022 Alessandro Fornasier, Giulio Delama.
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
// You can contact the authors at <alessandro.fornasier@aau.at> and
// <giulio.delama@aau.at>

#ifndef LS_SOLVER_HPP_
#define LS_SOLVER_HPP_

#include <assert.h>

#include "logger/logger.hpp"
#include "options/ls_solver_options.hpp"
#include "options/uwb_init_options.hpp"
#include "utils/data_structs.hpp"
#include "utils/utils.hpp"

namespace uwb_init
{
class LsSolver
{
public:
  /**
   * @brief Construct a new Ls Solver object
   *
   * @param logger
   * @param options initialization options specifying the type of model,
   * and the method used for building the least square problem
   * @param ls_solver_options configuration of the least square solver
   */
  LsSolver(const std::shared_ptr<Logger> logger, const std::shared_ptr<UwbInitOptions>& init_options,
           std::unique_ptr<LsSolverOptions>&& ls_solver_options);

  /**
   * @brief Configure LS solver problem type (unbiased or const bias), and method (single or double)
   *
   * @param init_options initialization options specifying the type of model,
   * and the method used for building the least square problem
   */
  void configure(const std::shared_ptr<UwbInitOptions>& init_options);

  /**
   * @brief Function to be called to solve the least square problem
   *
   * @param uwb_data
   * @param p_UinG_buffer
   * @param lsSolution
   * @param cov
   * @return true if a solution is found, false otherwise
   */
  [[nodiscard]] bool solve_ls(const TimedBuffer<UwbData>& uwb_data, const PositionBuffer& p_UinG_buffer,
                              Eigen::VectorXd& lsSolution, Eigen::MatrixXd& cov);


  /**
   * @brief Function to be called to solve the least square problem
   *
   * @param dict_uwb_data  <Tag_ID, Hist<UWBData>>
   * @param dict_p_UinG_buffer <Tag_ID, Hist<p_UinG>>
   * @param lsSolution
   * @param cov
   * @return true if a solution is found, false otherwise
   */
  bool solve_ls(const UwbDataPerTag& dict_uwb_data,
                const PositionBufferDict_t&dict_p_UinG_buffer, Eigen::VectorXd& lsSolution,
                Eigen::MatrixXd& cov);


  /**
   * @brief The least square problem
   *
   */
  std::function<bool(const UwbDataPerTag&, const PositionBufferDict_t&, Eigen::MatrixXd&, Eigen::VectorXd&,
                     Eigen::VectorXd&)>
      ls_problem;

private:
  /// Shared pointer to logger
  std::shared_ptr<Logger> logger_ = nullptr;

  /// LsSolver parameters
  std::unique_ptr<LsSolverOptions> solver_options_ = nullptr;

  ///
  /// \brief functions for least squares problem formulation depending on selected method and variables
  /// \param UWB data for the single anchor, coefficient matrix A, measurement vector b (A * x = b), uncertainty s
  /// \return ture if successful, false if not
  ///
  bool ls_single_const_bias(const UwbDataPerTag& dict_uwb_data, const PositionBufferDict_t &dict_p_UinG_buffer,
                            Eigen::MatrixXd& A, Eigen::VectorXd& b, Eigen::VectorXd& s);
  bool ls_single_no_bias(const UwbDataPerTag& dict_uwb_data, const PositionBufferDict_t& dict_p_UinG_buffer, Eigen::MatrixXd& A,
                         Eigen::VectorXd& b, Eigen::VectorXd& s);
  bool ls_double_const_bias(const UwbDataPerTag& dict_uwb_data, const PositionBufferDict_t& dict_p_UinG_buffer,
                            Eigen::MatrixXd& A, Eigen::VectorXd& b, Eigen::VectorXd& s);
  bool ls_double_no_bias(const UwbDataPerTag& dict_uwb_data, const PositionBufferDict_t& dict_p_UinG_buffer, Eigen::MatrixXd& A,
                         Eigen::VectorXd& b, Eigen::VectorXd& s);

  ///
  /// \brief find_pivot_idx Find pivot index (minimize weight uwb_dist^2*sigma_d + p_UinG'*sigma_p*p_UinG)
  /// \param uwb_data
  /// \param p_UinG_buffer
  /// \return
  ///
  std::pair<uint, double> find_pivot_idx(const TimedBuffer<UwbData>& uwb_data, const PositionBuffer& p_UinG_buffer);

};  // class ls_solver
}  // namespace uwb_init

#endif  // LS_SOLVER_HPP_
