// Copyright (C) 2022 Giulio Delama.
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
// You can contact the authors at <giulio.delama@aau.at>

#ifndef WPS_GEN_HPP_
#define WPS_GEN_HPP_

#include "logger/logger.hpp"
#include "options/planner_options.hpp"

using Eigen::seqN;
using tens = std::vector<Eigen::MatrixXd>;
using vec = std::vector<double>;
using mat = std::vector<vec>;
const double pi = 3.14159265359;

namespace uwb_init
{
class OptWpsGenerator
{
public:
  /**
   * @brief OptWpsGenerator
   *
   * @param logger
   * @param nls_solver_options
   */
  OptWpsGenerator(const std::shared_ptr<Logger> logger, std::unique_ptr<PlannerOptions>&& planner_options);

  /**
   * @brief generate_wps
   * @param UWBs
   * @param p_k
   * @return
   */
  Eigen::MatrixXd generate_wps(const Eigen::MatrixXd& UWBs, const Eigen::Vector3d& p_k);

  /**
   * @brief get_cost
   * @return
   */
  double get_cost();

private:
  /// Shared pointer to logger
  std::shared_ptr<Logger> logger_ = nullptr;

  /// OptWpsGenerator options
  std::unique_ptr<PlannerOptions> planner_options_ = nullptr;

  /// Cost value
  double cost_;

  void sortrows(Eigen::MatrixXd& A_nx3, int idx);

  Eigen::VectorXd repelem(Eigen::VectorXd V, Eigen::VectorXd N);

  Eigen::MatrixXd RndI(int LO, int HI, uint n_r, uint n_c);

  Eigen::MatrixXd Rnd(double LO, double HI, uint n_r, uint n_c);

  double dp(Eigen::MatrixXd P, Eigen::MatrixXd UWBs);

  double fitness_calculation(Eigen::MatrixXd chromosome, tens R, const Eigen::MatrixXd& UWBs, uint l_c,
                             const Eigen::VectorXd& p_k);

  Eigen::MatrixXd genrate_cube(double xs_2, double ys_2, double zs_2);

  tens genInitSet();

  bool selection(tens& selcted_chrom_for_reprodoction, tens& elit, tens& selcted_chrom_for_mutaion, tens pop,
                 Eigen::VectorXd Fit, uint c_n, uint m_n, uint e_n);

  void crossover(tens& selcted_chrom_for_reprodoction, uint l_c, uint c_n);

  void mutation(tens& selcted_chrom_for_mutaion, uint m_n, uint l_c);

  Eigen::MatrixXd GA(const Eigen::MatrixXd& UWBs, const Eigen::Vector3d& p_k);
};

}  // namespace uwb_init

#endif  // WPS_GEN_HPP_
