// Copyright (C) 2021 Farhad Shamsfakhr, Giulio Delama.
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
// You can contact the authors at <farhad.shamsfakhr@unitn.it> and
// <giulio.delama@aau.at>

#include <assert.h>

#include "planners/wps_gen.hpp"

namespace uwb_init
{
OptWpsGenerator::OptWpsGenerator(const std::shared_ptr<Logger> logger,
                                 std::unique_ptr<PlannerOptions>&& planner_options)
  : logger_(std::move(logger)), planner_options_(std::move(planner_options))
{
  // Debug assertation
  assert(logger_ != nullptr);
  assert(planner_options_ != nullptr);

  // Logging
  logger_->info("OptWpsGenerator: Initialized");
  std::stringstream ss;
  ss << "OptWpsGenerator options: C_e = " << planner_options_->C_e_.transpose() << '\n'
     << "OptWpsGenerator options: C_v = " << planner_options_->C_v_.transpose() << '\n';
  logger_->debug("OptWpsGenerator options: cell_len = " + std::to_string(planner_options_->cell_len_));
  logger_->debug("OptWpsGenerator options: pop_size = " + std::to_string(planner_options_->pop_size_));
  logger_->debug("OptWpsGenerator options: max_iter = " + std::to_string(planner_options_->itr_num_));
  logger_->debug("OptWpsGenerator options: pc = " + std::to_string(planner_options_->pc_));
  logger_->debug("OptWpsGenerator options: pm = " + std::to_string(planner_options_->pm_));
  logger_->debug("OptWpsGenerator options: x_n = " + std::to_string(planner_options_->x_n_));
  logger_->debug("OptWpsGenerator options: y_n = " + std::to_string(planner_options_->y_n_));
  logger_->debug("OptWpsGenerator options: z_n = " + std::to_string(planner_options_->z_n_));
  logger_->debug("OptWpsGenerator options: side_x = " + std::to_string(planner_options_->side_x_));
  logger_->debug("OptWpsGenerator options: side_y = " + std::to_string(planner_options_->side_y_));
  logger_->debug("OptWpsGenerator options: side_z = " + std::to_string(planner_options_->side_z_));
  logger_->debug("OptWpsGenerator options: z_min = " + std::to_string(planner_options_->z_min_));
  logger_->debug("OptWpsGenerator options: x_s = " + std::to_string(planner_options_->x_s_));
  logger_->debug("OptWpsGenerator options: y_s = " + std::to_string(planner_options_->y_s_));
  logger_->debug("OptWpsGenerator options: z_s = " + std::to_string(planner_options_->z_s_));
  logger_->debug(ss.str());
}

Eigen::MatrixXd OptWpsGenerator::generate_wps(const Eigen::MatrixXd& UWBs, const Eigen::Vector3d& p_k)
{
  Eigen::MatrixXd UWBs_trnsfrm(UWBs.rows(), 3);
  UWBs_trnsfrm << UWBs.rowwise() + (planner_options_->C_v_ - planner_options_->C_e_).transpose();
  Eigen::Vector3d p_k_trnsfrm;
  p_k_trnsfrm << p_k + planner_options_->C_v_ - planner_options_->C_e_;

  Eigen::MatrixXd p_optimal = GA(UWBs_trnsfrm, p_k_trnsfrm);
  p_optimal << p_optimal.rowwise() - (planner_options_->C_v_ - planner_options_->C_e_).transpose();

  const double inf = 1e+10;
  long K = p_optimal.rows();
  Eigen::MatrixXd Temp = p_optimal;
  Eigen::VectorXd tmp = p_optimal.row(K - 1);
  Eigen::MatrixXd final_p_optimal(p_optimal.rows(), 3);
  final_p_optimal.row(0) = p_optimal.row(K - 1);
  Temp.row(K - 1) << inf, inf, inf;
  Eigen::VectorXd dist;
  for (uint i = 0; i < K - 1; i++)
  {
    dist = (((Temp.rowwise() - tmp.transpose()).array().pow(2)).rowwise().sum().sqrt());
    auto it = std::min_element(dist.begin(), dist.end());
    long nmn = std::distance(std::begin(dist), it);
    tmp = Temp.row(nmn);
    final_p_optimal.row(i + 1) = tmp;
    Temp.row(nmn) << inf, inf, inf;
  }

  logger_->info("OptWpsGenerator: Computed optimal waypoints");

  std::stringstream ss;
  ss << "Optimal Waypoints: \n" << final_p_optimal << '\n';
  logger_->debug(ss.str());
  return final_p_optimal;
}

void OptWpsGenerator::sortrows(Eigen::MatrixXd& A_nx3, int idx)
{
  std::vector<Eigen::VectorXd> vec;
  for (uint i = 0; i < A_nx3.rows(); ++i)
    vec.push_back(A_nx3.row(i));

  std::sort(vec.begin(), vec.end(),
            [idx](Eigen::VectorXd const& t1, Eigen::VectorXd const& t2) { return t1(idx) < t2(idx); });

  for (uint i = 0; i < A_nx3.rows(); ++i)
    A_nx3.row(i) = vec[i];
};

Eigen::VectorXd OptWpsGenerator::repelem(Eigen::VectorXd V, Eigen::VectorXd N)
{
  Eigen::VectorXd out(uint(N.array().sum()));
  uint c = 0;
  for (uint i = 0; i < V.rows(); i++)
  {
    for (int j = 0; j < N[i]; j++)
    {
      out(c) = V(i);
      c++;
    }
  }
  return out;
}

Eigen::MatrixXd OptWpsGenerator::RndI(int LO, int HI, uint n_r, uint n_c)
{
  int range = HI - LO;
  Eigen::MatrixXd m = Eigen::MatrixXd::Random(n_r, n_c);
  m = (m + Eigen::MatrixXd::Constant(n_r, n_c, 1.)) * range / 2;
  m = (m + Eigen::MatrixXd::Constant(n_r, n_c, LO));
  return m.array().round();
}

Eigen::MatrixXd OptWpsGenerator::Rnd(double LO, double HI, uint n_r, uint n_c)
{
  double range = HI - LO;
  Eigen::MatrixXd m = Eigen::MatrixXd::Random(n_r, n_c);
  m = (m + Eigen::MatrixXd::Constant(n_r, n_c, 1.)) * range / 2.0;
  m = (m + Eigen::MatrixXd::Constant(n_r, n_c, LO));
  return m.array();
}

double OptWpsGenerator::dp(Eigen::MatrixXd P, Eigen::MatrixXd UWBs)
{
  Eigen::VectorXd d(P.rows(), 1);
  Eigen::MatrixXd L(P.rows(), P.cols());
  Eigen::VectorXd ones(P.rows());
  ones.setOnes();
  Eigen::MatrixXd H(P.rows(), P.cols() + 1);
  double E = 0.0;
  for (uint i = 0; i < UWBs.rows(); i++)
  {
    L = P.rowwise() - UWBs.row(i);
    d = ((L.array().pow(2)).rowwise().sum().sqrt());
    H << L.array().colwise() / d.array(), ones;
    E = E + sqrt(((H.transpose() * H).inverse()).trace());
  }
  return E / UWBs.rows();
}

double OptWpsGenerator::fitness_calculation(Eigen::MatrixXd chromosome, tens R, const Eigen::MatrixXd& UWBs, uint l_c,
                                            const Eigen::VectorXd& p_k)
{
  Eigen::MatrixXd P(l_c + 1, 3);
  for (uint id = 0; id < l_c; id++)
  {
    P.row(id) << R[id](0, int(chromosome(id, 0))), R[id](1, int(chromosome(id, 1))), R[id](2, int(chromosome(id, 2)));
  }
  P.row(l_c) << p_k.transpose();
  return dp(P, UWBs);
}

Eigen::MatrixXd OptWpsGenerator::genrate_cube(double xs_2, double ys_2, double zs_2)
{
  Eigen::VectorXd a = Eigen::VectorXd::LinSpaced(5, -pi, pi);
  double ph = pi / 4;
  Eigen::MatrixXd P(6, 5);
  Eigen::VectorXd ones(5);
  ones.setOnes();

  P.row(0) = xs_2 + (a.array() + ph).cos() / cos(ph) * xs_2;
  P.row(1) = P.row(0);

  P.row(2) = ys_2 + (a.array() + ph).sin() / sin(ph) * ys_2;
  P.row(3) = P.row(2);

  P.row(4) = zs_2 - ones.array() * zs_2;
  P.row(5) = zs_2 + ones.array() * zs_2;
  return P;
}

tens OptWpsGenerator::genInitSet()
{
  Eigen::VectorXd temp;
  Eigen::MatrixXd xyz =
      genrate_cube(planner_options_->x_s_ / 2, planner_options_->y_s_ / 2, planner_options_->z_s_ / 2);
  tens R = tens(planner_options_->x_n_ * planner_options_->y_n_ * planner_options_->z_n_,
                Eigen::MatrixXd(3, planner_options_->cell_len_));
  uint c = 0;
  for (uint i = 0; i < planner_options_->z_n_; i++)
  {
    for (uint j = 0; j < planner_options_->y_n_; j++)
    {
      for (uint k = 0; k < planner_options_->x_n_; k++)
      {
        temp = Eigen::VectorXd::LinSpaced(planner_options_->cell_len_, xyz(0, 0) + planner_options_->x_s_ * (k),
                                          xyz(0, 1) + planner_options_->x_s_ * (k));
        R[c].row(0) << temp.transpose();
        temp = Eigen::VectorXd::LinSpaced(planner_options_->cell_len_, xyz(2, 0) + planner_options_->y_s_ * (j),
                                          xyz(2, 3) + planner_options_->y_s_ * (j));
        R[c].row(1) << temp.transpose();
        temp = Eigen::VectorXd::LinSpaced(planner_options_->cell_len_, xyz(4, 0) + planner_options_->z_s_ * (i),
                                          xyz(5, 0) + planner_options_->z_s_ * (i));
        R[c].row(2) << temp.transpose();
        c++;
      }
    }
  }
  return R;
}

bool OptWpsGenerator::selection(tens& selcted_chrom_for_reprodoction, tens& elit, tens& selcted_chrom_for_mutaion,
                                tens pop, Eigen::VectorXd Fit, uint c_n, uint m_n, uint e_n)
{
  bool flag_ext = false;
  double F_max = Fit.maxCoeff();
  double F_min = Fit.minCoeff();
  if ((F_max - F_min) < 0.001)
  {
    flag_ext = true;
  }
  else
  {
    Eigen::VectorXd N = (10 * (1 - 0.1) * ((F_max - Fit.array()) / (F_max - F_min)) + 0.1).array().ceil();
    Eigen::VectorXd U =
        repelem(Eigen::VectorXd::LinSpaced(planner_options_->pop_size_, 0, planner_options_->pop_size_ - 1), N);
    std::random_shuffle(U.begin(), U.end());
    Eigen::VectorXd selcted_id = U.head(c_n);
    for (size_t i = 0; i < c_n; i++)
    {
      selcted_chrom_for_reprodoction[i] = pop[selcted_id(i)];
    }
    Eigen::MatrixXd sortedPop(planner_options_->pop_size_, 2);
    sortedPop.col(0) = Eigen::VectorXd::LinSpaced(planner_options_->pop_size_, 0, planner_options_->pop_size_ - 1);
    sortedPop.col(1) = Fit;
    sortrows(sortedPop, 1);
    for (uint i = 0; i < e_n; i++)
    {
      elit[i] = pop[sortedPop(i, 0)];
    }

    std::random_shuffle(U.begin(), U.end());
    selcted_id = U.head(m_n);
    for (uint i = 0; i < m_n; i++)
    {
      selcted_chrom_for_mutaion[i] = pop[selcted_id(i)];
    }
  }
  return flag_ext;
}

void OptWpsGenerator::crossover(tens& selcted_chrom_for_reprodoction, uint l_c, uint c_n)
{
  tens newpop = selcted_chrom_for_reprodoction;
  int cpoint;
  for (uint i = 0; i < c_n; i = i + 2)
  {
    int n = round(l_c / 2);
    cpoint = (rand() % (n + 1)) + 2;
    Eigen::MatrixXd temp_i = selcted_chrom_for_reprodoction[i];
    Eigen::MatrixXd temp_ii = selcted_chrom_for_reprodoction[i + 1];
    selcted_chrom_for_reprodoction[i] << temp_i(Eigen::seq(0, cpoint, 1), Eigen::all),
        temp_ii(Eigen::seq(cpoint + 1, l_c - 1, 1), Eigen::all);
    selcted_chrom_for_reprodoction[i + 1] << temp_ii(Eigen::seq(0, cpoint, 1), Eigen::all),
        temp_i(Eigen::seq(cpoint + 1, l_c - 1, 1), Eigen::all);
  }
}

void OptWpsGenerator::mutation(tens& selcted_chrom_for_mutaion, uint m_n, uint l_c)
{
  for (uint i = 0; i < m_n; i++)
  {
    Eigen::MatrixXd temp_i = selcted_chrom_for_mutaion[i];
    uint indx = (rand() % (l_c));
    selcted_chrom_for_mutaion[i].row(indx) = RndI(0, planner_options_->cell_len_ - 1, 1, 3);
  }
}

Eigen::MatrixXd OptWpsGenerator::GA(const Eigen::MatrixXd& UWBs, const Eigen::Vector3d& p_k)
{
  tens R = genInitSet();
  uint l_c = R.size();
  Eigen::VectorXd Fit(planner_options_->pop_size_);
  tens pop;
  Eigen::MatrixXd chromosome;
  for (uint i = 0; i < planner_options_->pop_size_; i++)
  {
    chromosome = RndI(0, planner_options_->cell_len_ - 1, l_c, 3);
    Fit(i) = fitness_calculation(chromosome, R, UWBs, l_c, p_k);
    pop.push_back(chromosome);
  }

  uint c_n = floor(planner_options_->pc_ * planner_options_->pop_size_);
  uint m_n = floor(planner_options_->pm_ * planner_options_->pop_size_);
  if ((c_n % 2) != 0)
    c_n++;
  uint e_n = floor(planner_options_->pop_size_ - c_n - m_n);
  tens selcted_chrom_for_reprodoction(c_n, Eigen::MatrixXd(l_c, 3));
  tens selcted_chrom_for_mutaion(m_n, Eigen::MatrixXd(l_c, 3));
  tens elit(e_n, Eigen::MatrixXd(l_c, 3));
  tens newpop;
  Eigen::MatrixXd Best_C;
  bool flag_ext;

  for (uint itr = 0; itr < planner_options_->itr_num_; itr++)
  {
    flag_ext = selection(selcted_chrom_for_reprodoction, elit, selcted_chrom_for_mutaion, pop, Fit, c_n, m_n, e_n);
    if (flag_ext)
      break;
    else
    {
      crossover(selcted_chrom_for_reprodoction, l_c, c_n);
      mutation(selcted_chrom_for_mutaion, m_n, l_c);
      newpop.insert(newpop.end(), selcted_chrom_for_reprodoction.begin(), selcted_chrom_for_reprodoction.end());
      newpop.insert(newpop.end(), selcted_chrom_for_mutaion.begin(), selcted_chrom_for_mutaion.end());
      newpop.insert(newpop.end(), elit.begin(), elit.end());
      pop = newpop;
      newpop.clear();

      for (uint i = 0; i < planner_options_->pop_size_; i++)
      {
        chromosome = pop[i];
        Fit(i) = fitness_calculation(chromosome, R, UWBs, l_c, p_k);
      }
      auto it = std::min_element(Fit.begin(), Fit.end());
      uint nmn = std::distance(std::begin(Fit), it);
      Best_C = pop[nmn];
      cost_ = *it;
    }
  }
  Eigen::MatrixXd P(l_c + 1, 3);
  for (uint id = 0; id < l_c; id++)
  {
    P.row(id) << R[id](0, uint(Best_C(id, 0))), R[id](1, uint(Best_C(id, 1))), R[id](2, uint(Best_C(id, 2)));
  }
  P.row(l_c) << p_k.transpose();
  return P;
}

}  // namespace uwb_init
