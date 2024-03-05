// Copyright (C) 2021 Alessandro Fornasier, Giulio Delama.
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

#include "solvers/linear/ls_solver.hpp"

namespace uwb_init
{
LsSolver::LsSolver(const std::shared_ptr<Logger> logger,
                   std::unique_ptr<LsSolverOptions>&& ls_solver_options)
  : logger_(std::move(logger)), solver_options_(std::move(ls_solver_options))
{
  // Debug assertation
  assert(logger_ != nullptr);
  assert(solver_options_ != nullptr);

  // Configure solver
  configure(solver_options_->init_method_, solver_options_->bias_type_);

  // Logging
  logger_->info("LsSolver: Initialized");
  logger_->debug("LsSolver options: sigma_pos = " + std::to_string(solver_options_->sigma_pos_));
  logger_->debug("LsSolver options: sigma_meas = " + std::to_string(solver_options_->sigma_meas_));
  logger_->debug("LsSolver options: use_RANSAC = " + std::to_string(solver_options_->use_RANSAC_));
  logger_->debug("LsSolver options: bias_type = " + BiasType_to_string(solver_options_->bias_type_));
  logger_->debug("LsSolver options: init_method = " + std::string(InitMethodString(solver_options_->init_method_)));
  logger_->debug("LsSolver options: ransac_opts = " + solver_options_->ransac_opts_.str());

  std::random_device rd; // obtain a random number from hardware
  ptr_gen_.reset(new std::mt19937(rd()));

}

void LsSolver::configure(const BiasType bias_type)
{
  configure(solver_options_->init_method_, bias_type);
}

void LsSolver::configure(const InitMethod init_method)
{
  configure(init_method, solver_options_->bias_type_);
}

void LsSolver::configure(InitMethod const init_method, BiasType const bias_type)
{
  solver_options_->init_method_ = init_method;
  solver_options_->bias_type_ = bias_type;

  // Bind LS-problem initialization function based on selected method and model variables
  switch (solver_options_->init_method_)
  {
    // Single measurement initialization method
    case InitMethod::SINGLE:
      switch (solver_options_->bias_type_)
      {
        // Unbiased measurement model (only distance between tag and uwb anchor)
        case BiasType::NO_BIAS:
          ls_problem = std::bind(&LsSolver::ls_single_no_bias, this, std::placeholders::_1, std::placeholders::_2,
                                 std::placeholders::_3, std::placeholders::_4, std::placeholders::_5);
          break;
        // Constant bias measurement model (distance and constant bias)
        case BiasType::CONST_BIAS:
          // fall through
        case BiasType::ALL_BIAS:
          ls_problem = std::bind(&LsSolver::ls_single_const_bias, this, std::placeholders::_1, std::placeholders::_2,
                                 std::placeholders::_3, std::placeholders::_4, std::placeholders::_5);
          break;
      }
      break;
    // Double measurements initialization method
    case InitMethod::DOUBLE:
      switch (solver_options_->bias_type_)
      {
        // Unbiased measurement model (only distance between tag and uwb anchor)
        case BiasType::NO_BIAS:
          ls_problem = std::bind(&LsSolver::ls_double_no_bias, this, std::placeholders::_1, std::placeholders::_2,
                                 std::placeholders::_3, std::placeholders::_4, std::placeholders::_5);
          break;
        // Constant bias measurement model (distance and constant bias)
        case BiasType::CONST_BIAS:
          // fall through
        case BiasType::ALL_BIAS:
          ls_problem = std::bind(&LsSolver::ls_single_const_bias, this, std::placeholders::_1, std::placeholders::_2,
                                 std::placeholders::_3, std::placeholders::_4, std::placeholders::_5);
          break;
      }
      break;
  }

  // Logging
  logger_->info("LsSolver: Configured");
}

BiasType LsSolver::bias_tpye() { return solver_options_->bias_type_; }

InitMethod LsSolver::init_method() { return solver_options_->init_method_; }

bool LsSolver::solve_ls(const TimedBuffer<UwbData>& uwb_data, const PositionBuffer& p_UinG_buffer,
                        Eigen::VectorXd& lsSolution, Eigen::MatrixXd& cov)
{
  // Coefficient matrix and measurement vector initialization
  Eigen::MatrixXd coeffs;  // A
  Eigen::VectorXd meas;    // b
  Eigen::VectorXd sigma;   // s

  UwbDataPerTag dict_uwb_data;
  dict_uwb_data.insert({(uint)0, uwb_data});
  PositionBufferDict_t dict_p_UinG_buffer;
  dict_p_UinG_buffer.insert({(uint)0, p_UinG_buffer});

  // Initialize least squares problem
  if (!(ls_problem(dict_uwb_data, dict_p_UinG_buffer, coeffs, meas, sigma)))
  {
    logger_->err("LsSolver: Cannot build Least Squares problem");
    return false;
  }

  // Compute weighted coefficients
  Eigen::MatrixXd W = sigma.cwiseInverse().cwiseSqrt().asDiagonal();
  Eigen::MatrixXd weighted_coeffs = W * coeffs;
  Eigen::VectorXd weighted_meas = W * meas;

  // Solve GLS
  Eigen::BDCSVD<Eigen::MatrixXd> svd(weighted_coeffs, Eigen::ComputeThinU | Eigen::ComputeThinV);
  lsSolution = svd.solve(weighted_meas);

  /* lsSolution changes w.r.t. chosen method and variables
 //
 // Const bias single:
 // [    0   ,     1   ,    2    ,  3 ,      4            ]
 // [p_AinG_x, p_AinG_y, p_AinG_z,  k , norm(p_AinG)^2-k^2]
 //
 // Const bias double:
 // [    0   ,     1   ,    2    ,  3 ]
 // [p_AinG_x, p_AinG_y, p_AinG_z,  k ]
 //
 // Unbiased single:
 // [    0    ,    1   ,    2   ,        3       ]
 // [p_AinG_x, p_AinG_y, p_AinG_z, norm(p_AinG)^2]
 //
 // Unbiased double:
 // [    0    ,    1   ,    2    ]
 // [p_AinG_x, p_AinG_y, p_AinG_z]
 */

  // Compute estimation Covariance (Var(X) = (A'*sigma^-1*A)^-1 = ((W*A)'*(W*A))^-1) = (A_'*A_)^-1)
  // if A_ = U*S*V' then (A_'*A_)^-1 = V*S^-1*S^-1*V' (see properties of SVD)
  cov = svd.matrixV() * svd.singularValues().asDiagonal().inverse() * svd.singularValues().asDiagonal().inverse() *
        svd.matrixV().transpose();

  // If covariance matrix is not semi-positive-definite return
  if (solver_options_->check_cov_ && !isSPD(cov))
  {
    logger_->err("LsSolver: Covariance matrix is not SPD");
    return false;
  }

  return true;
}

bool LsSolver::solve_ls(const UwbDataPerTag &dict_uwb_data, PositionBufferDict_t const& dict_p_UinG_buffer, Eigen::VectorXd &lsSolution, Eigen::MatrixXd &cov) {
  if(dict_uwb_data.size() < 1) {
    logger_->err("LsSolver::solve_ls: empty uwb range buffer obtained");
    return false;
  }
  if(dict_uwb_data.size() > dict_p_UinG_buffer.size())
  {
    logger_->err("LsSolver::solve_ls: uwb range buffer and position buffer do not match");
    return false;
  }


  // Coefficient matrix and measurement vector initialization
  Eigen::MatrixXd coeffs;  // A
  Eigen::VectorXd meas;    // b
  Eigen::VectorXd sigma;   // s

          // Initialize least squares problem
  if (!(ls_problem(dict_uwb_data, dict_p_UinG_buffer, coeffs, meas, sigma)))
  {
    logger_->err("LsSolver: Cannot build Least Squares problem");
    return false;
  }

          // Compute weighted coefficients
  Eigen::MatrixXd W_diag = sigma.cwiseInverse().cwiseSqrt();
  Eigen::MatrixXd W = W_diag.asDiagonal();
  Eigen::MatrixXd weighted_coeffs = W * coeffs;
  Eigen::VectorXd weighted_meas = W * meas;

          // Solve GLS
  Eigen::BDCSVD<Eigen::MatrixXd> svd(weighted_coeffs, Eigen::ComputeThinU | Eigen::ComputeThinV);
  lsSolution = svd.solve(weighted_meas);

  /* lsSolution changes w.r.t. chosen method and variables
  //
  // Const bias single:
  // [    0   ,     1   ,    2    ,  3 ,  4  , ... ,  end            ]
  // [p_AinG_x, p_AinG_y, p_AinG_z,  k_1, k_2, ... , norm(p_AinG)^2-k^2]
  //
  // Const bias double:
  // [    0   ,     1   ,    2    ,  3  , ... , end ]
  // [p_AinG_x, p_AinG_y, p_AinG_z,  k_1, ... , k_n ]
  //
  // Unbiased single:
  // [    0    ,    1   ,    2   ,        3       ]
  // [p_AinG_x, p_AinG_y, p_AinG_z, norm(p_AinG)^2]
  //
  // Unbiased double:
  // [    0    ,    1   ,    2    ]
  // [p_AinG_x, p_AinG_y, p_AinG_z]
  */

          // Compute estimation Covariance (Var(X) = (A'*sigma^-1*A)^-1 = ((W*A)'*(W*A))^-1) = (A_'*A_)^-1)
          // if A_ = U*S*V' then (A_'*A_)^-1 = V*S^-1*S^-1*V' (see properties of SVD)
  cov = svd.matrixV() * svd.singularValues().asDiagonal().inverse() * svd.singularValues().asDiagonal().inverse()
        * svd.matrixV().transpose();

          // If covariance matrix is not semi-positive-definite return
  if (!isSPD(cov)) {
    logger_->err("LsSolver: Covariance matrix is not SPD");
    return false;
  }
  return true;

}

bool uwb_init::LsSolver::solve_ls(const UwbDataPerTag &dict_uwb_data, const PositionBufferDict_t &dict_p_UinG_buffer,
                                  Eigen::VectorXd &lsSolution, Eigen::MatrixXd &cov, UwbDataPerTag &dict_uwb_inliers)
{

  // if RANSAC is disabled, use default method
  if(!solver_options_->use_RANSAC_)
  {
    dict_uwb_inliers = dict_uwb_data;
    return solve_ls(dict_uwb_data, dict_p_UinG_buffer, lsSolution, cov);
  }

  lsSolution.setZero(3);
  cov.setZero(3,3);
  std::vector<double> costs;
  std::vector<Eigen::VectorXd> lsSolutions;
  std::vector<Eigen::MatrixXd> Sigmas;

  costs.reserve(solver_options_->ransac_opts_.n);
  lsSolutions.reserve(solver_options_->ransac_opts_.n);


  if(has_Tag_enough_samples(dict_uwb_data)) {
    logger_->info("LsSolver::solve_ls: enough data collected for RANSAC");
  }

  // Repeat the random sampling n-time, hoping that we pick once data without outliers!
  for(size_t iter=0; iter < solver_options_->ransac_opts_.n; iter++)
  {
    // 1) take random from measurement and hope that it does not contain outlier
    auto dict_sampled_data = LsSolver::rand_samples(dict_uwb_data, solver_options_->ransac_opts_.s, ptr_gen_);

    // 2) solve the problem:
    Eigen::VectorXd lsSolution_i;
    Eigen::MatrixXd Sigma_i;
    if(solve_ls(dict_sampled_data, dict_p_UinG_buffer, lsSolution_i, Sigma_i))
    {
      double cost_i = LsSolver::compute_cost(dict_uwb_data, dict_p_UinG_buffer, lsSolution_i, solver_options_->bias_type_);
      costs.push_back(cost_i);
      lsSolutions.push_back(lsSolution_i);
      Sigmas.push_back(Sigma_i);
      logger_->debug("LsSolver::solve_ls: RANSAC iter=" + std::to_string(iter) + ", cost=" + std::to_string(cost_i));
    }
  }

  if(costs.empty()) {
    logger_->warn("LsSolver::solve_ls: RANSAC: no costs computed!");
    return false;
  }

  // 3) find the best model:
  auto iter_min_cost = std::min_element(costs.begin(), costs.end());
  size_t idx_best = iter_min_cost - costs.begin();
  if(logger_->getlevel() == LoggerLevel::FULL)
  {
    std::stringstream ss;
    ss << lsSolutions[idx_best].transpose();
    logger_->debug("LsSolver::solve_ls: RANSAC: best model at iter=" + std::to_string(idx_best) + ", cost=" + std::to_string(*iter_min_cost) + " is x_est=" + ss.str());
  }

  // 4) remove outliers that fall outside the 99% of the distribution 2*(sigma_uwb+sigma_pos) using all measurement and best match
  double threshold = (solver_options_->sigma_meas_ + solver_options_->sigma_pos_)*solver_options_->ransac_opts_.thres_num_std;

  std::pair<UwbDataPerTag, size_t> uwb_data_clean = LsSolver::remove_ouliers(dict_uwb_data, dict_p_UinG_buffer, lsSolutions[idx_best], solver_options_->bias_type_, threshold, solver_options_->ransac_opts_.n);
  logger_->debug("LsSolver::solve_ls: [" + std::to_string(uwb_data_clean.second) + "] outliers removed above threshold=" + std::to_string(threshold));
  dict_uwb_inliers = uwb_data_clean.first;

  // 5) fit a new model on the cleaned data:
  if(solve_ls(dict_uwb_inliers, dict_p_UinG_buffer, lsSolution, cov)) {
    if(logger_->getlevel() == LoggerLevel::FULL)
    {
      double cost_final = LsSolver::compute_cost(dict_uwb_inliers, dict_p_UinG_buffer, lsSolution, solver_options_->bias_type_);
      std::stringstream ss;
      ss << lsSolution.transpose();
      logger_->debug("LsSolver::solve_ls: final cost=" + std::to_string(cost_final) + " for x_best=" + ss.str());
    }
    return true;
  }
  else if(lsSolutions.size()) {
    lsSolution = lsSolutions[idx_best];
    cov = Sigmas[idx_best];
    return true;
  } else {
    return false;
  }

}

bool LsSolver::has_Tag_enough_samples(const UwbDataPerTag &dict_uwb_data) {
  std::unordered_map<uint, bool> dict = LsSolver::has_Tag_enough_samples(dict_uwb_data,
                                                                         solver_options_->ransac_opts_.num_samples_needed());

  bool res = true;
  for(auto const& e : dict) {
    if(!e.second) {
      res = false;
      logger_->debug("LsSolver::has_Tag_enough_samples(): more samples needed for ransac! TagID=[" + std::to_string(e.first) +
                     "] ! min=" + std::to_string(solver_options_->ransac_opts_.num_samples_needed()) + ", currenty=" +
                     std::to_string(dict_uwb_data.at(e.first).size()));
    }
  }
  return res;
}

std::unordered_map<uint, bool> LsSolver::has_Tag_enough_samples(const UwbDataPerTag &dict_uwb_data,
                                                                const size_t num_samples) {
  std::unordered_map<uint, bool> dict_enough;
  for(auto const&e : dict_uwb_data)
  {
    uint const Tag_ID = e.first;
    if(num_samples < e.second.size()){
      dict_enough[Tag_ID] = true;
    } else {
      dict_enough[Tag_ID] = false;
    }
  }
  return dict_enough;
}

UwbDataPerTag uwb_init::LsSolver::rand_samples(const UwbDataPerTag &dict_uwb_data, const size_t num_samples,
                                               std::shared_ptr<std::mt19937> ptr_gen)
{
  UwbDataPerTag dict_sampled_data;
  for(auto const&e : dict_uwb_data)
  {
    // only if we have enough samples to fit the model, otherwise ignore Tag_ID:
    if(num_samples < e.second.size())
    {
      auto indices = randperm(num_samples, e.second.size(), *ptr_gen);

      //for(auto &i : indices) {
      //  std::cout << i << ",";
      //}
      //std::cout << std::endl;

      uint const Tag_ID = e.first;
      TimedBuffer<UwbData> const& data = e.second;
      TimedBuffer<UwbData> sampled_data;
      for(auto idx : indices) {
        std::pair<double, UwbData> const& range_i = data.get_buffer()[idx];
        sampled_data.push_back(range_i.first, range_i.second);
      }

      dict_sampled_data[Tag_ID] = sampled_data;
    }
  }
  return  dict_sampled_data;
}

double LsSolver::compute_cost(const UwbDataPerTag &dict_uwb_data,
                              const PositionBufferDict_t &dict_p_UinG_buffer,
                              const Eigen::VectorXd &x_est,
                              const BiasType bias_type)
{
  size_t const num_tags = dict_uwb_data.size();
  size_t idx_tag = 0;
  uint j = 0;  // num measurements
  double abs_sum = 0.0;
  for(auto const&e : dict_uwb_data)
  {
    uint const Tag_ID = e.first;
    auto const& uwb_data = e.second;
    auto const& p_UinG_buffer = dict_p_UinG_buffer.at(Tag_ID);

    for (uint i = 0; i < uwb_data.size(); ++i)
    {
      // Get position at uwb timestamp
      Eigen::Vector3d p_UinG = p_UinG_buffer.get_at_timestamp(uwb_data[i].first);

      Eigen::Vector3d p_AinG(x_est(0), x_est(1), x_est(2));
      double gamma_i = 0.0;
      double beta_i = 1.0;
      switch(bias_type) {
        case BiasType::NO_BIAS:
        {
          break;
        }
        case BiasType::CONST_BIAS:
        {
          gamma_i = x_est(3 + idx_tag);
          break;
        }
        case BiasType::ALL_BIAS:
        {
          gamma_i = x_est(3 + idx_tag);
          beta_i = x_est(3 + num_tags + idx_tag);
          break;
        }
      }

      Eigen::Vector3d p_UtoA = p_UinG - p_AinG;
      double d_est = beta_i * p_UtoA.norm() + gamma_i;
      abs_sum += std::abs(d_est - uwb_data[i].second.distance_);
      j += 1;
    }
    idx_tag++;
  }
  return abs_sum/(1.0*j);
}

std::pair<UwbDataPerTag, size_t> LsSolver::remove_ouliers(const UwbDataPerTag &dict_uwb_data, const PositionBufferDict_t &dict_p_UinG_buffer, const Eigen::VectorXd &x_est, const BiasType bias_type, double threshold, const size_t min_num_samples) {
  UwbDataPerTag dict_clean;

  size_t const num_tags = dict_uwb_data.size();
  size_t idx_tag = 0;
  size_t num_outliers = 0;
  for(auto const&e : dict_uwb_data)
  {
    uint const Tag_ID = e.first;
    auto const& uwb_data = e.second;
    auto const& p_UinG_buffer = dict_p_UinG_buffer.at(Tag_ID);

    TimedBuffer<UwbData> uwb_data_clean;
    for (uint i = 0; i < uwb_data.size(); ++i)
    {
      // Get position at uwb timestamp
      Eigen::Vector3d p_UinG = p_UinG_buffer.get_at_timestamp(uwb_data[i].first);

      Eigen::Vector3d p_AinG(x_est(0), x_est(1), x_est(2));
      double gamma_i = 0.0;
      double beta_i = 1.0;
      switch(bias_type) {
        case BiasType::NO_BIAS:
        {
          break;
        }
        case BiasType::CONST_BIAS:
        {
          gamma_i = x_est(3 + idx_tag);
          break;
        }
        case BiasType::ALL_BIAS:
        {
          gamma_i = x_est(3 + idx_tag);
          beta_i = x_est(3 + num_tags + idx_tag);
          break;
        }
      }

      Eigen::Vector3d p_UtoA = p_UinG - p_AinG;
      double d_est = beta_i * p_UtoA.norm() + gamma_i;

      // gating: check if error is below a certain threshold
      if (std::abs(d_est - uwb_data[i].second.distance_) < threshold)
      {
        // insert good value:
        uwb_data_clean.push_back(uwb_data[i].first, uwb_data[i].second);
      }
      else {
        num_outliers++;
      }
    }

    // add all good values for one tag to the dictionary
    if(uwb_data_clean.size() > min_num_samples) {
      dict_clean[Tag_ID] = uwb_data_clean;
    }
    idx_tag++;
  }

  return std::make_pair(dict_clean, num_outliers);
}

bool LsSolver::ls_single_const_bias(const UwbDataPerTag &dict_uwb_data, const PositionBufferDict_t& dict_p_UinG_buffer,
                                    Eigen::MatrixXd& A, Eigen::VectorXd& b, Eigen::VectorXd& s)
{
  size_t const num_tags = dict_uwb_data.size();
  size_t num_meas = 0;
  for(auto const&e : dict_uwb_data)
  {
    num_meas += e.second.size();
  }
  // Const bias single:
  // [    0   ,     1   ,    2    ,  3 ,  4  , ... ,  end            ]
  // [p_AinG_x, p_AinG_y, p_AinG_z,  k_1, k_2, ... , norm(p_AinG)^2-k^2]

  // Coefficient matrix and measurement vector initialization
  A = Eigen::MatrixXd::Zero(num_meas, 4 + num_tags);
  b = Eigen::VectorXd::Zero(A.rows());
  s = Eigen::VectorXd::Ones(A.rows());

  // Fill the coefficient matrix and the measurement vector

  size_t idx_tag = 0;
  uint j = 0;
  for(auto const&e : dict_uwb_data)
  {
    uint const Tag_ID = e.first;
    auto const& uwb_data = e.second;
    auto const& p_UinG_buffer = dict_p_UinG_buffer.at(Tag_ID);

    for (uint i = 0; i < uwb_data.size(); ++i)
    {
      // Get position at uwb timestamp
      Eigen::Vector3d p_UinG = p_UinG_buffer.get_at_timestamp(uwb_data[i].first);

      // make bias gamma entry for the appropriate tag
      Eigen::VectorXd gamma_vec;
      gamma_vec.setZero(num_tags);
      gamma_vec(idx_tag) = 2 * uwb_data[i].second.distance_;

      // Fill row(i) of A and b
      A.row(j) << -2 * p_UinG.x(), -2 * p_UinG.y(), -2 * p_UinG.z(), gamma_vec.transpose(), 1;
      b(j) = std::pow(uwb_data[i].second.distance_, 2) - std::pow(p_UinG.norm(), 2);

      j += 1;
    }
    idx_tag++;
  }
  return true;
}

bool LsSolver::ls_single_no_bias(const UwbDataPerTag &dict_uwb_data, const PositionBufferDict_t &dict_p_UinG_buffer,
                                 Eigen::MatrixXd& A, Eigen::VectorXd& b, Eigen::VectorXd& s)
{
  size_t num_meas = 0;
  for(auto const&e : dict_uwb_data)
  {
    num_meas += e.second.size();
  }
  // Unbiased single:
  // [    0    ,    1   ,    2   ,        3       ]
  // [p_AinG_x, p_AinG_y, p_AinG_z, norm(p_AinG)^2]

  // Coefficient matrix and measurement vector initialization
  A = Eigen::MatrixXd::Zero(num_meas, 4);
  b = Eigen::VectorXd::Zero(A.rows());
  s = Eigen::VectorXd::Ones(A.rows());

  uint j = 0;
  for(auto const&e : dict_uwb_data)
  {
    uint const Tag_ID = e.first;

    auto const& uwb_data = e.second;
    auto const& p_UinG_buffer = dict_p_UinG_buffer.at(Tag_ID);
    // Fill the coefficient matrix and the measurement vector
    for (uint i = 0; i < uwb_data.size(); ++i)
    {
      // Get position at uwb timestamp
      Eigen::Vector3d p_UinG = p_UinG_buffer.get_at_timestamp(uwb_data[i].first);

      // Fill row(i) of A and b
      A.row(j) << -2 * p_UinG.x(), -2 * p_UinG.y(), -2 * p_UinG.z(), 1;
      b(j) = std::pow(uwb_data[i].second.distance_, 2) - std::pow(p_UinG.norm(), 2);

      j += 1;
    }
  }
  return true;
}

bool LsSolver::ls_double_const_bias(const UwbDataPerTag &dict_uwb_data, const PositionBufferDict_t& dict_p_UinG_buffer,
                                    Eigen::MatrixXd& A, Eigen::VectorXd& b, Eigen::VectorXd& s)
{

  size_t const num_tags = dict_uwb_data.size();
  size_t num_meas = 0;
  for(auto const&e : dict_uwb_data)
  {
    num_meas += e.second.size();
  }

  // Const bias double:
  // [    0   ,     1   ,    2    ,  3  , ... , end]
  // [p_AinG_x, p_AinG_y, p_AinG_z,  k_1, ... , k_n]

  // Number of data points must be at least 2
  if (num_meas < 2)
  {
    return false;
  }

  // Coefficient matrix and measurement vector initialization
  A = Eigen::MatrixXd::Zero(num_meas - num_tags, 3 + num_tags);
  b = Eigen::VectorXd::Zero(A.rows());
  s = Eigen::VectorXd::Zero(A.rows());

  size_t idx_tag = 0;
  uint j = 0;
  for(auto const&e : dict_uwb_data)
  {
    uint const Tag_ID = e.first;
    auto const& uwb_data = e.second;

    auto const& p_UinG_buffer = dict_p_UinG_buffer.at(Tag_ID);
    // Find pivot index (minimize weight uwb_dist^2*sigma_d + p_UinG'*sigma_p*p_UinG)
    std::pair<uint, double> res = find_pivot_idx(uwb_data, p_UinG_buffer);
    uint const pivot_idx = res.first;
    double const weight_pivot = res.second;

            // Position and distance at pivot_index
    Eigen::Vector3d p_UinG_pivot = p_UinG_buffer.get_at_timestamp(uwb_data[pivot_idx].first);
    double uwb_pivot = uwb_data[pivot_idx].second.distance_;

            // Fill the coefficient matrix and the measurement vector
    for (uint i = 0; i < uwb_data.size(); ++i)
    {
      // Skip pivot
      if (i == pivot_idx)
      {
        continue;
      }

              // Get position at timestamp
      Eigen::Vector3d p_UinG = p_UinG_buffer.get_at_timestamp(uwb_data[i].first);


      // make bias gamma entry for the appropriate tag
      Eigen::VectorXd gamma_vec;
      gamma_vec.setZero(num_tags);
      gamma_vec(idx_tag) = (uwb_data[i].second.distance_ - uwb_pivot);


              // Fill row(j) of A and b
      A.row(j) << -(p_UinG.x() - p_UinG_pivot.x()), -(p_UinG.y() - p_UinG_pivot.y()), -(p_UinG.z() - p_UinG_pivot.z()),
        gamma_vec.transpose();

      b(j) = 0.5 * (std::pow(uwb_data[i].second.distance_, 2) - std::pow(uwb_pivot, 2) -
                    (std::pow(p_UinG.norm(), 2) - std::pow(p_UinG_pivot.norm(), 2)));

      s(j) = std::pow(uwb_data[i].second.distance_, 2) * solver_options_->sigma_meas_ +
             p_UinG.transpose() * Eigen::Matrix3d::Identity() * solver_options_->sigma_pos_ * p_UinG + weight_pivot;

              // Increment row index
      j += 1;
    }

    idx_tag += 1;
  }

  return true;
}

bool LsSolver::ls_double_no_bias(const UwbDataPerTag &dict_uwb_data, const PositionBufferDict_t &dict_p_UinG_buffer,
                                 Eigen::MatrixXd& A, Eigen::VectorXd& b, Eigen::VectorXd& s)
{
  size_t const num_tags = dict_uwb_data.size();
  size_t num_meas = 0;
  for(auto const&e : dict_uwb_data)
  {
    num_meas += e.second.size();
  }
  // Unbiased double:
  // [    0    ,    1   ,    2    ]
  // [p_AinG_x, p_AinG_y, p_AinG_z]

  // Number of data points must be at least 2
  if (num_meas < 2)
  {
    return false;
  }

  // Coefficient matrix and measurement vector initialization
  A = Eigen::MatrixXd::Zero(num_meas - num_tags, 3);
  b = Eigen::VectorXd::Zero(A.rows());
  s = Eigen::VectorXd::Zero(A.rows());

  size_t idx_tag = 0;
  uint j = 0;
  for(auto const&e : dict_uwb_data)
  {
    uint const Tag_ID = e.first;
    auto const& uwb_data = e.second;

    auto const& p_UinG_buffer = dict_p_UinG_buffer.at(Tag_ID);
    // Find pivot index (minimize weight uwb_dist^2*sigma_d + p_UinG'*sigma_p*p_UinG)
    std::pair<uint, double> res = find_pivot_idx(uwb_data, p_UinG_buffer);
    uint const pivot_idx = res.first;
    double const weight_pivot = res.second;

            // Position and distance at pivot_index
    Eigen::Vector3d p_UinG_pivot = p_UinG_buffer.get_at_timestamp(uwb_data[pivot_idx].first);
    double uwb_pivot = uwb_data[pivot_idx].second.distance_;

            // Fill the coefficient matrix and the measurement vector
    for (uint i = 0; i < dict_uwb_data.size(); ++i)
    {
      // Skip pivot
      if (i == pivot_idx)
      {
        continue;
      }

              // Get position at uwb timestamp
      Eigen::Vector3d p_UinG = p_UinG_buffer.get_at_timestamp(uwb_data[i].first);

              // Fill row(j) of A and b
      A.row(j) << -(p_UinG.x() - p_UinG_pivot.x()), -(p_UinG.y() - p_UinG_pivot.y()), -(p_UinG.z() - p_UinG_pivot.z());

      b(j) = 0.5 * (std::pow(uwb_data[i].second.distance_, 2) - std::pow(uwb_pivot, 2) -
                    (std::pow(p_UinG.norm(), 2) - std::pow(p_UinG_pivot.norm(), 2)));
      s(j) = std::pow(uwb_data[i].second.distance_, 2) * solver_options_->sigma_meas_ +
             p_UinG.transpose() * solver_options_->sigma_pos_ * Eigen::Matrix3d::Identity() * p_UinG + weight_pivot;

              // Increment row index
      j += 1;
    }
    idx_tag += 1;
  }

  return true;
}

std::pair<uint, double> LsSolver::find_pivot_idx(const TimedBuffer<UwbData>& uwb_data,
                                                 const PositionBuffer& p_UinG_buffer)
{
  uint pivot_idx = 0;
  double weight_pivot =
    (std::pow(uwb_data[pivot_idx].second.distance_, 2) * solver_options_->sigma_meas_ +
     p_UinG_buffer.get_at_timestamp(uwb_data[pivot_idx].first).transpose() * Eigen::Matrix3d::Identity() *
       solver_options_->sigma_pos_ * p_UinG_buffer.get_at_timestamp(uwb_data[pivot_idx].first));
  for (uint i = 1; i < uwb_data.size(); ++i)
  {
    double weight_i = (std::pow(uwb_data[i].second.distance_, 2) * solver_options_->sigma_meas_ +
                       p_UinG_buffer.get_at_timestamp(uwb_data[i].first).transpose() * Eigen::Matrix3d::Identity() *
                         solver_options_->sigma_pos_ * p_UinG_buffer.get_at_timestamp(uwb_data[i].first));
    if (weight_i < weight_pivot)
    {
      pivot_idx = i;
      weight_pivot = weight_i;
    }
  }
  return std::make_pair(pivot_idx, weight_pivot);
}

}  // namespace uwb_init
