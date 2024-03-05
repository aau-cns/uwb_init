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

#include "solvers/nonlinear/nls_solver.hpp"
#include "solvers/linear/ls_solver.hpp"


#include <assert.h>

namespace uwb_init
{
NlsSolver::NlsSolver(const std::shared_ptr<Logger> logger, std::unique_ptr<NlsSolverOptions>&& nls_solver_options)
  : logger_(std::move(logger)), solver_options_(std::move(nls_solver_options))
{
  // Debug assertation
  assert(logger_ != nullptr);
  assert(solver_options_ != nullptr);

  // Logging
  logger_->info("NlsSolver: Initialized");
  logger_->debug("NlsSolver options: lambda = " + std::to_string(solver_options_->lambda_));
  logger_->debug("NlsSolver options: lambda_factor = " + std::to_string(solver_options_->lambda_factor_));
  logger_->debug("NlsSolver options: step_cond = " + std::to_string(solver_options_->step_cond_));
  logger_->debug("NlsSolver options: res_cond = " + std::to_string(solver_options_->res_cond_));
  logger_->debug("NlsSolver options: max_iter = " + std::to_string(solver_options_->max_iter_));
  logger_->debug("NlsSolver options: use_RANSAC = " + std::to_string(solver_options_->use_RANSAC_));
  logger_->debug("NlsSolver options: sigma_pos = " + std::to_string(solver_options_->sigma_pos_));
  logger_->debug("NlsSolver options: sigma_meas = " + std::to_string(solver_options_->sigma_meas_));
  logger_->debug("NlsSolver options: bias_type = " + BiasType_to_string(solver_options_->bias_type_));
  logger_->debug("NlsSolver options: ransac_opts = " + solver_options_->ransac_opts_.str());

  std::random_device rd; // obtain a random number from hardware
  ptr_gen_.reset(new std::mt19937(rd()));
}

void NlsSolver::configure(const BiasType bias_type)
{
  solver_options_->bias_type_ = bias_type;
}

BiasType NlsSolver::bias_type() {return solver_options_->bias_type_; }



bool NlsSolver::levenbergMarquardt(const TimedBuffer<UwbData>& uwb_data, const PositionBuffer& p_UinG_buffer,
                                   Eigen::VectorXd& theta, Eigen::MatrixXd& cov)
{
  UwbDataPerTag dict_uwb_data;
  dict_uwb_data.insert({(uint)0, uwb_data});
  PositionBufferDict_t dict_p_UinG_buffer;
  dict_p_UinG_buffer.insert({(uint)0, p_UinG_buffer});
  return levenbergMarquardt(dict_uwb_data, dict_p_UinG_buffer, theta, cov);

}
bool NlsSolver::levenbergMarquardt(const UwbDataPerTag &dict_uwb_data, const PositionBufferDict_t &dict_p_UinG_buffer, Eigen::VectorXd &theta, Eigen::MatrixXd &cov)
{
  int const num_tags = dict_uwb_data.size();
  if(num_tags < 1) {
    logger_->err("NlsSolver::levenbergMarquardt: empty uwb range buffer obtained");
    return false;
  }
  if(dict_uwb_data.size() > dict_p_UinG_buffer.size())
  {
    logger_->err("NlsSolver::levenbergMarquardt: uwb range buffer and position buffer do not match");
    return false;
  }

  size_t num_meas = 0;
  for(auto const&e : dict_uwb_data)
  {
    num_meas += e.second.size();
  }


  bool init_theta = false;
  switch (solver_options_->bias_type_) {
    case BiasType::NO_BIAS:
    {
      if(theta.size() != 3) {
        init_theta = true;
      }
      break;
    }
    case BiasType::CONST_BIAS: {
      if(theta.size() != 3 + num_tags ) {
        init_theta = true;
      }
    }
    case BiasType::ALL_BIAS:
    {
      if(theta.size() != 3 + 2*num_tags ) {
        init_theta = true;
      }
      break;
    }
  }
  if (init_theta) {
    theta = init_from_lsSolution(theta, num_tags);
  }

  // Lambda initialization
  double lambda = solver_options_->lambda_;

  // Data vectors initialization
  Eigen::VectorXd d_TA_vec = Eigen::VectorXd::Zero(num_meas);     // distance between Tag and Anchor
  Eigen::MatrixXd p_GT_vec = Eigen::MatrixXd::Zero(num_meas, 3);  // position of Tag in global frame
  Eigen::VectorXi tag_idx_vec = Eigen::VectorXi::Zero(num_meas);  // index of Tag
  size_t idx_meas = 0;
  size_t idx_tag = 0;
  for(auto const&e : dict_uwb_data)
  {
    uint const Tag_ID = e.first;
    auto const& uwb_data = e.second;
    auto const& p_UinG_buffer = dict_p_UinG_buffer.at(Tag_ID);

            // Create consistent data vectors
    for (uint i = 0; i < uwb_data.size(); ++i)
    {
      d_TA_vec(idx_meas) = uwb_data[i].second.distance_;
      p_GT_vec.row(idx_meas) = p_UinG_buffer.get_at_timestamp(uwb_data[i].first);
      tag_idx_vec(idx_meas) = idx_tag;
      idx_meas += 1;
    }

    // Check for consistency
    if (d_TA_vec.size() != p_GT_vec.rows())
    {
      // Refine unsuccessful
      logger_->err("NlsSolver::levenbergMarquardt(): Can not perform optimization (data vectors have different "
        "dimensions)");
      return false;
    }
    idx_tag += 1;
  }
  // Iteration counter
  size_t iter = 0;

  // Levenberg-Marquardt algorithm
  while (iter <= solver_options_->max_iter_)
  {
    // Jacobian, Hessian and residual initialization
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(d_TA_vec.size(), theta.size());
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(J.rows(), J.rows());
    Eigen::VectorXd res = Eigen::VectorXd::Zero(d_TA_vec.size());
    Eigen::VectorXd res_new = Eigen::VectorXd::Zero(d_TA_vec.size());

    // Auxiliary variables
    Eigen::VectorXd d_theta = Eigen::VectorXd::Zero(theta.size());
    double mse = 0.0;

    // Compute Jacobian and residual
    for (uint j = 0; j < J.rows(); ++j)
    {
      Eigen::VectorXd gamma_vec, beta_vec;
      size_t const idx_tag = tag_idx_vec(j);

      if (solver_options_->bias_type_ == BiasType::ALL_BIAS) //(theta.size() == 3 + 2*num_tags )
      {
        gamma_vec.setZero(num_tags);
        beta_vec.setZero(num_tags);

        // fill out the entry of the corresponding tag
        gamma_vec(idx_tag) = 1;
        beta_vec(idx_tag) = (theta.head(3).transpose() - p_GT_vec.row(j)).norm();

        double beta_i = theta(3+num_tags+idx_tag);
        double gamma_i = theta(3+idx_tag);

        // Jacobian [df/dp_AinG, df/dgamma, df/dbeta]
        J.row(j) << beta_i * (theta(0) - p_GT_vec(j, 0)) / (theta.head(3).transpose() - p_GT_vec.row(j)).norm(),
            beta_i * (theta(1) - p_GT_vec(j, 1)) / (theta.head(3).transpose() - p_GT_vec.row(j)).norm(),
            beta_i * (theta(2) - p_GT_vec(j, 2)) / (theta.head(3).transpose() - p_GT_vec.row(j)).norm(), gamma_vec.transpose(),
            beta_vec.transpose();
        // Residual res = y - f(theta) =  uwb_meas - (beta * ||p_AinG - p_UinG|| + gamma)
        res(j) = d_TA_vec(j) - (beta_i * (theta.head(3).transpose() - p_GT_vec.row(j)).norm() + gamma_i);
      }
      else if (solver_options_->bias_type_ == BiasType::CONST_BIAS) //(theta.size() == 3 + num_tags)
      {
        // fill out the entry of the corresponding tag
        gamma_vec.setZero(num_tags);
        gamma_vec(idx_tag) = 1;

        double gamma_i = theta(3+idx_tag);

        // Jacobian [df/dp_AinG, df/dgamma]
        J.row(j) << (theta(0) - p_GT_vec(j, 0)) / (theta.head(3).transpose() - p_GT_vec.row(j)).norm(),
            (theta(1) - p_GT_vec(j, 1)) / (theta.head(3).transpose() - p_GT_vec.row(j)).norm(),
            (theta(2) - p_GT_vec(j, 2)) / (theta.head(3).transpose() - p_GT_vec.row(j)).norm(), gamma_vec.transpose();
        // Residual res = y - f(theta) =  uwb_meas - (||p_AinG - p_UinG|| + gamma)
        res(j) = d_TA_vec(j) - ((theta.head(3).transpose() - p_GT_vec.row(j)).norm() + gamma_i);
      }
      else if (solver_options_->bias_type_ == BiasType::NO_BIAS) //(theta.size() == 3)
      {
        // Jacobian [df/dp_AinG]
        J.row(j) << (theta(0) - p_GT_vec(j, 0)) / (theta.head(3).transpose() - p_GT_vec.row(j)).norm(),
            (theta(1) - p_GT_vec(j, 1)) / (theta.head(3).transpose() - p_GT_vec.row(j)).norm(),
            (theta(2) - p_GT_vec(j, 2)) / (theta.head(3).transpose() - p_GT_vec.row(j)).norm();
        // Residual res = y - f(theta) =  uwb_meas - ||p_AinG - p_UinG||
        res(j) = d_TA_vec(j) - (theta.head(3).transpose() - p_GT_vec.row(j)).norm();
      }
    }

    // Calculate Moore-Penrose Pseudo-Inverse of matrix J
    Eigen::BDCSVD<Eigen::MatrixXd> svd_J(J, Eigen::ComputeThinU | Eigen::ComputeThinV);

    // While residual is not decreasing
    while (iter <= solver_options_->max_iter_)
    {
      // Calculate Hessian (H = J'*J = V*S*S*V' (see properties of SVD)
      H = svd_J.matrixV() * svd_J.singularValues().asDiagonal() * svd_J.singularValues().asDiagonal() *
          svd_J.matrixV().transpose();

      // Add lambda * diag(H) to Hessian
      H.diagonal() += lambda * H.diagonal();

      // Compute the singular value decomposition of H and calculate pinv_H
      Eigen::JacobiSVD<Eigen::MatrixXd> svd_H(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
      double tolerance = std::numeric_limits<double>::epsilon() * std::max(H.cols(), H.rows()) *
                         svd_H.singularValues().array().abs()(0);
      Eigen::MatrixXd pinv_H = svd_H.matrixV() *
                               (svd_H.singularValues().array().abs() > tolerance)
                                   .select(svd_H.singularValues().array().inverse(), 0)
                                   .matrix()
                                   .asDiagonal() *
                               svd_H.matrixU().adjoint();

      // Calculate step d_theta
      d_theta = pinv_H * J.transpose() * res;

      // Temporary theta
      Eigen::VectorXd theta_tmp = theta + d_theta;

      // Compute new residual
      for (uint j = 0; j < res_new.size(); ++j)
      {
        size_t const idx_tag = tag_idx_vec(j);
        if (solver_options_->bias_type_ == BiasType::ALL_BIAS) //(theta.size() == 3 + 2*num_tags)
        {
          double beta_i = theta_tmp(3+num_tags+idx_tag);
          double gamma_i = theta_tmp(3+idx_tag);
          res_new(j) =
              d_TA_vec(j) - (beta_i * (theta_tmp.head(3).transpose() - p_GT_vec.row(j)).norm() + gamma_i);
        }
        else if (solver_options_->bias_type_ == BiasType::CONST_BIAS) //(theta.size() == 3 + num_tags)
        {
          double gamma_i = theta_tmp(3+idx_tag);
          res_new(j) = d_TA_vec(j) - ((theta_tmp.head(3).transpose() - p_GT_vec.row(j)).norm() + gamma_i);
        }
        else if (solver_options_->bias_type_ == BiasType::NO_BIAS) //(theta.size() == 3)
        {
          res_new(j) = d_TA_vec(j) - ((theta_tmp.head(3).transpose() - p_GT_vec.row(j)).norm());
        }
      }

      // If residual decreases, update lambda and theta, and break
      if (res_new.norm() < res.norm())
      {
        // Decrease lambda
        lambda /= solver_options_->lambda_factor_;

        // Update theta
        theta += d_theta;

        // Calculate MSE = ||res_new||^2 / (n - p)
        mse = res_new.squaredNorm() / (d_TA_vec.size() - theta.size());

        // Compute estimation Covariance matrix
        cov = pinv_H * mse;

        // Break inner loop
        break;
      }

      // If residual does not decrease, increase lambda and continue
      lambda *= solver_options_->lambda_factor_;

      // Increase iteration counter
      ++iter;
    }

    // Norm of step stopping condition
    if (d_theta.norm() < solver_options_->step_cond_)
    {
      logger_->debug("NlsSolver::levenbergMarquardt(): Norm of step is less than " +
                    std::to_string(solver_options_->step_cond_));
      logger_->debug("NlsSolver::levenbergMarquardt(): Converged in " + std::to_string(iter) + " iterations");
      break;
    }

    // Residual stopping condition
    if (mse < solver_options_->res_cond_)
    {
      logger_->debug("NlsSolver::levenbergMarquardt(): Mean squared error is less than " +
                    std::to_string(solver_options_->res_cond_));
      logger_->debug("NlsSolver::levenbergMarquardt(): Converged in " + std::to_string(iter) + " iterations");
      break;
    }

    // Max iterations stopping condition
    if (iter >= (solver_options_->max_iter_))
    {
      logger_->warn("NlsSolver::levenbergMarquardt(): Maximum number of iterations reached (" +
                    std::to_string(solver_options_->max_iter_) + ")");
      break;
    }

    // Increment iteration counter
    ++iter;
  }

  // If covariance matrix is not semi-positive-definite return
  if (solver_options_->check_cov_ && !isSPD(cov))
  {
    logger_->err("NlsSolver::levenbergMarquardt(): Covariance matrix is not SPD");
    return false;
  }

  return true;
}

bool NlsSolver::levenbergMarquardt(const UwbDataPerTag &dict_uwb_data, const PositionBufferDict_t &dict_p_UinG_buffer, Eigen::VectorXd &theta, Eigen::MatrixXd &cov, UwbDataPerTag &dict_uwb_inliers)
{
  // if RANSAC is disabled, use default method
  if(!solver_options_->use_RANSAC_)
  {
    dict_uwb_inliers = dict_uwb_data;
    return levenbergMarquardt(dict_uwb_data, dict_p_UinG_buffer, theta, cov);
  }

  // make sure to initialize the NlsSolver from the LsSolvers initial guess properly:

  std::vector<double> costs;
  std::vector<Eigen::VectorXd> lsSolutions;
  std::vector<Eigen::MatrixXd> Sigmas;

  costs.reserve(solver_options_->ransac_opts_.n);
  lsSolutions.reserve(solver_options_->ransac_opts_.n);

  if(has_Tag_enough_samples(dict_uwb_data)) {
    logger_->info("NlsSolver::levenbergMarquardt: enough data collected for RANSAC");
  }

  size_t const num_tags = dict_uwb_data.size();
  theta = init_from_lsSolution(theta, num_tags);
  // Repeat the random sampling n-time, hoping that we pick once data without outliers!
  for(size_t iter=0; iter < solver_options_->ransac_opts_.n; iter++)
  {
    // 1) take random from measurement and hope that it does not contain outlier
    auto dict_sampled_data = LsSolver::rand_samples(dict_uwb_data, solver_options_->ransac_opts_.s, ptr_gen_);

    // 2) solve the problem:
    Eigen::VectorXd lsSolution_i = theta;
    Eigen::MatrixXd Sigma_i;
    if(levenbergMarquardt(dict_sampled_data, dict_p_UinG_buffer, lsSolution_i, Sigma_i))
    {
      double cost_i = LsSolver::compute_cost(dict_uwb_data, dict_p_UinG_buffer, lsSolution_i, solver_options_->bias_type_);
      costs.push_back(cost_i);
      lsSolutions.push_back(lsSolution_i);
      Sigmas.push_back(Sigma_i);
      logger_->debug("NlsSolver::levenbergMarquardt: RANSAC iter=" + std::to_string(iter) + ", cost=" + std::to_string(cost_i));
    }
  }

  if(costs.empty()) {
    logger_->warn("NlsSolver::levenbergMarquardt: RANSAC: no costs computed!");
    theta.setZero(3);
    cov.setZero(3,3);
    return false;
  }

  // 3) find the best model:
  auto iter_min_cost = std::min_element(costs.begin(), costs.end());
  size_t idx_best = iter_min_cost - costs.begin();
  if(logger_->getlevel() == LoggerLevel::FULL)
  {
    std::stringstream ss;
    ss << lsSolutions[idx_best].transpose();
    logger_->debug("NlsSolver::levenbergMarquardt: RANSAC: best model at iter=" + std::to_string(idx_best) + ", cost=" + std::to_string(*iter_min_cost) + " is x_est=" + ss.str());
  }

  // 4) remove outliers that fall outside the 99% of the distribution 2*(sigma_uwb+sigma_pos) using all measurement and best match
  double threshold = (solver_options_->sigma_meas_ + solver_options_->sigma_pos_)*solver_options_->ransac_opts_.thres_num_std;

  std::pair<UwbDataPerTag, size_t> uwb_data_clean = LsSolver::remove_ouliers(dict_uwb_data, dict_p_UinG_buffer, lsSolutions[idx_best], solver_options_->bias_type_, threshold, solver_options_->ransac_opts_.n);
  logger_->debug("NlsSolver::levenbergMarquardt: [" + std::to_string(uwb_data_clean.second) + "] outliers removed above threshold=" + std::to_string(threshold));
  dict_uwb_inliers = uwb_data_clean.first;

  // 4.1) ATTENTION: all measurements might have been removed:
  if(dict_uwb_inliers.empty()) {
    dict_uwb_inliers = dict_uwb_data;
  }

  //4.2) ATTENTION: the cleaned that might have removed tags, thus the best estimate and initial guess needs to be resized accordingly!
  if(dict_uwb_inliers.size() < dict_uwb_data.size()) {

    std::vector<size_t> ID_Tags_old;
    for(auto const&e : dict_uwb_data) { ID_Tags_old.push_back(e.first); }

    Eigen::VectorXd theta_best = lsSolutions[idx_best];

    size_t const num_tags_new = dict_uwb_inliers.size();
    // resize accordingly:
    theta = init_from_lsSolution(theta_best({0,1,3}), num_tags_new);

    if(solver_options_->bias_type_ != BiasType::NO_BIAS) {
      int idx_tag_new = 0;
      int idx_tag_old = 0;
      for(auto const& e : dict_uwb_data) {
        if(dict_uwb_inliers.find(e.first) != dict_uwb_inliers.end()) {
          theta(3+idx_tag_new) = theta_best(3+idx_tag_old);
          if (solver_options_->bias_type_ == BiasType::ALL_BIAS) {
            theta(3+num_tags_new+idx_tag_new) = theta_best(3+num_tags+idx_tag_old);
          }
          idx_tag_new++;
        }
        idx_tag_old++;
      }
    }
  }
  else
  {
    theta = lsSolutions[idx_best];
  }





  // 5) fit a new model on the cleaned data:
  if(levenbergMarquardt(dict_uwb_inliers, dict_p_UinG_buffer, theta, cov)) {
    if(logger_->getlevel() == LoggerLevel::FULL)
    {
      double cost_final = LsSolver::compute_cost(dict_uwb_inliers, dict_p_UinG_buffer, theta, solver_options_->bias_type_);
      std::stringstream ss;
      ss << theta.transpose();
      logger_->debug("NlsSolver::levenbergMarquardt: final cost=" + std::to_string(cost_final) + " for x_best=" + ss.str());
    }
    return true;
  }
  else if(lsSolutions.size()) {
    theta = lsSolutions[idx_best];
    cov = Sigmas[idx_best];
    return true;
  } else {
    return false;
  }


}

bool NlsSolver::has_Tag_enough_samples(const UwbDataPerTag &dict_uwb_data) {
  std::unordered_map<uint, bool> dict = LsSolver::has_Tag_enough_samples(dict_uwb_data, solver_options_->ransac_opts_.num_samples_needed());

  bool res = true;
  for(auto const& e : dict) {
    if(!e.second) {
      res = false;
      logger_->debug("NlsSolver::has_Tag_enough_samples(): more samples needed for ransac! TagID=[" + std::to_string(e.first) + "] ! min=" + std::to_string(solver_options_->ransac_opts_.num_samples_needed()) + ", currenty=" + std::to_string(dict_uwb_data.at(e.first).size()));
    }
  }
  return res;
}

Eigen::VectorXd NlsSolver::init_from_lsSolution(const Eigen::VectorXd &lsSolution, size_t const num_Tags) {

  Eigen::VectorXd p_AinG;
  if(lsSolution.rows() >= 3) {
    p_AinG = lsSolution.head(3);
  } else {
    p_AinG.setZero(3);
  }

  Eigen::VectorXd range_biases, const_biases;
  range_biases.setOnes(num_Tags);
  const_biases.setZero(num_Tags);

  if(lsSolution.rows() >= 3+int(num_Tags)) {
    // copy all solutions ot the vectors
    for(size_t idx = 0; idx < num_Tags; idx++) {
      const_biases(idx) = lsSolution(3+idx);
    }
  }

  Eigen::VectorXd nls_init_guess;
  // Initial guess for NLS based on bias type
  if (solver_options_->bias_type_ == BiasType::ALL_BIAS)
  {
    nls_init_guess = Eigen::VectorXd::Zero(3+2*num_Tags);
    nls_init_guess << p_AinG, const_biases, range_biases;
  }
  else if(solver_options_->bias_type_ == BiasType::CONST_BIAS) {
    nls_init_guess = Eigen::VectorXd::Zero(3+num_Tags);
    nls_init_guess << p_AinG, const_biases;
  }
  else {
    nls_init_guess = Eigen::VectorXd::Zero(3);
    nls_init_guess << p_AinG;
  }
  return nls_init_guess;

}

}  // namespace uwb_init
