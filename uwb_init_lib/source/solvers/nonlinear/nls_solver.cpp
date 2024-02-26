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
}

bool NlsSolver::levenbergMarquardt(const TimedBuffer<UwbData>& uwb_data, const PositionBuffer& p_UinG_buffer,
                                   Eigen::VectorXd& theta, Eigen::MatrixXd& cov)
{
  UwbDataPerTag dict_uwb_data;
  dict_uwb_data.insert({(uint)0, uwb_data});
  PositionBufferDict_t dict_p_UinG_buffer;
  dict_p_UinG_buffer.insert({(uint)0, p_UinG_buffer});
  return levenbergMarquardt(dict_uwb_data, dict_p_UinG_buffer, theta, cov);

}
bool NlsSolver::levenbergMarquardt(const std::unordered_map<uint, TimedBuffer<UwbData> > &dict_uwb_data, const PositionBufferDict_t &dict_p_UinG_buffer, Eigen::VectorXd &theta, Eigen::MatrixXd &cov)
{
  int const num_tags = dict_uwb_data.size();
  size_t num_meas = 0;
  for(auto const&e : dict_uwb_data)
  {
    num_meas += e.second.size();
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

      if (theta.size() == 3 + 2*num_tags )
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
      else if (theta.size() == 3 + num_tags)
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
      else if (theta.size() == 3)
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
        if (theta.size() == 3 + 2*num_tags)
        {
          double beta_i = theta_tmp(3+num_tags+idx_tag);
          double gamma_i = theta_tmp(3+idx_tag);
          res_new(j) =
              d_TA_vec(j) - (beta_i * (theta_tmp.head(3).transpose() - p_GT_vec.row(j)).norm() + gamma_i);
        }
        else if (theta.size() == 3 + num_tags)
        {
          double gamma_i = theta_tmp(3+idx_tag);
          res_new(j) = d_TA_vec(j) - ((theta_tmp.head(3).transpose() - p_GT_vec.row(j)).norm() + gamma_i);
        }
        else if (theta.size() == 3)
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
      logger_->info("NlsSolver::levenbergMarquardt(): Norm of step is less than " +
                    std::to_string(solver_options_->step_cond_));
      logger_->info("NlsSolver::levenbergMarquardt(): Converged in " + std::to_string(iter) + " iterations");
      break;
    }

    // Residual stopping condition
    if (mse < solver_options_->res_cond_)
    {
      logger_->info("NlsSolver::levenbergMarquardt(): Mean squared error is less than " +
                    std::to_string(solver_options_->res_cond_));
      logger_->info("NlsSolver::levenbergMarquardt(): Converged in " + std::to_string(iter) + " iterations");
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

}  // namespace uwb_init
