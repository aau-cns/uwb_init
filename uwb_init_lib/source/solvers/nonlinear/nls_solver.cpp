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

namespace uwb_init
{
NlsSolver::NlsSolver(const std::shared_ptr<Logger> logger, const NlsSolverOptions& nls_solver_options)
  : logger_(std::move(logger))
{
  // Set solver options
  solver_options_ = nls_solver_options;

  // Logging
  logger_->info("NlsSolver: Initialized");
}

bool NlsSolver::solve_nls(const TimedBuffer<UwbData>& uwb_data, const PositionBuffer& p_UinG_buffer,
                          Eigen::VectorXd& theta, Eigen::MatrixXd& cov)
{
  // Step norm vector
  Eigen::VectorXd step_vec_ = solver_options_.step_vec_;

  // Data vectors initialization
  Eigen::VectorXd uwb_vec = Eigen::VectorXd::Zero(uwb_data.size());
  Eigen::MatrixXd pose_vec = Eigen::MatrixXd::Zero(uwb_vec.size(), 3);

  // Create consistent data vectors
  for (uint i = 0; i < uwb_vec.size(); ++i)
  {
    uwb_vec(i) = uwb_data[i].second.distance_;
    pose_vec.row(i) = p_UinG_buffer.get_at_timestamp(uwb_data[i].first);
  }

  // Check for consistency
  if (uwb_vec.size() != pose_vec.rows())
  {
    // Refine unsuccessful
    logger_->err("NlsSolver::solve_nls(): Can not perform optimization (data vectors have different dimensions)");
    return false;
  }

  // Non-linear Least Squares
  for (uint i = 0; i < solver_options_.max_iter_; ++i)
  {
    // Jacobian matrix, residual and mean squared error vectors initialization
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(uwb_vec.size(), 5);
    Eigen::VectorXd res = Eigen::VectorXd::Zero(uwb_vec.size());
    Eigen::VectorXd mse_vec = Eigen::VectorXd::Zero(step_vec_.size());

    // Compute Jacobian and residual
    for (uint j = 0; j < uwb_vec.size(); ++j)
    {
      // Jacobian [df/dp_AinG, df/dbeta, df/dgamma]
      J.row(j) << theta(3) * (theta(0) - pose_vec(j, 0)) / (theta.head(3).transpose() - pose_vec.row(j)).norm(),
          theta(3) * (theta(1) - pose_vec(j, 1)) / (theta.head(3).transpose() - pose_vec.row(j)).norm(),
          theta(3) * (theta(2) - pose_vec(j, 2)) / (theta.head(3).transpose() - pose_vec.row(j)).norm(),
          (theta.head(3).transpose() - pose_vec.row(j)).norm(), 1;
      // Residual res = y - f(theta) =  uwb_meas - (beta * ||p_AinG - p_UinG|| + gamma)
      res(j) = uwb_vec(j) - (theta(3) * (theta.head(3).transpose() - pose_vec.row(j)).norm() + theta(4));
    }

    // Calculate Moore-Penrose Pseudo-Inverse of matrix J
    Eigen::BDCSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance =
        std::numeric_limits<double>::epsilon() * std::max(J.cols(), J.rows()) * svd.singularValues().array().abs()(0);
    Eigen::MatrixXd pinv_J = svd.matrixV() *
                             (svd.singularValues().array().abs() > tolerance)
                                 .select(svd.singularValues().array().inverse(), 0)
                                 .matrix()
                                 .asDiagonal() *
                             svd.matrixU().adjoint();

    // Compute norm of step
    Eigen::VectorXd d_theta = pinv_J * res;

    // Calculate mean squared error for each step
    for (uint j = 0; j < step_vec_.size(); ++j)
    {
      Eigen::VectorXd theta_new = theta + step_vec_(j) * d_theta;
      for (uint k = 0; k < uwb_vec.size(); ++k)
      {
        // MSE(k) = r(k)^2 = (uwb(k) - f(theta))^2
        mse_vec(j) += std::pow(
            uwb_vec(k) - (theta_new(3) * (theta_new.head(3).transpose() - pose_vec.row(k)).norm() + theta_new(4)), 2);
      }

      // MSE = r'*r/(m-p) where m is the number of data points and p is the number of parameters
      mse_vec(j) /= uwb_vec.size() - 5;
    }

    // Choose index that minimizes MSE
    Eigen::Index step_idx;
    mse_vec.minCoeff(&step_idx);

    // Perform parameters update theta(k+1) = theta(k) + step_norm * d_theta
    theta += step_vec_(step_idx) * d_theta;

    // Compute estimation Covariance (Var(X) = MSE*(J'*J)^-1) = MSE*V*S^-1*S^-1*V' (see properties of SVD)
    cov = mse_vec(step_idx) * svd.matrixV() * svd.singularValues().asDiagonal().inverse() *
          svd.singularValues().asDiagonal().inverse() * svd.matrixV().transpose();

    // If step norm is minimum reduce the step for next iteration
    if (step_idx == 0)
    {
      logger_->debug("NlsSolver::solve_nls(): Reducing step norm");
      step_vec_ /= 2;
    }

    // Norm of step stopping condition
    if (step_vec_(step_idx) * d_theta.norm() / theta.norm() < solver_options_.step_cond_)
    {
      logger_->info("NlsSolver::solve_nls(): Relative norm of step is less than " +
                    std::to_string(solver_options_.step_cond_));
      break;
    }

    // Residual stopping condition
    if (mse_vec(step_idx) < solver_options_.res_cond_)
    {
      logger_->info("NlsSolver::solve_nls(): Mean squared error is less than " +
                    std::to_string(solver_options_.res_cond_));
      break;
    }

    // Check if maximum number of iteration reached
    if (i == (solver_options_.max_iter_ - 1))
    {
      logger_->warn("NlsSolver::solve_nls(): Maximum number of iterations reached (" +
                    std::to_string(solver_options_.max_iter_) + ")");
    }
  }

  // If covariance matrix is not semi-positive-definite return
  if (!isSPD(cov))
  {
    logger_->err("NlsSolver::solve_nls(): Covariance matrix is not SPD");
    return false;
  }

  return true;
}

bool NlsSolver::solve_oea(const TimedBuffer<UwbData>& uwb_data, const PositionBuffer& p_UinG_buffer,
                          Eigen::VectorXd& theta, Eigen::MatrixXd& cov)
{
  // Data vectors initialization
  Eigen::VectorXd uwb_vec = Eigen::VectorXd::Zero(uwb_data.size());
  Eigen::MatrixXd pose_vec = Eigen::MatrixXd::Zero(uwb_vec.size(), 3);

  // Create consistent data vectors
  for (uint i = 0; i < uwb_vec.size(); ++i)
  {
    uwb_vec(i) = uwb_data[i].second.distance_;
    pose_vec.row(i) = p_UinG_buffer.get_at_timestamp(uwb_data[i].first);
  }

  // Check for consistency
  if (uwb_vec.size() != pose_vec.rows())
  {
    // Refine unsuccessful
    logger_->err("NlsSolver::solve_oea(): Can not perform optimization (data vectors have different dimensions)");
    return false;
  }

  // Output Error Algorithm
  for (uint i = 0; i < solver_options_.max_iter_; ++i)
  {
    // Simulate the system with current theta
    Eigen::VectorXd y_hat(uwb_vec.size());
    for (uint j = 0; j < y_hat.size(); ++j)
    {
      y_hat(j) = theta(3) * (theta.head(3).transpose() - pose_vec.row(j)).norm() + theta(4);
    }

    // Compute matrix e (error)
    Eigen::VectorXd e(uwb_vec.size());
    e = uwb_vec - y_hat;

    // Compute estimated covariance R_hat = 1/(N - p) * sum_i( e(i)*e(i)' )
    double R_hat = 1 / double(e.size() - 5) * e.transpose() * e;

    // Compute output sensitivities J
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(uwb_vec.size(), 5);
    for (uint j = 0; j < uwb_vec.size(); ++j)
    {
      // S(j, :) = [df/dp_AinG, df/dbeta, df/dgamma]
      J.row(j) << theta(3) * (theta(0) - pose_vec(j, 0)) / (theta.head(3).transpose() - pose_vec.row(j)).norm(),
          theta(3) * (theta(1) - pose_vec(j, 1)) / (theta.head(3).transpose() - pose_vec.row(j)).norm(),
          theta(3) * (theta(2) - pose_vec(j, 2)) / (theta.head(3).transpose() - pose_vec.row(j)).norm(),
          (theta.head(3).transpose() - pose_vec.row(j)).norm(), 1;
    }

    // Compute approximated Hessian M and the Jacobian g
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(5, 5);
    Eigen::VectorXd g = Eigen::VectorXd::Zero(5);
    M = 1 / R_hat * J.transpose() * J;
    g = 1 / R_hat * J.transpose() * e;

    // Update estimate theta
    Eigen::VectorXd d_theta(5);
    d_theta = M.inverse() * g;
    theta += d_theta;

    // Update covariance
    Eigen::BDCSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance =
        std::numeric_limits<double>::epsilon() * std::max(J.cols(), J.rows()) * svd.singularValues().array().abs()(0);
    Eigen::MatrixXd pinv_J = svd.matrixV() *
                             (svd.singularValues().array().abs() > tolerance)
                                 .select(svd.singularValues().array().inverse(), 0)
                                 .matrix()
                                 .asDiagonal() *
                             svd.matrixU().adjoint();

    // Compute estimation Covariance (Var(X) = MSE*(J'*J)^-1) = MSE*V*S^-1*S^-1*V' (see properties of SVD)
    cov = R_hat * svd.matrixV() * svd.singularValues().asDiagonal().inverse() *
          svd.singularValues().asDiagonal().inverse() * svd.matrixV().transpose();

    // Check if norm of step is small
    if (d_theta.norm() / theta.norm() < solver_options_.step_cond_)
    {
      logger_->info("NlsSolver::solve_oea(): Realtive norm of step is less than " +
                    std::to_string(solver_options_.step_cond_));
      break;
    }

    // Check if norm of gradient is small
    if (g.norm() < solver_options_.res_cond_)
    {
      logger_->info("NlsSolver::solve_oea(): Absolute norm of gradient is less than " +
                    std::to_string(solver_options_.res_cond_));
      break;
    }

    // Check if maximum number of iteration reached
    if (i == (solver_options_.max_iter_ - 1))
    {
      logger_->warn("NlsSolver::solve_oea(): Maximum number of iterations reached (" +
                    std::to_string(solver_options_.max_iter_) + ")");
    }
  }

  // If covariance matrix is not semi-positive-definite return
  if (!isSPD(cov))
  {
    logger_->err("NlsSolver::solve_oea(): Covariance matrix is not SPD");
    return false;
  }

  return true;
}

}  // namespace uwb_init
