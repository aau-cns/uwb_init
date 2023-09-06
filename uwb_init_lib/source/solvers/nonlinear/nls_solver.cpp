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
  // Lambda initialization
  double lambda = solver_options_->lambda_;

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
    logger_->err("NlsSolver::levenbergMarquardt(): Can not perform optimization (data vectors have different "
                 "dimensions)");
    return false;
  }

  // Iteration counter
  size_t iter = 0;

  // Levenberg-Marquardt algorithm
  while (iter <= solver_options_->max_iter_)
  {
    // Jacobian, Hessian and residual initialization
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(uwb_vec.size(), theta.size());
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(J.rows(), J.rows());
    Eigen::VectorXd res = Eigen::VectorXd::Zero(uwb_vec.size());
    Eigen::VectorXd res_new = Eigen::VectorXd::Zero(uwb_vec.size());

    // Auxiliary variables
    Eigen::VectorXd d_theta = Eigen::VectorXd::Zero(theta.size());
    double mse = 0.0;

    // Compute Jacobian and residual
    for (uint j = 0; j < J.rows(); ++j)
    {
      if (theta.size() == 5)
      {
        // Jacobian [df/dp_AinG, df/dgamma, df/dbeta]
        J.row(j) << theta(4) * (theta(0) - pose_vec(j, 0)) / (theta.head(3).transpose() - pose_vec.row(j)).norm(),
            theta(4) * (theta(1) - pose_vec(j, 1)) / (theta.head(3).transpose() - pose_vec.row(j)).norm(),
            theta(4) * (theta(2) - pose_vec(j, 2)) / (theta.head(3).transpose() - pose_vec.row(j)).norm(), 1,
            (theta.head(3).transpose() - pose_vec.row(j)).norm();
        // Residual res = y - f(theta) =  uwb_meas - (beta * ||p_AinG - p_UinG|| + gamma)
        res(j) = uwb_vec(j) - (theta(4) * (theta.head(3).transpose() - pose_vec.row(j)).norm() + theta(3));
      }
      else if (theta.size() == 4)
      {
        // Jacobian [df/dp_AinG, df/dgamma]
        J.row(j) << (theta(0) - pose_vec(j, 0)) / (theta.head(3).transpose() - pose_vec.row(j)).norm(),
            (theta(1) - pose_vec(j, 1)) / (theta.head(3).transpose() - pose_vec.row(j)).norm(),
            (theta(2) - pose_vec(j, 2)) / (theta.head(3).transpose() - pose_vec.row(j)).norm(), 1;
        // Residual res = y - f(theta) =  uwb_meas - (||p_AinG - p_UinG|| + gamma)
        res(j) = uwb_vec(j) - ((theta.head(3).transpose() - pose_vec.row(j)).norm() + theta(3));
      }
      else if (theta.size() == 3)
      {
        // Jacobian [df/dp_AinG]
        J.row(j) << (theta(0) - pose_vec(j, 0)) / (theta.head(3).transpose() - pose_vec.row(j)).norm(),
            (theta(1) - pose_vec(j, 1)) / (theta.head(3).transpose() - pose_vec.row(j)).norm(),
            (theta(2) - pose_vec(j, 2)) / (theta.head(3).transpose() - pose_vec.row(j)).norm();
        // Residual res = y - f(theta) =  uwb_meas - ||p_AinG - p_UinG||
        res(j) = uwb_vec(j) - (theta.head(3).transpose() - pose_vec.row(j)).norm();
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
        if (theta.size() == 5)
          res_new(j) =
              uwb_vec(j) - (theta_tmp(4) * (theta_tmp.head(3).transpose() - pose_vec.row(j)).norm() + theta_tmp(3));
        else if (theta.size() == 4)
          res_new(j) = uwb_vec(j) - ((theta_tmp.head(3).transpose() - pose_vec.row(j)).norm() + theta_tmp(3));
        else if (theta.size() == 3)
          res_new(j) = uwb_vec(j) - ((theta_tmp.head(3).transpose() - pose_vec.row(j)).norm());
      }

      // If residual decreases, update lambda and theta, and break
      if (res_new.norm() < res.norm())
      {
        // Decrease lambda
        lambda /= solver_options_->lambda_factor_;

        // Update theta
        theta += d_theta;

        // Calculate MSE = ||res_new||^2 / (n - p)
        mse = res_new.squaredNorm() / (uwb_vec.size() - theta.size());

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
  if (!isSPD(cov))
  {
    logger_->err("NlsSolver::levenbergMarquardt(): Covariance matrix is not SPD");
    return false;
  }

  return true;
}

}  // namespace uwb_init
