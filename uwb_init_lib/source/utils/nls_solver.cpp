// // Copyright (C) 2021 Giulio Delama, Alessandro Fornasier
// // Control of Networked Systems, Universitaet Klagenfurt, Austria
// //
// // All rights reserved.
// //
// // This software is licensed under the terms of the BSD-2-Clause-License with
// // no commercial use allowed, the full terms of which are made available
// // in the LICENSE file. No license in patents is granted.
// //
// // THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// // IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// // FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// // THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// // LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// // FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// // DEALINGS IN THE SOFTWARE.
// //
// // You can contact the authors at <alessandro.fornasier@aau.at> 
// // and <giulio.delama@aau.at>

#include "utils/nls_solver.hpp"

namespace uwb_init
{

NlsSolver::NlsSolver(const std::shared_ptr<Logger> logger) : logger_(std::move(logger))
{
    // Logging
    logger_->info("NlsSolver: Initialized");
}


bool NlsSolver::solve_nls(const TimedBuffer<UwbData>& uwb_data, const PositionBuffer& p_UinG_buffer,
                          Eigen::VectorXd& theta, Eigen::MatrixXd& cov)
{
  // Step norm vector
  Eigen::VectorXd step_vec = nls_params_.step_vec;

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
    logger_->err("NlsSolver: Can not perform optimization (data vectors have different dimensions)");
    return false;
  }

  // Non-linear Least Squares
  for (uint i = 0; i < nls_params_.max_iter; ++i)
  {
    // Jacobian and residual initialization
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(uwb_vec.size(), 5);
    Eigen::VectorXd res = Eigen::VectorXd::Zero(uwb_vec.size());
    Eigen::VectorXd res_vec = Eigen::VectorXd::Zero(step_vec.size());

    // Compute Jacobian and residual
    for (uint j = 0; j < uwb_vec.size(); ++j)
    {
      // Jacobian [df/dp_AinG, df/dbeta, df/dgamma]
      J.row(j) << theta(3) * (theta(0) - pose_vec(j, 0)) / (theta.head(3).transpose() - pose_vec.row(j)).norm(),
              theta(3) * (theta(1) - pose_vec(j, 1)) / (theta.head(3).transpose() - pose_vec.row(j)).norm(),
              theta(3) * (theta(2) - pose_vec(j, 2)) / (theta.head(3).transpose() - pose_vec.row(j)).norm(),
              (theta.head(3).transpose() - pose_vec.row(j)).norm(),
              1;
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

    // Compute estimation Covariance (Var(X) = (J'*J)^-1) = V*S^-1*S^-1*V' (see properties of SVD)
    cov = svd.matrixV() * svd.singularValues().asDiagonal().inverse() *
              svd.singularValues().asDiagonal().inverse() * svd.matrixV().transpose();

    // Compute norm of step
    Eigen::VectorXd d_theta = pinv_J * res;

    // Calculate residual for each step
    for (uint j = 0; j < step_vec.size(); ++j)
    {
      Eigen::VectorXd theta_new = theta + step_vec(j) * d_theta;
      for (uint k = 0; k < uwb_vec.size(); ++k)
      {
        res_vec(j) += std::pow(uwb_vec(k) - (theta_new(3) * (theta_new.head(3).transpose() - pose_vec.row(k)).norm() + theta_new(4)), 2);
      }
      res_vec(j) /= uwb_vec.size();
    }

    // Choose minimum residual index
    Eigen::Index step_idx;
    res_vec.minCoeff(&step_idx);

    // Perform parameters update theta(k+1) = theta(k) + step_norm * d_theta
    theta += step_vec(step_idx) * d_theta;

    // If step norm is minimum reduce the step
    if (step_idx == 0)
    {
      step_vec /= 2;
    }

    // Norm of step stopping condition
    if (step_vec(step_idx) < nls_params_.step_cond)
    {
      logger_->info("NlsSolver: Step norm is less than " +
                    std::to_string(nls_params_.step_cond));
      break;
    }

    // Residual stopping condition
    if (res_vec(step_idx) < nls_params_.res_cond)
    {
      logger_->info("NlsSolver: Residual is less than " +
                    std::to_string(nls_params_.res_cond));
      break;
    }

    // Check if maximum number of iteration reached
    if (i == (nls_params_.max_iter - 1))
    {
      logger_->warn("NlsSolver: Maximum number of iterations reached (" +
                    std::to_string(nls_params_.max_iter) + ")");
    }
  }

  // If covariance matrix is not semi-positive-definite return
  if (!isSPD(cov))
  {
    logger_->err("NlsSolver: Covariance matrix is not SPD");
    return false;
  }

  return true;
}

}  // namespace uwb_init
