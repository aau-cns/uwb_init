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
LsSolver::LsSolver(const std::shared_ptr<Logger> logger, const std::shared_ptr<UwbInitOptions>& init_options,
                   std::unique_ptr<LsSolverOptions>&& ls_solver_options)
  : logger_(std::move(logger)), solver_options_(std::move(ls_solver_options))
{
  // Debug assertation
  assert(logger_ != nullptr);
  assert(solver_options_ != nullptr);

  // Configure solver
  configure(init_options);

  // Logging
  logger_->info("LsSolver: Initialized");
  logger_->debug("LsSolver options: sigma_pos = " + std::to_string(solver_options_->sigma_pos_));
  logger_->debug("LsSolver options: sigma_meas = " + std::to_string(solver_options_->sigma_meas_));
}

void LsSolver::configure(const std::shared_ptr<UwbInitOptions>& init_options)
{
  // Bind LS-problem initialization function based on selected method and model variables
  switch (init_options->init_method_)
  {
    // Single measurement initialization method
    case InitMethod::SINGLE:
      switch (init_options->bias_type_)
      {
        // Unbiased measurement model (only distance between tag and uwb anchor)
        case BiasType::NO_BIAS:
          ls_problem = std::bind(&LsSolver::ls_single_no_bias, this, std::placeholders::_1, std::placeholders::_2,
                                 std::placeholders::_3, std::placeholders::_4, std::placeholders::_5);
          break;
        // Constant bias measurement model (distance and constant bias)
        case BiasType::CONST_BIAS:
          ls_problem = std::bind(&LsSolver::ls_single_const_bias, this, std::placeholders::_1, std::placeholders::_2,
                                 std::placeholders::_3, std::placeholders::_4, std::placeholders::_5);
          break;
        // Constant bias measurement model (distance and constant bias)
        case BiasType::ALL_BIAS:
          ls_problem = std::bind(&LsSolver::ls_single_const_bias, this, std::placeholders::_1, std::placeholders::_2,
                                 std::placeholders::_3, std::placeholders::_4, std::placeholders::_5);
          break;
      }
      break;
    // Double measurements initialization method
    case InitMethod::DOUBLE:
      switch (init_options->bias_type_)
      {
        // Unbiased measurement model (only distance between tag and uwb anchor)
        case BiasType::NO_BIAS:
          ls_problem = std::bind(&LsSolver::ls_double_no_bias, this, std::placeholders::_1, std::placeholders::_2,
                                 std::placeholders::_3, std::placeholders::_4, std::placeholders::_5);
          break;
        // Constant bias measurement model (distance and constant bias)
        case BiasType::CONST_BIAS:
          ls_problem = std::bind(&LsSolver::ls_double_const_bias, this, std::placeholders::_1, std::placeholders::_2,
                                 std::placeholders::_3, std::placeholders::_4, std::placeholders::_5);
          break;
        // Constant bias measurement model (distance and constant bias)
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

bool LsSolver::solve_ls(const TimedBuffer<UwbData>& uwb_data, const PositionBuffer& p_UinG_buffer,
                        Eigen::VectorXd& lsSolution, Eigen::MatrixXd& cov)
{
  // Coefficient matrix and measurement vector initialization
  Eigen::MatrixXd coeffs;
  Eigen::VectorXd meas;
  Eigen::VectorXd sigma;

  // Initialize least squares problem
  if (!(ls_problem(uwb_data, p_UinG_buffer, coeffs, meas, sigma)))
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
  if (!isSPD(cov))
  {
    logger_->err("LsSolver: Covariance matrix is not SPD");
    return false;
  }

  return true;
}

bool LsSolver::ls_single_const_bias(const TimedBuffer<UwbData>& uwb_data, const PositionBuffer& p_UinG_buffer,
                                    Eigen::MatrixXd& A, Eigen::VectorXd& b, Eigen::VectorXd& s)
{
  // Const bias single:
  // [    0   ,     1   ,    2    ,  3 ,      4            ]
  // [p_AinG_x, p_AinG_y, p_AinG_z,  k , norm(p_AinG)^2-k^2]

  // Coefficient matrix and measurement vector initialization
  A = Eigen::MatrixXd::Zero(uwb_data.size(), 5);
  b = Eigen::VectorXd::Zero(A.rows());
  s = Eigen::VectorXd::Ones(A.rows());

  // Fill the coefficient matrix and the measurement vector
  for (uint i = 0; i < uwb_data.size(); ++i)
  {
    // Get position at uwb timestamp
    Eigen::Vector3d p_UinG = p_UinG_buffer.get_at_timestamp(uwb_data[i].first);

    // Fill row(i) of A and b
    A.row(i) << -2 * p_UinG.x(), -2 * p_UinG.y(), -2 * p_UinG.z(), 2 * uwb_data[i].second.distance_, 1;
    b(i) = std::pow(uwb_data[i].second.distance_, 2) - std::pow(p_UinG.norm(), 2);
  }

  return true;
}

bool LsSolver::ls_single_no_bias(const TimedBuffer<UwbData>& uwb_data, const PositionBuffer& p_UinG_buffer,
                                 Eigen::MatrixXd& A, Eigen::VectorXd& b, Eigen::VectorXd& s)
{
  // Unbiased single:
  // [    0    ,    1   ,    2   ,        3       ]
  // [p_AinG_x, p_AinG_y, p_AinG_z, norm(p_AinG)^2]

  // Coefficient matrix and measurement vector initialization
  A = Eigen::MatrixXd::Zero(uwb_data.size(), 4);
  b = Eigen::VectorXd::Zero(A.rows());
  s = Eigen::VectorXd::Ones(A.rows());

  // Fill the coefficient matrix and the measurement vector
  for (uint i = 0; i < uwb_data.size(); ++i)
  {
    // Get position at uwb timestamp
    Eigen::Vector3d p_UinG = p_UinG_buffer.get_at_timestamp(uwb_data[i].first);

    // Fill row(i) of A and b
    A.row(i) << -2 * p_UinG.x(), -2 * p_UinG.y(), -2 * p_UinG.z(), 1;
    b(i) = std::pow(uwb_data[i].second.distance_, 2) - std::pow(p_UinG.norm(), 2);
  }

  return true;
}

bool LsSolver::ls_double_const_bias(const TimedBuffer<UwbData>& uwb_data, const PositionBuffer& p_UinG_buffer,
                                    Eigen::MatrixXd& A, Eigen::VectorXd& b, Eigen::VectorXd& s)
{
  // Const bias double:
  // [    0   ,     1   ,    2    ,  3 ]
  // [p_AinG_x, p_AinG_y, p_AinG_z,  k ]

  // Number of data points must be at least 2
  if (uwb_data.size() < 2)
  {
    return false;
  }

  // Coefficient matrix and measurement vector initialization
  A = Eigen::MatrixXd::Zero(uwb_data.size() - 1, 4);
  b = Eigen::VectorXd::Zero(A.rows());
  s = Eigen::VectorXd::Zero(A.rows());

  // Find pivot index (minimize weight uwb_dist^2*sigma_d + p_UinG'*sigma_p*p_UinG)
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

  // Position and distance at pivot_index
  Eigen::Vector3d p_UinG_pivot = p_UinG_buffer.get_at_timestamp(uwb_data[pivot_idx].first);
  double uwb_pivot = uwb_data[pivot_idx].second.distance_;

  // Fill the coefficient matrix and the measurement vector
  uint j = 0;
  for (uint i = 0; i < uwb_data.size(); ++i)
  {
    // Skip pivot
    if (i == pivot_idx)
    {
      continue;
    }

    // Get position at timestamp
    Eigen::Vector3d p_UinG = p_UinG_buffer.get_at_timestamp(uwb_data[i].first);

    // Fill row(j) of A and b
    A.row(j) << -(p_UinG.x() - p_UinG_pivot.x()), -(p_UinG.y() - p_UinG_pivot.y()), -(p_UinG.z() - p_UinG_pivot.z()),
        (uwb_data[i].second.distance_ - uwb_pivot);

    b(j) = 0.5 * (std::pow(uwb_data[i].second.distance_, 2) - std::pow(uwb_pivot, 2) -
                  (std::pow(p_UinG.norm(), 2) - std::pow(p_UinG_pivot.norm(), 2)));

    s(j) = std::pow(uwb_data[i].second.distance_, 2) * solver_options_->sigma_meas_ +
           p_UinG.transpose() * Eigen::Matrix3d::Identity() * solver_options_->sigma_pos_ * p_UinG + weight_pivot;

    // Increment row index
    j += 1;
  }

  return true;
}

bool LsSolver::ls_double_no_bias(const TimedBuffer<UwbData>& uwb_data, const PositionBuffer& p_UinG_buffer,
                                 Eigen::MatrixXd& A, Eigen::VectorXd& b, Eigen::VectorXd& s)
{
  // Unbiased double:
  // [    0    ,    1   ,    2    ]
  // [p_AinG_x, p_AinG_y, p_AinG_z]

  // Number of data points must be at least 2
  if (uwb_data.size() < 2)
  {
    return false;
  }

  // Coefficient matrix and measurement vector initialization
  A = Eigen::MatrixXd::Zero(uwb_data.size() - 1, 3);
  b = Eigen::VectorXd::Zero(A.rows());
  s = Eigen::VectorXd::Zero(A.rows());

  // Find pivot index (minimize weight uwb_dist^2*sigma_d + p_UinG'*sigma_p*p_UinG)
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

  // Position and distance at pivot_index
  Eigen::Vector3d p_UinG_pivot = p_UinG_buffer.get_at_timestamp(uwb_data[pivot_idx].first);
  double uwb_pivot = uwb_data[pivot_idx].second.distance_;

  // Fill the coefficient matrix and the measurement vector
  uint j = 0;
  for (uint i = 0; i < uwb_data.size(); ++i)
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

  return true;
}

}  // namespace uwb_init
