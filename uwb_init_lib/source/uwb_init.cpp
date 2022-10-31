// // Copyright (C) 2021 Martin Scheiber, Alessandro Fornasier
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
// // You can contact the authors at <martin.scheiber@aau.at>,
// // <alessandro.fornasier@aau.at> and <giulio.delama@aau.at>

#include "uwb_init.hpp"

namespace uwb_init
{
UwbInitializer::UwbInitializer(UwbInitOptions& params, const LoggerLevel& level)
  : logger_(std::make_shared<Logger>(level)), params_(params)
{
  // Bind LS-problem initialization function based on selected method and model variables
  switch (params_.init_method)
  {
    case UwbInitOptions::InitMethod::SINGLE:
      switch (params_.init_variables)
      {
        case UwbInitOptions::InitVariables::NO_BIAS:
          ls_problem = std::bind(&UwbInitializer::ls_single_no_bias, this, std::placeholders::_1, std::placeholders::_2,
                                 std::placeholders::_3);
          break;

        case UwbInitOptions::InitVariables::CONST_BIAS:
          ls_problem = std::bind(&UwbInitializer::ls_single_const_bias, this, std::placeholders::_1,
                                 std::placeholders::_2, std::placeholders::_3);
          break;
      }
      break;

    case UwbInitOptions::InitMethod::DOUBLE:
      switch (params_.init_variables)
      {
        case UwbInitOptions::InitVariables::NO_BIAS:
          ls_problem = std::bind(&UwbInitializer::ls_double_no_bias, this, std::placeholders::_1, std::placeholders::_2,
                                 std::placeholders::_3);
          break;

        case UwbInitOptions::InitVariables::CONST_BIAS:
          ls_problem = std::bind(&UwbInitializer::ls_double_const_bias, this, std::placeholders::_1,
                                 std::placeholders::_2, std::placeholders::_3);
          break;
      }
      break;
  }

  // Logging
  logger_->info("UwbInitializer: " + params_.InitMethod());
  logger_->info("UwbInitializer: " + params_.InitVariables());
}

void UwbInitializer::clear_buffers()
{
  // Clear buffers
  uwb_data_buffer_.clear();
  p_UinG_buffer.clear();
}

void UwbInitializer::reset()
{
  clear_buffers();

  // Reset status
  ls_sols_.clear();
  nls_sols_.clear();
}

void UwbInitializer::feed_uwb(const double timestamp, const std::vector<UwbData> uwb_measurements)
{
  // Add measurements to data buffer
  for (uint i = 0; i < uwb_measurements.size(); ++i)
  {
    // Check validity and if measurement is actual bigger than 0.0
    if (uwb_measurements[i].valid_ && uwb_measurements[i].distance_ > 0.0)
    {
      // Push back element to buffer
      uwb_data_buffer_[uwb_measurements[i].id_].push_back(timestamp, uwb_measurements[i]);
    }
    else
    {
      logger_->warn("UwbInitializer::feed_uwb(): DISCARDING measurment " +
                    std::to_string(uwb_measurements[i].distance_) + " from anchor " +
                    std::to_string(uwb_measurements[i].id_));
    }
  }
}

void UwbInitializer::feed_pose(const double timestamp, const Eigen::Vector3d p_UinG)
{
  p_UinG_buffer.push_back(timestamp, p_UinG);
}

bool UwbInitializer::init_anchors()
{
  // Logging
  logger_->info("UwbInitializer::init_anchors(): Performing uwb anchors initialization");

  // Check if pose buffer is empty
  if (p_UinG_buffer.empty())
  {
    logger_->err("UwbInitializer::init_anchors(): Initialization FAILED (pose buffer is empty)");
    return false;
  }

  // Variable for keeping track if all anchors have been correctly initialized
  bool init_successful = true;

  // For each uwb ID extract buffer
  for (const auto& uwb_data : uwb_data_buffer_)
  {
    // Check if uwb buffer is empty
    if (uwb_data.second.empty())
    {
      logger_->err("Anchor[" + std::to_string(uwb_data.first) + "]: Initialization FAILED (uwb buffer is empty)");
      return false;
    }

    // Check if anchor is already initialized
    if (ls_sols_.contains(uwb_data.first))
    {
      continue;
    }

    // Solve ls problem and initialize anchor
    else if (solve_ls(uwb_data.first))
    {
      logger_->info("Anchor[" + std::to_string(uwb_data.first) + "]: Correctly initialized");
      continue;
    }
    else
    {
      logger_->err("Anchor[" + std::to_string(uwb_data.first) + "]: Not initialized correctly");
      init_successful = false;
    }
  }

  // If initialization is not successful return
  if (!(init_successful))
  {
    logger_->err("UwbInitializer::init_anchors(): Initialization FAILED");
    return false;
  }

  // Else all anchors have been initialized correctly
  logger_->info("UwbInitializer::init_anchors(): Initialization complete");
  return true;
}

bool UwbInitializer::refine_anchors()
{
  // Logging
  logger_->info("UwbInitializer::refine_anchors(): Performing anchors refinement");

  // Check if pose buffer is empty
  if (p_UinG_buffer.empty())
  {
    logger_->err("UwbInitializer::refine_anchors(): Operation FAILED (pose buffer is empty)");
    return false;
  }

  for (const auto& ls_sol : ls_sols_)
  {
    // Check if uwb buffer is empty
    if (uwb_data_buffer_.at(ls_sol.first).empty())
    {
      logger_->err("UwbInitializer::refine_anchors(): Operation FAILED (uwb buffer is empty)");
      return false;
    }

    // Check if anchor is already initialized
    if (nls_sols_.contains(ls_sol.first))
    {
      continue;
    }

    if (!solve_nls(ls_sol.first))
    {
      return false;
    }

    // Refine successful
    logger_->info("Anchor[" + std::to_string(ls_sol.first) + "]: Correctly refined");
  }

  return true;
}

bool UwbInitializer::solve_ls(const uint& anchor_id)
{
  // Coefficient matrix and measurement vector initialization
  Eigen::MatrixXd coeffs;
  Eigen::VectorXd meas;

  // Initialize least squares problem
  if (!(ls_problem(uwb_data_buffer_.at(anchor_id), coeffs, meas)))
  {
    logger_->err("Anchor[" + std::to_string(anchor_id) + "]: Least Squares problem can not be initialized");
    return false;
  }

  // Check the coefficient matrix condition number and solve the LS problem
  Eigen::BDCSVD<Eigen::MatrixXd> svd(coeffs, Eigen::ComputeThinU | Eigen::ComputeThinV);

  // Solve LS problem
  Eigen::VectorXd lsSolution = svd.solve(meas);

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

  // Assign values to parameters
  Eigen::Vector3d p_AinG = lsSolution.head(3);
  double const_bias = 0.0;

  // If constant bias was estimated assign the value
  if (lsSolution.size() > 3)
  {
    const_bias = lsSolution(3);
  }

  // Compute estimation Covariance
  Eigen::MatrixXd cov = (std::pow((coeffs * lsSolution - meas).norm(), 2) / (coeffs.rows() * coeffs.cols())) *
                        (svd.matrixV().inverse().transpose() * svd.singularValues().asDiagonal().inverse() *
                         svd.singularValues().asDiagonal().inverse() * svd.matrixV().inverse());

  // Initialize anchor and solution
  UwbAnchor new_anchor(anchor_id, p_AinG);
  LSSolution ls_sol(new_anchor, const_bias, cov);

  // Add solution to vector
  ls_sols_.emplace(std::make_pair(anchor_id, ls_sol));

  return true;
}

bool UwbInitializer::solve_nls(const uint& anchor_id)
{
  // Parameter vector (p_AinG, gamma, beta)
  Eigen::VectorXd theta(5);
  theta << ls_sols_.at(anchor_id).anchor_.p_AinG_, 1.0, ls_sols_.at(anchor_id).gamma_;

  // Step norm vector
  Eigen::VectorXd step_vec = params_.step_vec;

  // Data vectors initialization
  Eigen::VectorXd uwb_vec = Eigen::VectorXd::Zero(uwb_data_buffer_.at(anchor_id).size());
  Eigen::MatrixXd pose_vec = Eigen::MatrixXd::Zero(uwb_vec.size(), 3);

  // Create consistent data vectors
  for (uint i = 0; i < uwb_vec.size(); ++i)
  {
    uwb_vec(i) = uwb_data_buffer_.at(anchor_id)[i].second.distance_;
    pose_vec.row(i) = p_UinG_buffer.get_at_timestamp(uwb_data_buffer_.at(anchor_id)[i].first);
  }

  // Check for consistency
  if (uwb_vec.size() != pose_vec.rows())
  {
    // Refine unsuccessful
    logger_->err("Anchor[" + std::to_string(anchor_id) +
                 "]: Can not be refined (data vectors have different dimensions)");
    return false;
  }

  // Non-linear Least Squares
  for (uint i = 0; i < params_.max_iter; ++i)
  {
    // Jacobian and residual initialization
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(uwb_vec.size(), 5);
    Eigen::VectorXd res = Eigen::VectorXd::Zero(uwb_vec.size());
    Eigen::VectorXd res_vec = Eigen::VectorXd::Zero(step_vec.size());

    // Compute Jacobian and residual
    for (uint j = 0; j < uwb_vec.size(); ++j)
    {
      // Jacobian [df/dp_AinG, df/dbeta, df/dgamma]
      Eigen::VectorXd row(J.cols());
      Eigen::VectorXd theta_pose = theta.head(3);
      Eigen::VectorXd pose_vec_row = pose_vec.row(j);
      Eigen::VectorXd diff = theta_pose - pose_vec_row;
      double diff_norm = diff.norm();
      row << theta(3) * (theta(0) - pose_vec(j, 0)) / diff_norm,
          theta(3) * (theta(1) - pose_vec(j, 1)) / diff_norm,
          theta(3) * (theta(2) - pose_vec(j, 2)) / diff_norm,
          diff_norm, 1;
      J.row(j) = row.transpose();
      // Residual res = y - f(theta) =  uwb_meas - (beta * ||p_AinG - p_UinG|| + gamma)
      res(j) = uwb_vec(j) - (theta(3) * diff_norm + theta(4));
    }

    // Calculate Moore-Penrose Pseudo-Inverse of matrix J
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
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

    // Calculate residual for each step
    for (uint j = 0; j < step_vec.size(); ++j)
    {
      Eigen::VectorXd theta_new = theta + step_vec(j) * d_theta;
      for (uint k = 0; k < uwb_vec.size(); ++k)
      {
        Eigen::VectorXd theta_pose = theta_new.head(3);
        Eigen::VectorXd pose_vec_row = pose_vec.row(j);
        Eigen::VectorXd diff = theta_pose - pose_vec_row;
        double diff_norm = diff.norm();
        res_vec(j) +=
            std::pow(uwb_vec(k) - (theta_new(3) * diff_norm + theta(4)), 2);
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
    if (step_vec(step_idx) < params_.step_cond)
    {
      logger_->info("Anchor[" + std::to_string(anchor_id) + "]: Step norm is less than " +
                    std::to_string(params_.step_cond));
      break;
    }

    // Residual stopping condition
    if (res_vec(step_idx) < params_.res_cond)
    {
      logger_->info("Anchor[" + std::to_string(anchor_id) + "]: Residual is less than " +
                    std::to_string(params_.res_cond));
      break;
    }

    // Check if maximum number of iteration reached
    if (i == (params_.max_iter - 1))
    {
      logger_->warn("Anchor[" + std::to_string(anchor_id) + "]: Maximum number of iterations reached (" +
                    std::to_string(params_.max_iter) + ")");
    }
  }

  // Initialize anchor and solution
  UwbAnchor new_anchor(anchor_id, theta.head(3));
  Eigen::MatrixXd cov = Eigen::MatrixXd::Ones(5, 5);
  NLSSolution nls_sol(new_anchor, theta(3), theta(4), cov);

  // Add solution to vector
  nls_sols_.emplace(std::make_pair(anchor_id, nls_sol));

  return true;
}

bool UwbInitializer::ls_single_const_bias(const TimedBuffer<UwbData>& uwb_data, Eigen::MatrixXd& A, Eigen::VectorXd& b)
{
  // Const bias single:
  // [    0   ,     1   ,    2    ,  3 ,      4            ]
  // [p_AinG_x, p_AinG_y, p_AinG_z,  k , norm(p_AinG)^2-k^2]

  // Coefficient matrix and measurement vector initialization
  A = Eigen::MatrixXd::Zero(uwb_data.size(), 5);
  b = Eigen::VectorXd::Zero(uwb_data.size());

  // Fill the coefficient matrix and the measurement vector
  for (uint i = 0; i < uwb_data.size(); ++i)
  {
    // Get position at uwb timestamp
    Eigen::Vector3d p_UinG = p_UinG_buffer.get_at_timestamp(uwb_data[i].first);
    Eigen::VectorXd row(A.cols());
    row << -2 * p_UinG.x(), -2 * p_UinG.y(), -2 * p_UinG.z(), 2 * uwb_data[i].second.distance_, 1;
    A.row(i) = row.transpose();
    b(i) = std::pow(uwb_data[i].second.distance_, 2) - std::pow(p_UinG.norm(), 2);
  }

  return true;
}

bool UwbInitializer::ls_single_no_bias(const TimedBuffer<UwbData>& uwb_data, Eigen::MatrixXd& A, Eigen::VectorXd& b)
{
  // Unbiased single:
  // [    0    ,    1   ,    2   ,        3       ]
  // [p_AinG_x, p_AinG_y, p_AinG_z, norm(p_AinG)^2]

  // Coefficient matrix and measurement vector initialization
  A = Eigen::MatrixXd::Zero(uwb_data.size(), 4);
  b = Eigen::VectorXd::Zero(uwb_data.size());

  // Fill the coefficient matrix and the measurement vector
  for (uint i = 0; i < uwb_data.size(); ++i)
  {
    // Get position at uwb timestamp
    Eigen::Vector3d p_UinG = p_UinG_buffer.get_at_timestamp(uwb_data[i].first);
    Eigen::VectorXd row(4);
    row << -2 * p_UinG.x(), -2 * p_UinG.y(), -2 * p_UinG.z(), 1;
    A.row(i) = row.transpose();
    b(i) = std::pow(uwb_data[i].second.distance_, 2) - std::pow(p_UinG.norm(), 2);
  }

  return true;
}

bool UwbInitializer::ls_double_const_bias(const TimedBuffer<UwbData>& uwb_data, Eigen::MatrixXd& A, Eigen::VectorXd& b)
{
  // Const bias double:
  // [    0   ,     1   ,    2    ,  3 ]
  // [p_AinG_x, p_AinG_y, p_AinG_z,  k ]

  // Number of parameters for selected model and method
  const uint8_t n_params = 4;

  // Coefficient matrix and measurement vector initialization
  std::vector<double> coeffs_vec;
  std::vector<double> meas_vec;

  // Find pivot (minimum distance) index
  uint pivot_idx = 0;
  for (uint i = 0; i < uwb_data.size(); ++i)
  {
    if (uwb_data[i].second.distance_ < uwb_data[pivot_idx].second.distance_)
    {
      pivot_idx = i;
    }
  }

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
    Eigen::VectorXd row(n_params);
    row << -(p_UinG.x() - p_UinG_pivot.x()), -(p_UinG.y() - p_UinG_pivot.y()), -(p_UinG.z() - p_UinG_pivot.z()),
        (uwb_data[i].second.distance_ - uwb_pivot);

    coeffs_vec.push_back(row(0));
    coeffs_vec.push_back(row(1));
    coeffs_vec.push_back(row(2));
    coeffs_vec.push_back(row(3));
    meas_vec.push_back(0.5 * (std::pow(uwb_data[i].second.distance_, 2) - std::pow(uwb_pivot, 2) -
                              (std::pow(p_UinG.norm(), 2) - std::pow(p_UinG_pivot.norm(), 2))));
  }

  // Assertation, check vectors size
  assert(coeffs_vec.size() == n_params * meas_vec.size());

  // Map vectors to Eigen matrices
  A = Eigen::MatrixXd(Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      coeffs_vec.data(), coeffs_vec.size() / n_params, n_params));
  b = Eigen::VectorXd(
      Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>(meas_vec.data(), meas_vec.size(), 1));

  return true;
}

bool UwbInitializer::ls_double_no_bias(const TimedBuffer<UwbData>& uwb_data, Eigen::MatrixXd& A, Eigen::VectorXd& b)
{
  // Unbiased double:
  // [    0    ,    1   ,    2    ]
  // [p_AinG_x, p_AinG_y, p_AinG_z]

  // Number of parameters for selected model and method
  const uint8_t n_params = 3;

  // Coefficient matrix and measurement vector initialization
  std::vector<double> coeffs_vec;
  std::vector<double> meas_vec;

  // Find pivot (minimum distance) index
  uint pivot_idx = 0;
  for (uint i = 0; i < uwb_data.size(); ++i)
  {
    if (uwb_data[i].second.distance_ < uwb_data[pivot_idx].second.distance_)
    {
      pivot_idx = i;
    }
  }

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

    // Get position at uwb timestamp
    Eigen::Vector3d p_UinG = p_UinG_buffer.get_at_timestamp(uwb_data[i].first);
    Eigen::VectorXd row(n_params);
    row << -(p_UinG.x() - p_UinG_pivot.x()), -(p_UinG.y() - p_UinG_pivot.y()), -(p_UinG.z() - p_UinG_pivot.z());

    coeffs_vec.push_back(row(0));
    coeffs_vec.push_back(row(1));
    coeffs_vec.push_back(row(2));
    meas_vec.push_back(0.5 * (std::pow(uwb_data[i].second.distance_, 2) - std::pow(uwb_pivot, 2) -
                              (std::pow(p_UinG.norm(), 2) - std::pow(p_UinG_pivot.norm(), 2))));
  }

  // Assertation, check vectors size
  assert(coeffs_vec.size() == n_params * meas_vec.size());

  // Map vectors to Eigen matrices
  A = Eigen::MatrixXd(Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      coeffs_vec.data(), coeffs_vec.size() / n_params, n_params));
  b = Eigen::VectorXd(
      Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>(meas_vec.data(), meas_vec.size(), 1));

  return true;
}

}  // namespace uwb_init
