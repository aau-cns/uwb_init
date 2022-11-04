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
UwbInitializer::UwbInitializer(const LoggerLevel& level)
  : logger_(std::make_shared<Logger>(level))
{
  // Initialize least squares solver
  ls_solver_ = std::make_shared<LsSolver>(logger_, init_params_);

  // Logging
  logger_->info("UwbInitializer: " + get_init_method());
  logger_->info("UwbInitializer: " + get_init_variables());
}

UwbInitializer::UwbInitializer(const UwbInitOptions init_params, const LoggerLevel& level)
  : logger_(std::make_shared<Logger>(level)), init_params_(init_params)
{
  // Initialize least squares solver
  ls_solver_ = std::make_shared<LsSolver>(logger_, init_params_);

  // Logging
  logger_->info("UwbInitializer: " + get_init_method());
  logger_->info("UwbInitializer: " + get_init_variables());
}

void UwbInitializer::set_init_method_single()
{
  init_params_.init_method = UwbInitOptions::InitMethod::SINGLE;
  ls_solver_->configure(init_params_);

  // Logging
  logger_->info("UwbInitializer: " + get_init_method());
}

void UwbInitializer::set_init_method_double()
{
  init_params_.init_method = UwbInitOptions::InitMethod::DOUBLE;
  ls_solver_->configure(init_params_);

  // Logging
  logger_->info("UwbInitializer: " + get_init_method());
}

void UwbInitializer::set_init_unbiased()
{
  init_params_.init_variables = UwbInitOptions::InitVariables::NO_BIAS;
  ls_solver_->configure(init_params_);

  // Logging
  logger_->info("UwbInitializer: " + get_init_variables());
}

void UwbInitializer::set_init_const_bias()
{
  init_params_.init_variables = UwbInitOptions::InitVariables::CONST_BIAS;
  ls_solver_->configure(init_params_);

  // Logging
  logger_->info("UwbInitializer: " + get_init_variables());
}

std::string const UwbInitializer::get_init_method() const
{
  switch (init_params_.init_method)
  {
    case UwbInitOptions::InitMethod::SINGLE:
      return "InitMethod::SINGLE";
    case UwbInitOptions::InitMethod::DOUBLE:
      return "InitMethod::DOUBLE";
  }
  return " ";
}

std::string const UwbInitializer::get_init_variables() const
{
  switch (init_params_.init_variables)
  {
    case UwbInitOptions::InitVariables::NO_BIAS:
      return "InitVariables::NO_BIAS";
    case UwbInitOptions::InitVariables::CONST_BIAS:
      return "InitVariables::CONST_BIAS";
  }
  return " ";
}

void UwbInitializer::clear_buffers()
{
  uwb_data_buffer_.clear();
  p_UinG_buffer_.clear();
}

void UwbInitializer::clear_solutions()
{
  ls_sols_.clear();
  nls_sols_.clear();
}

void UwbInitializer::reset()
{
  clear_buffers();
  clear_solutions();
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

void UwbInitializer::feed_uwb(const double timestamp, const UwbData uwb_measurement)
{
  // Check validity and if measurement is actual bigger than 0.0
  if (uwb_measurement.valid_ && uwb_measurement.distance_ > 0.0)
  {
      // Push back element to buffer
      uwb_data_buffer_[uwb_measurement.id_].push_back(timestamp, uwb_measurement);
  }
  else
  {
      logger_->warn("UwbInitializer::feed_uwb(): DISCARDING measurment " +
                    std::to_string(uwb_measurement.distance_) + " from anchor " +
                    std::to_string(uwb_measurement.id_));
  }
}

void UwbInitializer::feed_pose(const double timestamp, const Eigen::Vector3d p_UinG)
{
  p_UinG_buffer_.push_back(timestamp, p_UinG);
}

bool UwbInitializer::init_anchors()
{
  // Logging
  logger_->info("UwbInitializer: Performing uwb anchors initialization");

  // Check if pose buffer is empty
  if (p_UinG_buffer_.empty())
  {
    logger_->err("UwbInitializer: Initialization FAILED (pose buffer is empty)");
    return false;
  }

  // Variable for keeping track if all anchors have been correctly initialized
  bool init_successful = true;

  // For each uwb ID extract uwb buffer
  for (const auto& uwb_data : uwb_data_buffer_)
  {
    // Check if uwb buffer is empty
    if (uwb_data.second.empty())
    {
      logger_->err("Anchor[" + std::to_string(uwb_data.first) + "]: Initialization FAILED (uwb buffer is empty)");
      init_successful = false;
      continue;
    }

    // Check if a solution is already present
    if (ls_sols_.contains(uwb_data.first))
    {
      logger_->info("Anchor[" + std::to_string(uwb_data.first) + "]: Already initialized");
      std::cout << "Anchor[" << uwb_data.first << "]: p_AinG = " << ls_sols_.at(uwb_data.first).p_AinG().transpose() << "\n" << std::endl;
      std::cout << "Anchor[" << uwb_data.first << "]: covariance =\n" << ls_sols_.at(uwb_data.first).cov_ << "\n" << std::endl;
      std::cout << "Anchor[" << uwb_data.first << "]: gamma = " << ls_sols_.at(uwb_data.first).gamma_ << "\n" << std::endl;
      continue;
    }

    // Initialize solution and covariance
    Eigen::VectorXd lsSolution;
    Eigen::MatrixXd cov;

    // Solve ls problem and initialize anchor
    if (ls_solver_->solve_ls(uwb_data.second, p_UinG_buffer_, lsSolution, cov))
    {
      // Assign values to parameters
      Eigen::Vector3d p_AinG = lsSolution.head(3);
      double const_bias = 0.0;

      // If constant bias was estimated assign the value
      if (init_params_.init_variables == UwbInitOptions::InitVariables::CONST_BIAS)
      {
        const_bias = lsSolution(3);
      }

      // Initialize anchor and solution
      UwbAnchor new_anchor(uwb_data.first, p_AinG);
      LSSolution ls_sol(new_anchor, const_bias, cov);

      // Add solution to vector
      ls_sols_.emplace(std::make_pair(uwb_data.first, ls_sol));

      logger_->info("Anchor[" + std::to_string(uwb_data.first) + "]: Correctly initialized");
      std::cout << "Anchor[" << uwb_data.first << "]: p_AinG = " << ls_sols_.at(uwb_data.first).p_AinG().transpose() << "\n" << std::endl;
      std::cout << "Anchor[" << uwb_data.first << "]: covariance =\n" << ls_sols_.at(uwb_data.first).cov_ << "\n" << std::endl;
      std::cout << "Anchor[" << uwb_data.first << "]: gamma = " << ls_sols_.at(uwb_data.first).gamma_ << "\n" << std::endl;
    }
    // Can not initialize
    else
    {
      logger_->err("Anchor[" + std::to_string(uwb_data.first) + "]: Not initialized correctly");
      init_successful = false;
    }
  }

  // If initialization is not successful return false
  if (!(init_successful))
  {
    logger_->err("UwbInitializer: Initialization FAILED");
    return false;
  }

  // Else all anchors have been initialized correctly
  logger_->info("UwbInitializer: Initialization complete");
  return true;
}

bool UwbInitializer::refine_anchors()
{
  // Logging
  logger_->info("UwbInitializer: Performing anchors refinement");

  // Check if pose buffer is empty
  if (p_UinG_buffer_.empty())
  {
    logger_->err("UwbInitializer: Operation FAILED (pose buffer is empty)");
    return false;
  }

  // Variable for keeping track if all anchors have been correctly refined
  bool refine_successful = true;

  // For each uwb ID extract LS solution
  for (const auto& ls_sol : ls_sols_)
  {
    // Check if uwb buffer is empty
    if (uwb_data_buffer_.at(ls_sol.first).empty())
    {
      logger_->err("Anchor[" + std::to_string(ls_sol.first) + "]: Operation FAILED (uwb buffer is empty)");
      refine_successful = false;
      continue;
    }
    // Check if a solution is already present
    else if (nls_sols_.contains(ls_sol.first))
    {
      logger_->info("Anchor[" + std::to_string(ls_sol.first) + "]: Already refined");
      std::cout << "Anchor[" << ls_sol.first << "]: p_AinG = " << nls_sols_.at(ls_sol.first).p_AinG().transpose() << "\n" << std::endl;
      std::cout << "Anchor[" << ls_sol.first << "]: covariance =\n" << nls_sols_.at(ls_sol.first).cov_ << "\n" << std::endl;
      std::cout << "Anchor[" << ls_sol.first << "]: beta = " << nls_sols_.at(ls_sol.first).beta_ << "\n" << std::endl;
      std::cout << "Anchor[" << ls_sol.first << "]: gamma = " << nls_sols_.at(ls_sol.first).gamma_ << "\n" << std::endl;
      continue;
    }
    // Perform nonlinear optimization
    else if (solve_nls(ls_sol.first))
    {
      // Refine successful
      logger_->info("Anchor[" + std::to_string(ls_sol.first) + "]: Correctly refined");
      std::cout << "Anchor[" << ls_sol.first << "]: p_AinG = " << nls_sols_.at(ls_sol.first).p_AinG().transpose() << "\n" << std::endl;
      std::cout << "Anchor[" << ls_sol.first << "]: covariance =\n" << nls_sols_.at(ls_sol.first).cov_ << "\n" << std::endl;
      std::cout << "Anchor[" << ls_sol.first << "]: beta = " << nls_sols_.at(ls_sol.first).beta_ << "\n" << std::endl;
      std::cout << "Anchor[" << ls_sol.first << "]: gamma = " << nls_sols_.at(ls_sol.first).gamma_ << "\n" << std::endl;
      continue;
    }
    // Can not refine
    else
    {
      logger_->err("Anchor[" + std::to_string(ls_sol.first) + "]: Not refined correctly");
      refine_successful = false;
      continue;
    }
  }

  // If refinement is not successful return false
  if (!(refine_successful))
  {
    logger_->err("UwbInitializer: Refinement FAILED");
    return false;
  }

  // Else all anchors have been initialized correctly
  logger_->info("UwbInitializer: Refinement complete");
  return true;
}

bool UwbInitializer::solve_nls(const uint& anchor_id)
{
  // Parameter vector (p_AinG, gamma, beta)
  Eigen::VectorXd theta(5);
  theta << ls_sols_.at(anchor_id).anchor_.p_AinG_, 1.0, ls_sols_.at(anchor_id).gamma_;

  // Step norm vector
  Eigen::VectorXd step_vec = nls_params_.step_vec;

  // Data vectors initialization
  Eigen::VectorXd uwb_vec = Eigen::VectorXd::Zero(uwb_data_buffer_.at(anchor_id).size());
  Eigen::MatrixXd pose_vec = Eigen::MatrixXd::Zero(uwb_vec.size(), 3);
  Eigen::MatrixXd cov;

  // Create consistent data vectors
  for (uint i = 0; i < uwb_vec.size(); ++i)
  {
    uwb_vec(i) = uwb_data_buffer_.at(anchor_id)[i].second.distance_;
    pose_vec.row(i) = p_UinG_buffer_.get_at_timestamp(uwb_data_buffer_.at(anchor_id)[i].first);
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
      J.row(j) << theta(3) * (theta(0) - pose_vec(j, 0)) / (theta.head(3) - pose_vec.row(j).transpose()).norm(),
              theta(3) * (theta(1) - pose_vec(j, 1)) / (theta.head(3) - pose_vec.row(j).transpose()).norm(),
              theta(3) * (theta(2) - pose_vec(j, 2)) / (theta.head(3) - pose_vec.row(j).transpose()).norm(),
              (theta.head(3) - pose_vec.row(j).transpose()).norm(),
              1;
      // Residual res = y - f(theta) =  uwb_meas - (beta * ||p_AinG - p_UinG|| + gamma)
      res(j) = uwb_vec(j) - (theta(3) * (theta.head(3) - pose_vec.row(j).transpose()).norm() + theta(4));
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
        res_vec(j) += std::pow(uwb_vec(k) - (theta_new(3) * (theta_new.head(3) - pose_vec.row(j).transpose()).norm() + theta(4)), 2);
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
      logger_->info("Anchor[" + std::to_string(anchor_id) + "]: Step norm is less than " +
                    std::to_string(nls_params_.step_cond));
      break;
    }

    // Residual stopping condition
    if (res_vec(step_idx) < nls_params_.res_cond)
    {
      logger_->info("Anchor[" + std::to_string(anchor_id) + "]: Residual is less than " +
                    std::to_string(nls_params_.res_cond));
      break;
    }

    // Check if maximum number of iteration reached
    if (i == (nls_params_.max_iter - 1))
    {
      logger_->warn("Anchor[" + std::to_string(anchor_id) + "]: Maximum number of iterations reached (" +
                    std::to_string(nls_params_.max_iter) + ")");
    }
  }

  // If covariance matrix is not semi-positive-definite return
  if (!isSPD(cov))
  {
    return false;
  }

  // Initialize anchor and solution
  UwbAnchor new_anchor(anchor_id, theta.head(3));
  NLSSolution nls_sol(new_anchor, theta(3), theta(4), cov);

  // Add solution to vector
  nls_sols_.emplace(std::make_pair(anchor_id, nls_sol));

  return true;
}

}  // namespace uwb_init
