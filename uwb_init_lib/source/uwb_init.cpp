// Copyright (C) 2022 Alessandro Fornasier, Giulio Delama and Martin Scheiber.
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
// You can contact the authors at <alessandro.fornasier@aau.at>,
// <giulio.delama@aau.at> and <martin.scheiber@aau.at>

#include "uwb_init.hpp"

namespace uwb_init
{
UwbInitializer::UwbInitializer(const LoggerLevel& level, const UwbInitOptions& init_options,
                               const LsSolverOptions& ls_solver_options, const NlsSolverOptions& nls_solver_options)
  : logger_(std::make_shared<Logger>(level))
  , init_options_(init_options)
  , ls_solver_(logger_, init_options, ls_solver_options)
  , nls_solver_(logger_, nls_solver_options)
{
  // Logging
  logger_->info("UwbInitializer: " + std::string(InitMethodString(init_options_.init_method_)));
  logger_->info("UwbInitializer: " + std::string(BiasTypeString(init_options_.bias_type_)));
}

void UwbInitializer::set_init_method(const InitMethod& method)
{
  init_options_.init_method_ = method;

  // Logging
  logger_->info("UwbInitializer: " + std::string(InitMethodString(init_options_.init_method_)));

  // Configure Least Squares Solver
  ls_solver_.configure(init_options_);
}

void UwbInitializer::set_bias_type(const BiasType& type)
{
  init_options_.bias_type_ = type;

  // Logging
  logger_->info("UwbInitializer: " + std::string(BiasTypeString(init_options_.bias_type_)));

  // Configure Least Squares Solver
  ls_solver_.configure(init_options_);
}

const LSSolutions& UwbInitializer::get_ls_solutions() const
{
  if (ls_sols_.empty())
  {
    throw std::runtime_error("UwbInitializer::get_ls_solutions() Required empty solutions");
  }
  return ls_sols_;
}

const NLSSolutions& UwbInitializer::get_nls_solutions() const
{
  if (nls_sols_.empty())
  {
    throw std::runtime_error("UwbInitializer::get_nls_solutions() Required empty solutions");
  }
  return nls_sols_;
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
    logger_->warn("UwbInitializer::feed_uwb(): DISCARDING measurment " + std::to_string(uwb_measurement.distance_) +
                  " from anchor " + std::to_string(uwb_measurement.id_));
  }
}

void UwbInitializer::feed_position(const double timestamp, const Eigen::Vector3d p_UinG)
{
  p_UinG_buffer_.push_back(timestamp, p_UinG);
}

bool UwbInitializer::init_anchors()
{
  // Logging
  logger_->info("UwbInitializer: Performing uwb anchors initialization");

  // Check if position buffer is empty
  if (p_UinG_buffer_.empty())
  {
    logger_->err("UwbInitializer: Initialization FAILED (position buffer is empty)");
    return false;
  }

  // Variable for keeping track if at least one anchor has been correctly initialized
  bool init_successful = false;

  // For each uwb ID extract uwb buffer
  for (const auto& uwb_data : uwb_data_buffer_)
  {
    // Check if uwb buffer is empty
    if (uwb_data.second.empty())
    {
      logger_->err("Anchor[" + std::to_string(uwb_data.first) + "]: Initialization FAILED (uwb buffer is empty)");
      continue;
    }

    // Check if a solution is already present
    if (ls_sols_.find(uwb_data.first) != ls_sols_.end())
    {
      logger_->info("Anchor[" + std::to_string(uwb_data.first) + "]: Already initialized");
      std::stringstream ss;
      ss << "Anchor[" << uwb_data.first << "]\n"
         << "p_AinG = " << ls_sols_.at(uwb_data.first).anchor_.p_AinG_.transpose() << '\n'
         << "Covariance = \n"
         << ls_sols_.at(uwb_data.first).cov_ << '\n'
         << "gamma = " << ls_sols_.at(uwb_data.first).gamma_ << '\n';
      logger_->debug(ss.str());
      init_successful = true;
      continue;
    }

    // Initialize solution and covariance
    Eigen::VectorXd lsSolution;
    Eigen::MatrixXd cov;

    // Solve ls problem and initialize anchor
    if (ls_solver_.solve_ls(uwb_data.second, p_UinG_buffer_, lsSolution, cov))
    {
      // Assign values to parameters
      Eigen::Vector3d p_AinG = lsSolution.head(3);
      double const_bias = 0.0;

      // If constant bias was estimated assign the value
      if (init_options_.bias_type_ == BiasType::CONST_BIAS)
      {
        const_bias = lsSolution(3);
      }

      // Initialize anchor and solution
      UwbAnchor new_anchor(uwb_data.first, p_AinG);
      LSSolution ls_sol(new_anchor, const_bias, cov);

      // Add solution to vector
      ls_sols_.emplace(std::make_pair(uwb_data.first, ls_sol));

      logger_->info("Anchor[" + std::to_string(uwb_data.first) + "]: Correctly initialized");
      std::stringstream ss;
      ss << "Anchor[" << uwb_data.first << "]\n"
         << "p_AinG = " << ls_sols_.at(uwb_data.first).anchor_.p_AinG_.transpose() << '\n'
         << "Covariance = \n"
         << ls_sols_.at(uwb_data.first).cov_ << '\n'
         << "gamma = " << ls_sols_.at(uwb_data.first).gamma_ << '\n';
      logger_->debug(ss.str());
      init_successful = true;
    }
    // Can not initialize
    else
    {
      logger_->err("Anchor[" + std::to_string(uwb_data.first) + "]: Can not be initialized");
    }
  }

  // Initialization finished
  logger_->info("UwbInitializer: Initialization complete");
  return init_successful;
}

bool UwbInitializer::refine_anchors()
{
  // Logging
  logger_->info("UwbInitializer: Performing anchors refinement");

  // Check if pose buffer is empty
  if (p_UinG_buffer_.empty())
  {
    logger_->err("UwbInitializer: Refinement FAILED (position buffer is empty)");
    return false;
  }

  // Variable for keeping track if at least one anchor has been correctly refined
  bool refine_successful = false;

  // For each uwb ID extract LS solution
  for (const auto& ls_sol : ls_sols_)
  {
    // Check if uwb buffer is empty
    if (uwb_data_buffer_.at(ls_sol.first).empty())
    {
      logger_->err("Anchor[" + std::to_string(ls_sol.first) + "]: Refinement FAILED (uwb buffer is empty)");
      continue;
    }

    // Check if a solution is already present
    if (nls_sols_.find(ls_sol.first) != nls_sols_.end())
    {
      logger_->info("Anchor[" + std::to_string(ls_sol.first) + "]: Already refined");
      std::stringstream ss;
      ss << "Anchor[" << ls_sol.first << "]\n"
         << "p_AinG = " << nls_sols_.at(ls_sol.first).anchor_.p_AinG_.transpose() << '\n'
         << "Covariance = \n"
         << nls_sols_.at(ls_sol.first).cov_ << '\n'
         << "gamma = " << nls_sols_.at(ls_sol.first).gamma_ << '\n'
         << "beta = " << nls_sols_.at(ls_sol.first).beta_ << '\n';
      logger_->debug(ss.str());
      refine_successful = true;
      continue;
    }

    // Initialize solution and covariance
    Eigen::VectorXd theta(5);
    Eigen::MatrixXd cov;

    // Theta0 (p_AinG, gamma, beta)
    theta << ls_sol.second.anchor_.p_AinG_, 1.0, ls_sol.second.gamma_;

    // Perform nonlinear optimization
    if (nls_solver_.solve_nls(uwb_data_buffer_.at(ls_sol.first), p_UinG_buffer_, theta, cov))
    {
      // Initialize anchor and solution
      UwbAnchor new_anchor(ls_sol.first, theta.head(3));
      NLSSolution nls_sol(new_anchor, theta(3), theta(4), cov);

      // Add solution to vector
      nls_sols_.emplace(std::make_pair(ls_sol.first, nls_sol));

      // Refine successful
      logger_->info("Anchor[" + std::to_string(ls_sol.first) + "]: Correctly refined");
      std::stringstream ss;
      ss << "Anchor[" << ls_sol.first << "]\n"
         << "p_AinG = " << nls_sols_.at(ls_sol.first).anchor_.p_AinG_.transpose() << '\n'
         << "Covariance = \n"
         << nls_sols_.at(ls_sol.first).cov_ << '\n'
         << "gamma = " << nls_sols_.at(ls_sol.first).gamma_ << '\n'
         << "beta = " << nls_sols_.at(ls_sol.first).beta_ << '\n';
      logger_->debug(ss.str());
      refine_successful = true;
    }
    // Can not refine
    else
    {
      logger_->err("Anchor[" + std::to_string(ls_sol.first) + "]: Can not be refined");
    }
  }

  // Refinement complete
  logger_->info("UwbInitializer: Refinement complete");
  return refine_successful;
}

}  // namespace uwb_init
