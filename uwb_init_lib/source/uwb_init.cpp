﻿// Copyright (C) 2022 Alessandro Fornasier, Giulio Delama and Martin Scheiber.
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

#include <assert.h>

namespace uwb_init
{
UwbInitializer::UwbInitializer(const LoggerLevel& level, std::shared_ptr<UwbInitOptions>&& init_options,
                               std::unique_ptr<LsSolverOptions>&& ls_solver_options,
                               std::unique_ptr<NlsSolverOptions>&& nls_solver_options,
                               std::unique_ptr<PlannerOptions>&& planner_options)
  : logger_(std::make_shared<Logger>(level))
  , init_options_(std::move(init_options))
  , ls_solver_(logger_, init_options_, std::move(ls_solver_options))
  , nls_solver_(logger_, std::move(nls_solver_options))
  , planner_(logger_, std::move(planner_options))
{
  // Debug assertation
  assert(logger_ != nullptr);
  assert(init_options_ != nullptr);

  // Logging
  logger_->info("UwbInitializer: " + std::string(InitMethodString(init_options_->init_method_)));
  logger_->info("UwbInitializer: " + std::string(BiasTypeString(init_options_->bias_type_)));
}

void UwbInitializer::set_init_method(const InitMethod& method)
{
  init_options_->init_method_ = method;

  // Logging
  logger_->info("UwbInitializer: " + std::string(InitMethodString(init_options_->init_method_)));

  // Configure Least Squares Solver
  ls_solver_.configure(init_options_);
}

void UwbInitializer::set_bias_type(const BiasType& type)
{
  init_options_->bias_type_ = type;

  // Logging
  logger_->info("UwbInitializer: " + std::string(BiasTypeString(init_options_->bias_type_)));

  // Configure Least Squares Solver
  ls_solver_.configure(init_options_);
}

const LSSolutions& UwbInitializer::get_ls_solutions() const
{
  if (ls_sols_.empty())
  {
    throw std::runtime_error("UwbInitializer::get_ls_solutions(): Required empty vector.");
  }
  return ls_sols_;
}

const NLSSolutions& UwbInitializer::get_nls_solutions() const
{
  if (nls_sols_.empty())
  {
    throw std::runtime_error("UwbInitializer::get_nls_solutions(): Required empty vector.");
  }
  return nls_sols_;
}

const NLSSolutions& UwbInitializer::get_refined_solutions() const
{
  if (refined_sols_.empty())
  {
    throw std::runtime_error("UwbInitializer::get_refined_solutions(): Required empty vector.");
  }
  return refined_sols_;
}

const Waypoints& UwbInitializer::get_waypoints() const
{
  if (opt_wps_.empty())
  {
    throw std::runtime_error("UwbInitializer::get_waypoints(): Required empty vector.");
  }
  return opt_wps_;
}

void UwbInitializer::clear_buffers()
{
  uwb_data_buffer_.clear();
  p_UinG_buffer_.clear();

  // Logging
  logger_->debug("UwbInitializer::clear_buffers(): Buffers cleared");
}

void UwbInitializer::clear_solutions()
{
  ls_sols_.clear();
  nls_sols_.clear();
  refined_sols_.clear();
  opt_wps_.clear();

  // Logging
  logger_->debug("UwbInitializer::clear_solutions(): Solutions cleared");
}

void UwbInitializer::reset()
{
  clear_buffers();
  clear_solutions();

  // Logging
  logger_->debug("UwbInitializer::reset(): Reset completed");
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
      logger_->debug("UwbInitializer::feed_uwb(): added measurement from anchor " +
                     std::to_string(uwb_measurements[i].id_) + " at timestamp " + std::to_string(timestamp));
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
    logger_->debug("UwbInitializer::feed_uwb(): added measurement from anchor " + std::to_string(uwb_measurement.id_) +
                   " at timestamp " + std::to_string(timestamp));
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
  logger_->debug("UwbInitializer::feed_position(): added position at timestamp " + std::to_string(timestamp));
}

bool UwbInitializer::init_anchors()
{
  // Logging
  logger_->info("UwbInitializer: Performing uwb anchors initialization");

  // Clear already existing solutions
  clear_solutions();

  // If position buffer is empty return false
  if (p_UinG_buffer_.empty())
  {
    logger_->err("UwbInitializer: Initialization FAILED (position buffer is empty)");
    return false;
  }

  // For each uwb ID extract uwb buffer
  for (const auto& uwb_data : uwb_data_buffer_)
  {
    // If uwb buffer is empty return false
    if (uwb_data.second.empty())
    {
      logger_->err("Anchor[" + std::to_string(uwb_data.first) + "]: Initialization FAILED (uwb buffer is empty)");
      return false;
    }

    // Initialize LS solution and covariance
    Eigen::VectorXd lsSolution;
    Eigen::MatrixXd lsCov;

    // Initialize NLS solution and covariance
    Eigen::VectorXd nlsSolution = Eigen::VectorXd::Zero(5);
    Eigen::MatrixXd nlsCov;

    // Try to solve LS problem
    if (ls_solver_.solve_ls(uwb_data.second, p_UinG_buffer_, lsSolution, lsCov))
    {
      // Assign values to parameters
      Eigen::Vector3d p_AinG = lsSolution.head(3);
      double const_bias = 0.0;

      // If constant bias was estimated assign the value
      if (init_options_->bias_type_ == BiasType::CONST_BIAS)
      {
        const_bias = lsSolution(3);
      }

      // Initialize anchor and solution
      UwbAnchor new_anchor(uwb_data.first, p_AinG);
      LSSolution ls_sol(new_anchor, const_bias, lsCov);

      // Add solution to vector
      ls_sols_.emplace(std::make_pair(uwb_data.first, ls_sol));

      // Initial guess (p_AinG, gamma, beta)
      nlsSolution << ls_sol.anchor_.p_AinG_, ls_sol.gamma_, 1.0;

      logger_->info("Anchor[" + std::to_string(uwb_data.first) + "]: Coarse solution found");
    }
    // If LS fails assign empty solution
    else
    {
      logger_->warn("Anchor[" + std::to_string(uwb_data.first) +
                    "]: Coarse initialization FAILED. Assigning empty "
                    "solution");

      // Initialize anchor and solution
      UwbAnchor new_anchor(uwb_data.first, Eigen::Vector3d::Zero());
      Eigen::MatrixXd cov = 0.1 * Eigen::MatrixXd::Identity(3, 3);
      LSSolution ls_sol(new_anchor, 0.0, lsCov);
      ls_sols_.emplace(std::make_pair(uwb_data.first, ls_sol));

      // Initial guess (p_AinG, gamma, beta)
      nlsSolution << ls_sol.anchor_.p_AinG_, ls_sol.gamma_, 1.0;
    }

    // Perform nonlinear optimization
    if (nls_solver_.levenbergMarquardt(uwb_data.second, p_UinG_buffer_, nlsSolution, nlsCov))
    {
      // Initialize anchor and solution
      UwbAnchor new_anchor(uwb_data.first, nlsSolution.head(3));
      NLSSolution nls_sol(new_anchor, nlsSolution(3), nlsSolution(4), nlsCov);

      // Compute standard deviation
      Eigen::VectorXd std_dev = nls_sol.cov_.diagonal().cwiseSqrt();

      // Add solution to vector
      nls_sols_.emplace(std::make_pair(uwb_data.first, nls_sol));

      // Refine successful
      logger_->info("Anchor[" + std::to_string(uwb_data.first) + "]: Correctly initialized");
      std::stringstream ss;
      ss << "Anchor[" << uwb_data.first << "]\n"
         << "p_AinG = " << nls_sols_.at(uwb_data.first).anchor_.p_AinG_.transpose() << '\n'
         << "Covariance = \n"
         << nls_sols_.at(uwb_data.first).cov_ << '\n'
         << "gamma = " << nls_sols_.at(uwb_data.first).gamma_ << '\n'
         << "beta = " << nls_sols_.at(uwb_data.first).beta_ << '\n'
         << "Standard deviation = " << std_dev.transpose();
      logger_->debug(ss.str());
    }
    // If NLS fails return false
    else
    {
      logger_->err("Anchor[" + std::to_string(uwb_data.first) + "]: Initialization FAILED");
      return false;
    }
  }

  // Initialization finished
  logger_->info("UwbInitializer: Initialization SUCCESSFUL");
  return true;

}  // namespace uwb_init

bool UwbInitializer::compute_waypoints(const Eigen::Vector3d pos_k)
{
  // Logging
  logger_->info("UwbInitializer: Calculating optimal waypoints");

  // If no anchors have been initialized return false
  if (nls_sols_.empty())
  {
    logger_->err("UwbInitializer: Anchors are not initialized. Perform initialization first");
    return false;
  }

  // Check if optimal waypoints have been already computed
  if (!opt_wps_.empty())
  {
    logger_->warn("UwbInitializer: Clearing already computed waypoints");
    opt_wps_.clear();
  }

  // Construct the map of the uwb anchors (matrix Nx3)
  Eigen::MatrixXd map = Eigen::MatrixXd::Zero(ls_sols_.size(), 3);
  uint idx = 0;
  for (const auto& nls_sol : nls_sols_)
  {
    map.row(idx) << nls_sol.second.anchor_.p_AinG_.transpose();
    idx += 1;
  }

  // Compute optimal waypoints given the map and the current position
  Eigen::MatrixXd wps = planner_.generate_wps(map, pos_k);

  // Debug sstream
  std::stringstream ss;
  ss << "\nCurrent tag position:" << pos_k.transpose() << '\n'
     << "Current map:\n"
     << map << '\n'
     << "Computed optimal waypoints:\n";

  // Save optimal waypoints in data struct
  auto sep = "]\n";
  for (uint idx = 0; idx < wps.rows(); ++idx)
  {
    // Change separator to avoid \n at the end
    if (idx == wps.rows() - 1)
    {
      sep = "]";
    }

    // Fill optimal waypoints
    opt_wps_.emplace_back(Waypoint(wps.row(idx)));

    // Debug sstream
    ss << "[" << opt_wps_[idx].x_ << ", " << opt_wps_[idx].y_ << ", " << opt_wps_[idx].z_ << sep;
  }

  // Logging results
  logger_->debug(ss.str());

  return true;
}

bool UwbInitializer::refine_anchors()
{
  // Logging
  logger_->info("UwbInitializer: Performing anchors refinement");

  // If position buffer is empty return false
  if (p_UinG_buffer_.empty())
  {
    logger_->err("UwbInitializer: Refinement FAILED (position buffer is empty)");
    return false;
  }

  // Clear Refinement solutions
  refined_sols_.clear();

  // For each uwb ID extract uwb buffer
  for (const auto& nls_sol : nls_sols_)
  {
    // If uwb buffer is empty return false
    if (uwb_data_buffer_.at(nls_sol.first).empty())
    {
      logger_->err("Anchor[" + std::to_string(nls_sol.first) + "]: Refinement FAILED (uwb buffer is empty)");
      return false;
    }

    // Initialize NLS solution and covariance
    Eigen::VectorXd theta = Eigen::VectorXd::Zero(5);
    Eigen::MatrixXd cov;

    // Initial guess (p_AinG, gamma, beta)
    theta << nls_sol.second.anchor_.p_AinG_, nls_sol.second.gamma_, nls_sol.second.beta_;

    // Perform nonlinear optimization
    if (nls_solver_.levenbergMarquardt(uwb_data_buffer_.at(nls_sol.first), p_UinG_buffer_, theta, cov))
    {
      // Initialize anchor and solution
      UwbAnchor new_anchor(nls_sol.first, theta.head(3));
      NLSSolution refined_sol(new_anchor, theta(3), theta(4), cov);

      // Compute standard deviation
      Eigen::VectorXd std_dev = refined_sol.cov_.diagonal().cwiseSqrt();

      // Add solution to vector
      refined_sols_.emplace(std::make_pair(nls_sol.first, refined_sol));

      // Refine successful
      logger_->info("Anchor[" + std::to_string(nls_sol.first) + "]: Correctly refined");
      std::stringstream ss;
      ss << "Anchor[" << nls_sol.first << "]\n"
         << "p_AinG = " << refined_sols_.at(nls_sol.first).anchor_.p_AinG_.transpose() << '\n'
         << "Covariance = \n"
         << refined_sols_.at(nls_sol.first).cov_ << '\n'
         << "gamma = " << refined_sols_.at(nls_sol.first).gamma_ << '\n'
         << "beta = " << refined_sols_.at(nls_sol.first).beta_ << '\n'
         << "Standard deviation = " << std_dev.transpose();
      logger_->debug(ss.str());
    }
    // If NLS fails return false
    else
    {
      logger_->err("Anchor[" + std::to_string(nls_sol.first) + "]: Refinement FAILED");
      return false;
    }
  }

  // Refinement finished
  logger_->info("UwbInitializer: Refinement SUCCESSFUL");
  return true;
}

}  // namespace uwb_init
