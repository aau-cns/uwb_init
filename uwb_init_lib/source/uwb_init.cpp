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

#include <assert.h>

namespace uwb_init
{
UwbInitializer::UwbInitializer(const LoggerLevel& level, std::shared_ptr<UwbInitOptions>&& init_options,
                               std::unique_ptr<LsSolverOptions>&& ls_solver_options,
                               std::unique_ptr<NlsSolverOptions>&& nls_solver_options,
                               std::unique_ptr<PlannerOptions>&& planner_options)
  : logger_(std::make_shared<Logger>(level))
  , init_options_(std::move(init_options))
  , ls_solver_(logger_, std::move(ls_solver_options))
  , nls_solver_(logger_, std::move(nls_solver_options))
  , planner_(logger_, std::move(planner_options))
{
  // Debug assertation
  assert(logger_ != nullptr);
  assert(init_options_ != nullptr);
}

void UwbInitializer::set_init_method(const InitMethod& method)
{
  // Logging
  logger_->info("UwbInitializer::set_init_method(): " + std::string(InitMethodString(method)));

  // Configure Least Squares Solver
  ls_solver_.configure(method);
}

void UwbInitializer::set_bias_type(const BiasType& type)
{

  // Logging
  logger_->info("UwbInitializer:set_bias_type(): " + std::string(BiasTypeString(type)));

  // Configure Least Squares Solver
  ls_solver_.configure(type);

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
    feed_uwb(timestamp, uwb_measurements[i]);
  }
}

void UwbInitializer::feed_uwb(const double timestamp, const UwbData uwb_measurement)
{
  uint Anchor_ID = uwb_measurement.id_Anchor;
  uint Tag_ID = uwb_measurement.id_Tag;

  // Check validity
  if (uwb_measurement.valid_)
  {
    if (uwb_data_buffer_.find(Anchor_ID) == uwb_data_buffer_.end()) {
      uwb_data_buffer_.insert({Anchor_ID, UwbDataBuffer()});
    }
    if (uwb_data_buffer_[Anchor_ID].find(Tag_ID) == uwb_data_buffer_[Anchor_ID].end()) {
      uwb_data_buffer_[Anchor_ID].insert({Tag_ID, TimedBuffer<UwbData>()});
    }


    uwb_data_buffer_[Anchor_ID][Tag_ID].push_back(timestamp, uwb_measurement);
    //logger_->debug("UwbInitializer::feed_uwb(): added measurement from tag_ID=" + std::to_string(Tag_ID)
    //               + " to anchor_ID=" + std::to_string(Anchor_ID) + " at timestamp " + std::to_string(timestamp));

  }
  else
  {
    logger_->warn("UwbInitializer::feed_uwb(): DISCARDING measurment " + std::to_string(uwb_measurement.distance_) +
                  " from anchor " + std::to_string(uwb_measurement.id_Anchor));
  }
}

void UwbInitializer::feed_position(const double timestamp, const Eigen::Vector3d p_UinG, const uint Tag_ID)
{
  if (p_UinG_buffer_.find(Tag_ID) == p_UinG_buffer_.end())
  {
    p_UinG_buffer_.insert({Tag_ID, PositionBuffer()});
  }

  p_UinG_buffer_[Tag_ID].push_back(timestamp, p_UinG);

  //logger_->debug("UwbInitializer::feed_position(): added position from [" + std::to_string(Tag_ID) + "] at timestamp " + std::to_string(timestamp));
}

///
/// TODO: this initialization routine should be reworked and cleaned.
///
bool UwbInitializer::init_anchors()
{
  // Logging
  logger_->info("UwbInitializer: Performing uwb anchors initialization");

  // Counter for initialized anchors
  uint init_count = 0;

  // Clear already existing solutions
  clear_solutions();

  // If position buffer is empty return false
  if (p_UinG_buffer_.empty())
  {
    logger_->err("UwbInitializer: Initialization FAILED (position buffer is empty)");
    return false;
  }

  // For each uwb anchor ID extract uwb buffer and use the same tag position buffer p_UinG_buffer_: multiple anchors to one tag
  for (const auto& e : uwb_data_buffer_)
  {
    uint const ID_Anchor = e.first;
    UwbDataPerTag const& uwb_data = e.second;

    // Logging
    logger_->info("Anchor[" + std::to_string(ID_Anchor) + "]: Starting initialization");

    // If uwb buffer is empty try next anchor
    if (uwb_data.empty())
    {
      logger_->warn("Anchor[" + std::to_string(ID_Anchor) + "]: Initialization FAILED (uwb buffer is empty)");
      continue;
    }

    // Initialize LS solution and covariance
    Eigen::VectorXd lsSolution;
    Eigen::MatrixXd lsCov;

    // Initialize NLS solution and covariance
    Eigen::VectorXd nlsSolution;
    Eigen::MatrixXd nlsCov;



    UwbDataPerTag uwb_data_inliers;
    // Try to solve LS problem
    if (init_options_->enable_ls_ && ls_solver_.solve_ls(uwb_data, p_UinG_buffer_, lsSolution, lsCov, uwb_data_inliers) && lsSolution.size() >= 3)
    {
      // Logging
      logger_->info("Anchor[" + std::to_string(ID_Anchor) + "]: Coarse solution found");
      std::vector<size_t> ID_Tags;
      for(auto const&e : uwb_data_inliers) { ID_Tags.push_back(e.first); }

      LSSolution ls_sol = to_LSSolution(lsSolution, lsCov, ID_Anchor, ID_Tags);
      logger_->debug(" * " + ls_sol.str());
      std::stringstream ss; ss << lsSolution.transpose();
      logger_->debug("* theta:" +  ss.str());
      // Add solution to vector
      ls_sols_.emplace(std::make_pair(ID_Anchor, ls_sol));
    }
    else
    {
      // If LS fails assign empty solution
      logger_->warn("Anchor[" + std::to_string(ID_Anchor) +
                    "]: Coarse initialization FAILED. Assigning empty "
                    "solution");
      lsSolution = Eigen::VectorXd::Zero(3);
      uwb_data_inliers = uwb_data;
    }

    // assign initial guess for NlsSolver
    nlsSolution = lsSolution;

    // Perform nonlinear optimization
    if (nls_solver_.levenbergMarquardt(uwb_data_inliers, p_UinG_buffer_, nlsSolution, nlsCov, uwb_data_inliers))
    {
      // Logging
      logger_->info("Anchor[" + std::to_string(ID_Anchor) + "]: Solutiuon refined");

      // Increase counter
      init_count += 1;

      std::vector<size_t> ID_Tags;
      for(auto const&e : uwb_data_inliers) { ID_Tags.push_back(e.first); }

      NLSSolution nls_sol = to_NLSSolution(nlsSolution, nlsCov, ID_Anchor, ID_Tags);
      // Compute standard deviation
      Eigen::VectorXd std_dev = nls_sol.cov_.diagonal().cwiseSqrt();

      // Add solution to vector
      nls_sols_.emplace(std::make_pair(ID_Anchor, nls_sol));

      // Refine successful
      logger_->info("Anchor[" + std::to_string(ID_Anchor) + "]: Correctly initialized");
      logger_->debug(" * " + nls_sol.str());
      std::stringstream ss; ss << nlsSolution.transpose();
      logger_->debug("* theta: " +  ss.str());

    }
    // If NLS fails continue with next anchor
    else
    {
      logger_->warn("Anchor[" + std::to_string(ID_Anchor) + "]: Initialization FAILED");
    }
  }

  // Logging initialization results
  if (init_count == 0)
  {
    logger_->err("UwbInitializer: Initialization FAILED (no anchor initialized)");
    return false;
  }
  else if (init_count < init_options_->min_num_anchors_)
  {
    logger_->err("UwbInitializer: Initialization FAILED (initialized " + std::to_string(init_count) +
                 " anchors out of " + std::to_string(init_options_->min_num_anchors_) + "required)");
    return false;
  }

  logger_->info("UwbInitializer: Initialization SUCCESSFUL (initialized " + std::to_string(init_count) + " anchors)");

  // Return true if at least min_num_anchors_ have been initialized
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
    size_t const ID_Anchor = nls_sol.first;
    UwbDataPerTag const& uwb_data = uwb_data_buffer_.at(ID_Anchor);

      // If uwb buffer is empty return false
    if (uwb_data.empty())
      {
        logger_->err("Anchor[" + std::to_string(nls_sol.first) + "]: Refinement FAILED (uwb buffer is empty)");
      return false;
    }
    // Initialize NLS solution and covariance
    Eigen::VectorXd theta = nls_sol.second.to_vec();
    Eigen::MatrixXd cov;

    std::vector<size_t> ID_Tags;
    for(auto const&e : uwb_data) {
      ID_Tags.push_back(e.first);
    }

    // Perform nonlinear optimization
    if (nls_solver_.levenbergMarquardt(uwb_data, p_UinG_buffer_, theta, cov))
    {

      // Initialize anchor and solution
      NLSSolution refined_sol = to_NLSSolution(theta, cov, ID_Anchor, ID_Tags);

      // Compute standard deviation
      Eigen::VectorXd std_dev = refined_sol.cov_.diagonal().cwiseSqrt();

      // Add solution to vector
      refined_sols_.emplace(std::make_pair(ID_Anchor, refined_sol));

      // Refine successful
      logger_->info("Anchor[" + std::to_string(ID_Anchor) + "]: Correctly refined");
      logger_->debug("* " + refined_sol.str());
      std::stringstream ss; ss << theta.transpose();
      logger_->debug("* theta: " +  ss.str());
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

LSSolution UwbInitializer::to_LSSolution(const Eigen::VectorXd &lsSolution,
                                         const Eigen::MatrixXd &lsCov,
                                         const size_t ID_Anchor,
                                         const std::vector<size_t> &ID_Tags)
{

  size_t const num_Tags = ID_Tags.size();
  Eigen::MatrixXd Sigma = lsCov;

          // Initialize new anchor
  UwbAnchor new_anchor(ID_Anchor, lsSolution.head(3));

          // Initialize constant bias
  std::unordered_map<uint, double> const_biases; // gammas

          // If constant bias was estimated assign the value, else resize covariance
  if (ls_solver_.bias_tpye() != BiasType::NO_BIAS)
  {
    int idx = 3;
    for(auto const &e : p_UinG_buffer_) {

      if(idx < lsSolution.rows()) {
        const_biases.insert({e.first, lsSolution(idx)});
      }
      idx++;
    }
  }
  else
  {
    Sigma.conservativeResizeLike(Eigen::MatrixXd::Zero(3+num_Tags, 3+num_Tags));
    for(size_t idx = 3; idx <3+num_Tags; idx++) {
      Sigma(idx, idx) = init_options_->const_bias_prior_cov_;
    }
    for(auto const &ID_Tag : ID_Tags) {
      const_biases.insert({ID_Tag, 0.0});
    }
  }

  // Initialize solution
  return LSSolution(new_anchor, const_biases, lsCov);
}

NLSSolution UwbInitializer::to_NLSSolution(const Eigen::VectorXd &nlsSolution, const Eigen::MatrixXd &nlsCov, const size_t ID_Anchor, const std::vector<size_t> &ID_Tags)
{
  size_t const num_Tags = ID_Tags.size();

  // Initialize new anchor
  UwbAnchor new_anchor(ID_Anchor, nlsSolution.head(3));
  logger_->debug(" * " + new_anchor.str());

          // Initialize biases
  std::unordered_map<uint, double> const_biases; // gammas
  std::unordered_map<uint, double>  range_biases; // betas

  Eigen::MatrixXd Sigma = nlsCov;

  // Switch bias type and resize covariance
  switch (nls_solver_.bias_type())
  {
    case BiasType::ALL_BIAS:
    {
      size_t idx = 3;
      for(auto const& ID_Tag : ID_Tags)
      {
        const_biases.insert({ID_Tag, nlsSolution(idx)});
        idx++;
      }
      for(auto const& ID_Tag : ID_Tags)
      {
        range_biases.insert({ID_Tag, nlsSolution(idx)});
        idx++;
      }

      break;
    }
    case BiasType::CONST_BIAS:
    {
      size_t idx = 3;
      for(auto const& ID_Tag : ID_Tags)
      {
        const_biases.insert({ID_Tag, nlsSolution(idx)});
        idx++;
      }
      Sigma.conservativeResizeLike(Eigen::MatrixXd::Zero(3+2*num_Tags, 3+2*num_Tags));
      for(size_t idx = 3+num_Tags; idx < 3+2*num_Tags; idx++)
      {
        Sigma(idx, idx) = init_options_->dist_bias_prior_cov_;
      }

      for(auto const &ID_Tag : ID_Tags) {
        range_biases.insert({ID_Tag, 1.0});
      }
      break;
    }
    case BiasType::NO_BIAS:
    {
      Sigma.conservativeResizeLike(Eigen::MatrixXd::Zero(3+2*num_Tags, 3+2*num_Tags));
      for(size_t idx = 3; idx < 3+num_Tags; idx++ )
      {
        Sigma(idx, idx) = init_options_->dist_bias_prior_cov_;
      }
      for(size_t idx = 3+num_Tags; idx < 3+2*num_Tags; idx++ )
      {
        Sigma(idx, idx) = init_options_->dist_bias_prior_cov_;
      }
      for(auto const& ID_Tag : ID_Tags)
      {
        const_biases.insert({ID_Tag, 0.0});
      }
      for(auto const& ID_Tag : ID_Tags)
      {
        range_biases.insert({ID_Tag, 1.0});
      }
      break;
    }
  }

  // Initialize solution
  return NLSSolution(new_anchor, const_biases, range_biases, Sigma);
}





}  // namespace uwb_init
