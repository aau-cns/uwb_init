// Copyright (C) 2022 Giulio Delama.
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
// You can contact the author at <giulio.delama@aau.at>

#ifndef UWB_INIT_PLANNER_OPTIONS_HPP_
#define UWB_INIT_PLANNER_OPTIONS_HPP_

#include <Eigen/Eigen>

namespace uwb_init
{
///
/// \brief The PlannerOptions struct is an object containing all 'static' parameters used
/// for the waypoints generator.
///
struct PlannerOptions
{
  // Independent parameters

  uint cell_len_{ 10 };

  uint pop_size_{ 10 };                                 // Population size

  uint itr_num_{ 3000 };                                // Number of generations

  double pc_{ 0.5 };                                    // Crossover probability

  double pm_{ 0.2 };                                    // Mutaion probability

  uint x_n_{ 2 };                                       // X-dimention of the inital grid search

  uint y_n_{ 2 };                                       // Y-dimention of the inital grid search

  uint z_n_{ 2 };                                       // Z-dimention of the inital grid search

  double side_x_{ 8 };                                  // Maximum side length along X-axis

  double side_y_{ 8 };                                  // Maximum side length along Y-axis

  double side_z_{ 8 };                                  // Maximum side length along Z-axis

  double z_min_{ 1 };                                   // Minimum distance from the ground

  // Dependent parameters

  double x_s_;                                       // x-side length of each small cell (side_x / x_n)

  double y_s_;                                       // x-side length of each small cell (side_y / y_n)

  double z_s_;                                       // x-side length of each small cell (side_z / z_n)

  Eigen::Vector3d C_e_;        // Origin (The environment centroid)

  Eigen::Vector3d C_v_;        // The volume centroid

  PlannerOptions()
  {
    C_e_ << 0, 0, side_z_/2 + z_min_;
    x_s_ = side_x_ / x_n_;
    y_s_ = side_y_ / y_n_;
    z_s_ = side_z_ / z_n_;
    C_v_ << side_x_/2, side_y_/2, side_z_/2;
  }

  PlannerOptions(const uint& cell_len, const uint& pop_size, const uint& itr_num, const double& pc,
                 const double& pm, const uint& x_n, const uint& y_n, const uint& z_n,
                 const double& side_x, const double& side_y, const double& side_z, const double& z_min)
      : cell_len_(cell_len), pop_size_(pop_size), itr_num_(itr_num), pc_(pc), pm_(pm), x_n_(x_n), y_n_(y_n),
        z_n_(z_n), side_x_(side_x), side_y_(side_y), side_z_(side_z), z_min_(z_min)
  {
    C_e_ << 0, 0, side_z_/2 + z_min_;
    x_s_ = side_x_ / x_n_;
    y_s_ = side_y_ / y_n_;
    z_s_ = side_z_ / z_n_;
    C_v_ << side_x_/2, side_y_/2, side_z_/2;
  }

};  // struct PlannerOptions

}  // namespace uwb_init

#endif  // UWB_INIT_PLANNER_OPTIONS_HPP_
