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
  /// cell length (defualt suggested value: 10)
  uint cell_len_;

  /// Population size (defualt suggested value: 10)
  uint pop_size_;

  /// Number of generations (defualt suggested value: 3000)
  uint itr_num_;

  /// Crossover probability (defualt suggested value: 0.5)
  double pc_;

  /// Mutaion probability (defualt suggested value: 0.2)
  double pm_;

  /// X-dimention of the inital grid search (defualt suggested value: 2)
  uint x_n_;

  /// Y-dimention of the inital grid search (defualt suggested value: 2)
  uint y_n_;

  /// Z-dimention of the inital grid search (defualt suggested value: 3)
  uint z_n_;

  /// Maximum side length along X-axis (defualt suggested value: 3)
  double side_x_;

  /// Maximum side length along Y-axis (defualt suggested value: 4)
  double side_y_;

  /// Maximum side length along Z-axis (defualt suggested value: 50)
  double side_z_;

  /// Minimum distance from the ground (defualt suggested value: 1)
  double z_min_;

  /// x-coordinate of the centroid of the environment
  double C_e_x_;

  /// y-coordinate of the centroid of the environment
  double C_e_y_;

  /// x-side length of each small cell (side_x / x_n)
  double x_s_;

  /// x-side length of each small cell (side_y / y_n)
  double y_s_;

  /// x-side length of each small cell (side_z / z_n)
  double z_s_;

  /// Origin (The environment centroid)
  Eigen::Vector3d C_e_;

  /// The volume centroid
  Eigen::Vector3d C_v_;

  PlannerOptions(const uint& cell_len, const uint& pop_size, const uint& itr_num, const double& pc, const double& pm,
                 const uint& x_n, const uint& y_n, const uint& z_n, const double& side_x, const double& side_y,
                 const double& side_z, const double& z_min, const double& C_e_x, const double& C_e_y)
    : cell_len_(cell_len)
    , pop_size_(pop_size)
    , itr_num_(itr_num)
    , pc_(pc)
    , pm_(pm)
    , x_n_(x_n)
    , y_n_(y_n)
    , z_n_(z_n)
    , side_x_(side_x)
    , side_y_(side_y)
    , side_z_(side_z)
    , z_min_(z_min)
    , C_e_x_(C_e_x)
    , C_e_y_(C_e_y)
  {
    C_e_ << C_e_x_, C_e_y_, side_z_ / 2 + z_min_;
    x_s_ = side_x_ / x_n_;
    y_s_ = side_y_ / y_n_;
    z_s_ = side_z_ / z_n_;
    C_v_ << side_x_ / 2, side_y_ / 2, side_z_ / 2;
  }

};  // struct PlannerOptions

}  // namespace uwb_init

#endif  // UWB_INIT_PLANNER_OPTIONS_HPP_
