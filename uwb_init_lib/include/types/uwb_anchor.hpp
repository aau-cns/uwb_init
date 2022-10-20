// Copyright (C) 2021 Martin Scheiber, Control of Networked Systems, University of Klagenfurt, Austria.
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
// You can contact the author at <martin.scheiber@aau.at>

#ifndef UAV_INIT_TYPES_UWB_ANCHOR_HPP_
#define UAV_INIT_TYPES_UWB_ANCHOR_HPP_

#include <Eigen/Eigen>

namespace UavInit
{
///
/// \brief The UwbAnchor struct is an object describing an UWB anchor.
///
struct UwbAnchor
{
  uint id{ 0 };                                        //!< id of anchor
  Eigen::Vector3d p_AinG{ Eigen::VectorXd::Zero(3) };  //!< position of anchor in 'global' frame
  double bias_d{ 0.0 };                                //!< beta = (1+alpha) distance bias
  double bias_c{ 0.0 };                                //!< constant measurement bias

  Eigen::Matrix3d cov_p_AinG{ Eigen::MatrixXd::Zero(3, 3) };  //!< covariance of position
  double cov_bias_d{ 0.0 };                                   //!< covariance of distance bias
  double cov_bias_c{ 0.0 };                                   //!< covariance of constant bias

  bool initialized{ false };  //!< flag to determine if anchor is initialized

  UwbAnchor(){};
  UwbAnchor(uint _id) : id(_id){};

};  // struct UwbAnchor
}  // namespace UavInit

#endif  // UAV_INIT_TYPES_UWB_ANCHOR_HPP_
