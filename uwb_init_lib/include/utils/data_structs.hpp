// Copyright (C) 2022 Alessandro Fornasier, Giulio Delama, and Martin Scheiber,
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
// You can contact the authors at
// <alessandro.fornasier@aau.at>, <giulio.delama@aau.at>, and
// <martin.scheiber@aau.at>

#ifndef UWB_INIT_TYPES_HPP_
#define UWB_INIT_TYPES_HPP_

#include <Eigen/Eigen>
#include <cstdint>
#include <stdexcept>

#include "buffers/timedbuffer.hpp"
#include "utils/utils.hpp"

namespace uwb_init
{
/**
 * @brief A simple data structure that defines a UWB anchor
 */
struct UwbAnchor
{
  /// Id of the anchor
  uint id_;

  /// Position of the anchor (A) in the global frame of reference (G)
  Eigen::Vector3d p_AinG_;

  UwbAnchor(const uint& id, const Eigen::Vector3d& p_AinG) : id_(id), p_AinG_(p_AinG)
  {
  }
};

/**
 * @brief A simple data structure that defines the LS solution of the UWB anchors initialization problem
 */
struct LSSolution
{
  /// UWB anchor
  UwbAnchor anchor_;

  /// Gamma: Constant bias
  double gamma_;

  /// Covariance of the solution
  Eigen::MatrixXd cov_;

  LSSolution(const UwbAnchor& anchor, const double& gamma, const Eigen::MatrixXd& cov) : anchor_(anchor), gamma_(gamma)
  {
    if (!(cov.rows() == 4 && cov.cols() == 4 && isPD(cov)))
    {
      throw std::invalid_argument("UwbLSSolution: Invalid covariance");
    }
    cov_ = cov;
  }
};

/**
 * @brief A simple data structure that defines the NLS solution of the UWB anchors initialization problem
 */
struct NLSSolution
{
  /// UWB anchor
  UwbAnchor anchor_;

  /// Gamma: Constant bias
  double gamma_;

  /// Beta: Distance multiplier bias
  double beta_;

  /// Covariance of the solution
  Eigen::MatrixXd cov_;

  NLSSolution(const UwbAnchor& anchor, const double& gamma, const double& beta, const Eigen::MatrixXd& cov)
    : anchor_(anchor), gamma_(gamma), beta_(beta)
  {
    if (!(cov.rows() == 5 && cov.cols() == 5 && isPD(cov)))
    {
      throw std::invalid_argument("UwbLSSolution: Invalid covariance");
    }
    cov_ = cov;
  }
};

/**
 * @brief A simple data structure that defines a UWB range measurement
 *
 */
struct UwbData
{
  /// Validity flag, determines if distance is valid
  bool valid_;

  /// Distance measurement between anchor and tag in meters
  double distance_;

  /// Id of the anchor from which the measurement is received
  uint id_;

  UwbData(const bool& valid, const double& distance, const uint& id) : valid_(valid), distance_(distance), id_(id)
  {
  }
};

typedef TimedBuffer<Eigen::Vector3d> PositionBuffer;
typedef std::unordered_map<uint, TimedBuffer<UwbData>> UwbDataBuffer;
typedef std::unordered_map<uint, LSSolution> LSSolutions;
typedef std::unordered_map<uint, NLSSolution> NLSSolutions;

}  // namespace uwb_init

#endif  // UWB_INIT_TYPES_HPP_
