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

#ifndef DATA_STRUCTURES_HPP_
#define DATA_STRUCTURES_HPP_

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

  std::string str() const
  {
    std::string s("UwbAnchor [" + std::to_string(id_) + "]: " + std::to_string(p_AinG_(0)) +
                  ", " + std::to_string(p_AinG_(1)) + ", " + std::to_string(p_AinG_(2)));
    return s;
  }
};

/**
 * @brief A simple data structure that defines the LS solution of the UWB anchors initialization problem
 */
struct LSSolution
{
  /// UWB anchor
  UwbAnchor anchor_;

  /// Gamma: Constant bias: map<id_Tag, gamma>
  std::unordered_map<uint, double> gammas_;

  /// Covariance of the solution
  Eigen::MatrixXd cov_;

  LSSolution(const UwbAnchor& anchor, const double& gamma, const Eigen::MatrixXd& cov, const uint id_tag = 0)
    : anchor_(anchor) {
    // Check if cov is either 3x3 or 4x4 and if it is semi positive definite

     gammas_.insert({id_tag, gamma});

    if (!(cov.rows() == cov.cols() && cov.rows() <= 5 && isSPD(cov)))
    {
      throw std::invalid_argument("LSSolution: Invalid covariance");
    }
    cov_ = cov;
  }

  LSSolution(const UwbAnchor& anchor, const std::unordered_map<uint, double>& gammas, const Eigen::MatrixXd& cov)
    : anchor_(anchor), gammas_(gammas) {


    long dim = 3+2*gammas_.size();
    // Check if cov is 5x5 and if it is semi positive definite
    if (!(cov.rows() <= dim && cov.cols() <= dim && isSPD(cov)))
    {
      throw std::invalid_argument("NLSSolution: Invalid covariance");
    }
    cov_ = cov;
  }

  std::string str() const
  {
    std::stringstream ss;
    ss << "LSSolution: " << anchor_.str() << "; const biases:";
    for(auto const& e: gammas_) {
      ss << "[" << e.first << "]=" << e.second << ", ";
    }
    ss << " Cov=[" << cov_.diagonal().transpose() << "]";
    return ss.str();
  }
};

/**
 * @brief A simple data structure that defines the NLS solution of the UWB anchors initialization problem
 */
struct NLSSolution
{
  /// UWB anchor
  UwbAnchor anchor_;

  /// Gamma: Constant bias: map<id_Tag, gamma>
  std::unordered_map<uint, double> gammas_;

  /// Beta: Distance multiplier bias: map<id_Tag, beta>
  std::unordered_map<uint, double> betas_;

  /// Covariance of the solution
  Eigen::MatrixXd cov_;

  NLSSolution(const UwbAnchor& anchor, const double& gamma, const double& beta, const Eigen::MatrixXd& cov, const uint id_tag = 0)
    : anchor_(anchor) {

    gammas_.insert({id_tag, gamma});
    betas_.insert({id_tag, beta});
    // Check if cov is 5x5 and if it is semi positive definite
    if (!(cov.rows() == 5 && cov.cols() == 5 && isSPD(cov)))
    {
      throw std::invalid_argument("NLSSolution: Invalid covariance");
    }
    cov_ = cov;
  }

  NLSSolution(const UwbAnchor& anchor, const std::unordered_map<uint, double>& gammas, std::unordered_map<uint, double>& betas, const Eigen::MatrixXd& cov)
    : anchor_(anchor), gammas_(gammas), betas_(betas) {

    if (gammas_.size() != betas_.size()) {
      throw std::invalid_argument("NLSSolution: Invalid gammas and betas dimension!");
    }

    long dim = 3+2*gammas_.size();
    // Check if cov is 5x5 and if it is semi positive definite
    if (!(cov.rows() == dim && cov.cols() == dim && isSPD(cov)))
    {
      throw std::invalid_argument("NLSSolution: Invalid covariance");
    }
    cov_ = cov;
  }
  std::string str() const
  {
    std::stringstream ss;
    ss << "NLSSolution: " << anchor_.str() << "; const biases:";
    for(auto const& e: gammas_) {
      ss << "[" << e.first << "]=" << e.second << ", ";
    }
    ss << "; range biases:";
    for(auto const& e: betas_) {
      ss << "[" << e.first << "]=" << e.second << ", ";
    }
    ss << " Cov=[" << cov_.diagonal().transpose() << "]";
    return ss.str();
  }

  Eigen::VectorXd to_vec() const
  {
    Eigen::VectorXd v;
    v.setZero(3+gammas_.size()+betas_.size());

    Eigen::VectorXd gammas, betas;
    gammas.setZero(gammas_.size());
    betas.setOnes(betas_.size());

    size_t idx = 0;
    for(auto const& e : gammas_)
    {
      gammas(idx) = e.second;
    }
    idx = 0;
    for(auto const& e : betas_)
    {
      betas(idx) = e.second;
    }

    v << anchor_.p_AinG_.x(), anchor_.p_AinG_.y(), anchor_.p_AinG_.z(), gammas, betas;
    return v;
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
  uint id_Anchor;

  /// Id of the tag from which the measurement is received
  uint id_Tag = 0;

  UwbData(const bool& valid, const double& distance, const uint& id_anchor, const uint id_tag = 0) : valid_(valid), distance_(distance),
                                                                                                     id_Anchor(id_anchor), id_Tag(id_tag)
  {
  }
};

/**
 * @brief A simple data structure that defines a Waypoint with 2 digit precision
 *
 */
struct Waypoint
{
  double x_;
  double y_;
  double z_;

  Waypoint(const double& x, const double& y, const double& z)
    : x_(std::round(x * 100) / 100), y_(std::round(y * 100) / 100), z_(std::round(z * 100) / 100)
  {
  }

  Waypoint(const Eigen::Vector3d& wp)
    : x_(std::round(wp.x() * 100) / 100), y_(std::round(wp.y() * 100) / 100), z_(std::round(wp.z() * 100) / 100)
  {
  }
};
// Hist<p_TinG>
typedef TimedBuffer<Eigen::Vector3d> PositionBuffer;
// map<Tag_ID, Hist<p_TinG>>>
typedef std::map<uint, PositionBuffer> PositionBufferDict_t;
// map<Anchor_ID,  Hist<UwbData>>
typedef std::map<uint, TimedBuffer<UwbData>> UwbDataBuffer;
// map<Tag_ID,  Hist<UwbData>>
typedef std::map<uint, TimedBuffer<UwbData>> UwbDataPerTag;
// map<Anchor_ID, map<Tag_ID, Hist<UwbData>>>
typedef std::map<uint, UwbDataPerTag> UwbDataBufferDict_t;
// map<Anchor_ID,  LSSolution>
typedef std::map<uint, LSSolution> LSSolutions;
// map<Anchor_ID,  NLSSolution>
typedef std::map<uint, NLSSolution> NLSSolutions;
typedef std::vector<Waypoint> Waypoints;

}  // namespace uwb_init

#endif  // DATA_STRUCTURES_HPP_
