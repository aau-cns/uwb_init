// Copyright (C) 2021 Martin Scheiber, Alessandro Fornasier
// Control of Networked Systems, Universitaet Klagenfurt, Austria
//
// You can contact the authors at <martin.scheiber@aau.at> and
// <alessandro.fornasier@aau.at>
//
// All rights reserved.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.

#ifndef UAV_INIT_UWB_INIT_HPP_
#define UAV_INIT_UWB_INIT_HPP_

#include <Eigen/Dense>
#include <map>
#include <numeric>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace uav_init
{
class UwbInitializer
{
public:
  /**
   * @brief Default constructor
   */
  UwbInitializer(int n_anchors) : _n_anchors(n_anchors)
  {
  }

  /**
   * @brief Stores incoming uwb (valid) readings
   *
   * @param timestamp Timestamp of uwb reading
   * @param valid uwb readings associated with anchor
   */
  void feed_uwb(double timestamp, std::map<size_t, double> uwb_ranges, Eigen::Vector3d p_UinG);

  /**
   * @brief Try to initialize anchors and measurement bias through LS
   *
   * @param vector of anchors position
   * @param measuremnt (distance dependent) bias
   */
  bool try_to_initialize_anchors(std::map<size_t, Eigen::Vector3d>& p_ANCHORSinG, double& distance_bias,
                                 double& const_bias, std::map<size_t, Eigen::Matrix3d>& Anchors_Covs,
                                 double& distance_bias_Cov, double& const_bias_Cov);

protected:
  /// number of anchors
  int _n_anchors;

  /// Our history of uwb readings [anchor, [p_UinG, timestamp, distance]]
  std::multimap<size_t, std::tuple<Eigen::Vector3d, double, double>> uwb_data;

  /// Set of measurement associated with a specific anchor [p_UinG, timestamp, distance]
  std::vector<std::tuple<Eigen::Vector3d, double, double>> single_anchor_uwb_data;
};

}  // namespace uav_init

#endif  // UAV_INIT_UWB_INIT_HPP_
