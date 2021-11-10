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

#include <ros/ros.h>

#include <Eigen/Dense>
#include <deque>
#include <map>
#include <numeric>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "types/types.hpp"
#include "options/uwb_init_options.hpp"

namespace uav_init
{

class UwbInitializer
{
public:
  ///
  /// \brief UwbInitializer default constructor
  /// \param params parameter/options used for UWB initialization
  ///
  UwbInitializer(UwbInitOptions &params) : params_(params)
  {
    buffer_p_UinG_.init(params_.buffer_size_s);
    uwb_data_buffer_.init(params_.buffer_size_s);
  }

  ///
  /// \brief feed_uwb stores incoming UWB (valid) readings
  /// \param uwb_measurements UwbData vector of measurements
  ///
  void feed_uwb(const std::vector<UwbData> uwb_measurements);

  void feed_pose(const double timestamp, const Eigen::Vector3d p_UinG);

  /**
   * @brief Try to initialize anchors and measurement bias through LS
   *
   * @param vector of anchors position
   * @param measuremnt (distance dependent) bias
   */
  bool try_to_initialize_anchors(std::map<size_t, Eigen::Vector3d>& p_ANCHORSinG, double& distance_bias,
                                 double& const_bias, std::map<size_t, Eigen::Matrix3d>& Anchors_Covs,
                                 double& distance_bias_Cov, double& const_bias_Cov);
  bool try_to_initialize_anchors(UwbAnchorBuffer &anchor_buffer);

protected:
  UwbInitOptions params_;

  // anchor and measurement handeling
  uint n_anchors_;                 //!< number of anchors in use
//  double buffer_size_s_{ 100.0 };  //!< buffer size in s of UWB module positions
  Eigen::Vector3d cur_p_UinG_;     //!< current position of the UWB module in global frame
  //  PositionBuffer buffer_p_UinG_;   //!< buffer of UWB module positions in global frame
//  TimedBuffer<Eigen::Vector3d> buffer_p_UinG_;  //!< buffer of UWB module positions in global frame
  PositionBufferTimed buffer_p_UinG_;  //!< buffer of UWB module positions in global frame

  /// Our history of uwb readings [anchor, [p_UinG, timestamp, distance]]
  //  std::multimap<size_t, std::tuple<Eigen::Vector3d, double, double>> uwb_data;
//  std::map<uint16_t, std::vector<UwbData>> uwb_data_buffer_;
  UwbDataBuffer uwb_data_buffer_;

  /// Set of measurement associated with a specific anchor [p_UinG, timestamp, distance]
  //  std::vector<std::tuple<Eigen::Vector3d, double, double>> single_anchor_uwb_data;
};

}  // namespace uav_init

#endif  // UAV_INIT_UWB_INIT_HPP_
