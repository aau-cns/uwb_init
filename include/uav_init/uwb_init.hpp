// Copyright (C) 2021 Martin Scheiber, Alessandro Fornasier
// Control of Networked Systems, Universitaet Klagenfurt, Austria
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
// You can contact the authors at <martin.scheiber@aau.at> and
// <alessandro.fornasier@aau.at>

#ifndef UAV_INIT_UWB_INIT_HPP_
#define UAV_INIT_UWB_INIT_HPP_

#include <ros/ros.h>

#include <Eigen/Dense>
#include <deque>
#include <map>

#include "options/uwb_init_options.hpp"
#include "types/types.hpp"

namespace uav_init
{
///
/// \brief The UwbInitializer class is an object used for UWB data handeling for the purpose of initializing the UWB
/// anchors.
///
class UwbInitializer
{

public:
  ///
  /// \brief UwbInitializer default constructor
  /// \param params parameter/options used for UWB initialization
  ///
  UwbInitializer(UwbInitOptions& params) : params_(params)
  {
    // initialize buffers
    buffer_p_UinG_.init(params_.buffer_size_s);
    uwb_data_buffer_.init(params_.buffer_size_s);

    switch (params_.init_variables)
    {
      case UwbInitOptions::InitVariables::FULL_BIAS: {
        if (params_.init_method == UwbInitOptions::InitMethod::SINGLE)
          fx_init_ = std::bind(&UwbInitializer::initialize_single_all, this, std::placeholders::_1,
                               std::placeholders::_2, std::placeholders::_3);
        else if (params_.init_method == UwbInitOptions::InitMethod::DOUBLE)
          fx_init_ = std::bind(&UwbInitializer::initialize_double_all, this, std::placeholders::_1,
                               std::placeholders::_2, std::placeholders::_3);
        else
        {
          INIT_ERROR_STREAM("No initialization routine for method-variable pair " << params_.init_method << "-"
                                                                                  << params_.init_variables);
          exit(EXIT_FAILURE);
        }
        break;
      }
      case UwbInitOptions::InitVariables::NO_BIAS: {
        if (params_.init_method == UwbInitOptions::InitMethod::SINGLE)
          fx_init_ = std::bind(&UwbInitializer::initialize_single_nobias, this, std::placeholders::_1,
                               std::placeholders::_2, std::placeholders::_3);
        else
        {
          INIT_ERROR_STREAM("No initialization routine for method-variable pair " << params_.init_method << "-"
                                                                                  << params_.init_variables);
          exit(EXIT_FAILURE);
        }
        break;
      }
      case UwbInitOptions::InitVariables::DIST_BIAS: {
        INIT_ERROR_STREAM("No initialization routine for method-variable pair " << params_.init_method << "-"
                                                                                << params_.init_variables);
        exit(EXIT_FAILURE);
        break;
      }
      case UwbInitOptions::InitVariables::CONST_BIAS: {
        INIT_ERROR_STREAM("No initialization routine for method-variable pair " << params_.init_method << "-"
                                                                                << params_.init_variables);
        exit(EXIT_FAILURE);
        break;
      }
    }
  }

  ///
  /// \brief reset resets the initializer and all its buffers
  ///
  void reset();

  ///
  /// \brief feed_uwb stores incoming UWB (valid) readings
  /// \param uwb_measurements UwbData vector of measurements
  ///
  /// \todo allow feeding of old(er) measurements
  ///
  void feed_uwb(const std::vector<UwbData> uwb_measurements);

  ///
  /// \brief feed_pose stores incoming positions of the UAV in the global frame
  /// \param timestamp timestamp of pose
  /// \param p_UinG position to add to buffer
  ///
  /// \todo allow feeding of old(er) measurements
  ///
  void feed_pose(const double timestamp, const Eigen::Vector3d p_UinG);

  ///
  /// \brief try_to_initialize_anchors tries to initialize all anchors for which readings exist
  /// \param anchor_buffer
  /// \return true if all anchors were successfully initialized
  ///
  /// This function performs a least-squares initialization using the per anchor measurments given the measurement model
  /// from \cite Blueml2021. It will try to initialize each anchor (validated per ID) individually. If an anchor was already
  /// successfully initialized in the past it is skipped.
  /// It will also return 'true' if all anchors, for which measurements are present were successfully initialized at
  /// some point.
  ///
  bool try_to_initialize_anchors(UwbAnchorBuffer& anchor_buffer);

private:
  UwbInitOptions params_;  //!< initializer parameters

  // anchor and measurement handling
  Eigen::Vector3d cur_p_UinG_;         //!< current position of the UWB module in global frame
  PositionBufferTimed buffer_p_UinG_;  //!< buffer of UWB module positions in global frame

  UwbDataBuffer uwb_data_buffer_;  //!< history of uwb readings in DataBuffer

  // init handling
  std::function<bool(UwbAnchorBuffer&, const uint&, const double&)> fx_init_;

//  // Debugging
//  std::vector<Eigen::Vector3d> p_AinG_gt_{ { Eigen::Vector3d(1.225, -1.459, 0.073),
//                                             Eigen::Vector3d(-0.989, 0.350, 0.082),
//                                             Eigen::Vector3d(-0.048, 2.058, 0.055) } };

  ///
  /// \brief initialize_single try to initialize all anchors using the single measurement formulation
  /// \param anchor_buffer
  /// \return true if all anchors were successfully initialized
  ///
  bool initialize_single_all(UwbAnchorBuffer& anchor_buffer, const uint& anchor_id, const double& calc_time);
  bool initialize_double_all(UwbAnchorBuffer& anchor_buffer, const uint& anchor_id, const double& calc_time);
  bool initialize_single_nobias(UwbAnchorBuffer& anchor_buffer, const uint& anchor_id, const double& calc_time);
};

}  // namespace uav_init

#endif  // UAV_INIT_UWB_INIT_HPP_
