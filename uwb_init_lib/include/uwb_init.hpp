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
// <alessandro.fornasier@aau.at> <giulio.delama@aau.at>

#ifndef UAV_INIT_UWB_INIT_HPP_
#define UAV_INIT_UWB_INIT_HPP_

#include <Eigen/Dense>
#include <deque>
#include <map>
#include <chrono>

#include "options/uwb_init_options.hpp"
#include "types/types.hpp"
#include "utils/logger.hpp"

namespace UavInit
{
///
/// \brief The UwbInitializer class is the main object used for UWB data handling
/// for the purpose of initializing the UWB anchors.
///
class UwbInitializer
{

public:
    ///
    /// \brief UwbInitializer default constructor
    /// \param params parameter/options used for UWB initialization
    ///
    UwbInitializer(UwbInitOptions& params, const LoggerLevel& level = LoggerLevel::FULL);

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
    /// \brief init_anchors tries to initialize all anchors for which readings exist
    /// \param anchor_buffer
    /// \return true if all anchors were successfully initialized
    ///
    /// This function performs a least-squares initialization using the per anchor measurments given the measurement model
    /// from \cite Blueml2021. It will try to initialize each anchor (validated per ID) individually. If an anchor was already
    /// successfully initialized in the past it is skipped.
    /// It will also return 'true' if all anchors, for which measurements are present were successfully initialized at
    /// some point.
    ///
    bool init_anchors(UwbAnchorBuffer& anchor_buffer);

    ///
    /// \todo TODO (gid) planner for waypoint generation to refine anchors
    ///

    ///
    /// \brief refine_anchors refines the already initialized anchors via optimal waypoints for non-
    /// linear least squares optimization
    ///
    bool refine_anchors(UwbAnchorBuffer& anchor_buffer);

    // Shared pointer to logger
    std::shared_ptr<Logger> logger_ = nullptr;

private:

    // Initializer parameters
    UwbInitOptions params_;

    // Auxiliary variables for checks depending on parameters
    bool check_beta_sq = false;

    // Anchor and measurement handling
    PositionBufferTimed buffer_p_UinG_;     //!< buffer of UWB module positions in global frame

    UwbDataBuffer uwb_data_buffer_;     //!< history of uwb readings in DataBuffer

    // Least squares initialization handling
    std::function<bool(std::deque<std::pair<double, UwbData>>&, Eigen::MatrixXd&, Eigen::VectorXd&)> ls_problem_;

    // Least squares solver
    bool solve_ls(UwbAnchorBuffer& anchor_buffer, const uint& anchor_id);

    // Nonlinear Least Squares solver
    bool solve_nls(UwbAnchor& anchor, std::deque<std::pair<double, UwbData>>& uwb_data);

    ///
    /// \brief functions for least squares problem formulation depending on selected method and variables
    /// \param UWB data for the single anchor, coefficient matrix A, measurement vector b (A * x = b)
    /// \return ture if successful, false if not
    ///
    bool ls_single_full_bias(std::deque<std::pair<double, UwbData>>& uwb_data,
                             Eigen::MatrixXd& A, Eigen::VectorXd& b);
    bool ls_single_dist_bias(std::deque<std::pair<double, UwbData>>& uwb_data,
                             Eigen::MatrixXd& A, Eigen::VectorXd& b);
    bool ls_single_const_bias(std::deque<std::pair<double, UwbData>>& uwb_data,
                              Eigen::MatrixXd& A, Eigen::VectorXd& b);
    bool ls_single_no_bias(std::deque<std::pair<double, UwbData>>& uwb_data,
                           Eigen::MatrixXd& A, Eigen::VectorXd& b);
    bool ls_double_full_bias(std::deque<std::pair<double, UwbData>>& uwb_data,
                             Eigen::MatrixXd& A, Eigen::VectorXd& b);
    bool ls_double_dist_bias(std::deque<std::pair<double, UwbData>>& uwb_data,
                             Eigen::MatrixXd& A, Eigen::VectorXd& b);
    bool ls_double_const_bias(std::deque<std::pair<double, UwbData>>& uwb_data,
                              Eigen::MatrixXd& A, Eigen::VectorXd& b);
    bool ls_double_no_bias(std::deque<std::pair<double, UwbData>>& uwb_data,
                           Eigen::MatrixXd& A, Eigen::VectorXd& b);

};

}  // namespace UavInit

#endif  // UAV_INIT_UWB_INIT_HPP_
