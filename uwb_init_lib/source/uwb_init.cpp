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

#include "uwb_init.hpp"

#include "utils/logger.hpp"

namespace UwbInit
{

void UwbInitializer::reset()
{
    // Reset buffers
    uwb_data_buffer_.reset();
    buffer_p_UinG_.reset();

    // Reset positions
    cur_p_UinG_ = Eigen::Vector3d::Zero();
}


void UwbInitializer::feed_uwb(const std::vector<UwbData> uwb_measurements)
{
    // Add measurements to data buffer
    for (uint i = 0; i < uwb_measurements.size(); ++i) {
        // Check validity and if measurement is actual bigger than 0.0 (otherwise another error happend)
        if (uwb_measurements[i].valid && uwb_measurements[i].distance > 0.0) {
            uwb_data_buffer_.push_back(uwb_measurements[i].id, uwb_measurements[i].timestamp, uwb_measurements[i]);
        }
        else {
            // TODO (gid)
            //      INIT_DEBUG_STREAM("DISCARDING measurment " << uwb_measurements[i].distance << " from anchor "
            //                                                 << uwb_measurements[i].id);
        }
    }
}


void UwbInitializer::feed_pose(const double timestamp, const Eigen::Vector3d p_UinG)
{
    /// \todo TODO(scm): maybe use buffer here for timesyncing with UWB modules
    /// Currently this method does not take delayed UWB measurements into account
    buffer_p_UinG_.push_back(timestamp, p_UinG);
}


bool UwbInitializer::init_anchors(UwbAnchorBuffer& anchor_buffer)
{
    // Check if pose buffer is empty
    if ( buffer_p_UinG_.is_emtpy() ) {
        // TODO (gid) logging
        return false;
    }

    // Check if uwb buffer is empty
    if ( uwb_data_buffer_.is_emtpy() ) {
        // TODO (gid) logging
        return false;
    }

    // Variable for keeping track if all anchors has been correctly initialized
    bool init_successful = true;

    /// \todo TODO(scm): this can be improved by making DataBuffer a iterable class
    for (const auto& kv : uwb_data_buffer_.get_buffer()) {
        // Get ID of anchor and its data
        const auto anchor_id = kv.first;

        // Check if anchor is already initialized
        if (!params_.b_do_continous_init && anchor_buffer.contains_id(anchor_id) &&
                anchor_buffer.is_initialized(anchor_id))
        {
            continue;
        }
        // Solve ls problem and initialize anchor
        else if (solve_ls(anchor_buffer, anchor_id))
        {
            // TODO (gid) nonlinear optimization if option enabled
            continue;
        }
        else
        {
            init_successful = false;
        }
    }

    // TODO (gid) logging initialization successful
    return init_successful;
}


bool UwbInitializer::solve_ls(UwbAnchorBuffer& anchor_buffer, const uint& anchor_id)
{
    // starting time for elapsed time calculation
    auto start_t = std::chrono::steady_clock::now();

    // define result
    UwbAnchor new_uwb_anchor(anchor_id);

    // Define uwb data
    std::deque<std::pair<double, UwbData>> single_anchor_uwb_data;

    // First check
    if ( !(uwb_data_buffer_.get_buffer_values(anchor_id, single_anchor_uwb_data)) ) {
        return false;
    }

    // Coefficient matrix and measurement vector initialization
    Eigen::MatrixXd coeffs;
    Eigen::VectorXd meas;

    // Initialize least squares problem
    if ( !(ls_problem_(single_anchor_uwb_data, coeffs, meas)) ) {
        new_uwb_anchor.initialized = false;
        return false;
    }

    // Check the coefficient matrix condition number and solve the LS problem
    Eigen::BDCSVD<Eigen::MatrixXd> svd(coeffs, Eigen::ComputeThinU | Eigen::ComputeThinV);
    double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size()-1);

    // Second check (condition number) TODO (gid) add logger
    if ( !(cond < params_.max_cond_num) ) {
        new_uwb_anchor.initialized = false;
        return false;
    }

    // Solve LS problem
    Eigen::VectorXd lsSolution = svd.solve(meas);

    // lsSolution changes w.r.t. chosen method and variables!
    //
    // Full bias single:
    // [      0     ,       1     ,       2     ,  3 , 4,          5              ]
    // [b^2*p_AinG_x, b^2*p_AinG_y, b^2*p_AinG_z, b^2, k, b^2*(norm(p_AinG)^2-k^2)]
    //
    // Full bias double:
    // [      0     ,       1     ,       2     ,  3 , 4]
    // [b^2*p_AinG_x, b^2*p_AinG_y, b^2*p_AinG_z, b^2, k]
    //
    // Dist bias single:
    // [      0     ,       1     ,       2     ,  3 ,         4         ]
    // [b^2*p_AinG_x, b^2*p_AinG_y, b^2*p_AinG_z, b^2, b^2*norm(p_AinG)^2]
    //
    // Dist bias double:
    // [      0     ,       1     ,       2     ,  3 ]
    // [b^2*p_AinG_x, b^2*p_AinG_y, b^2*p_AinG_z, b^2]
    //
    // Const bias single:
    // [    0   ,     1   ,    2    ,  3 ,      4            ]
    // [p_AinG_x, p_AinG_y, p_AinG_z,  k , norm(p_AinG)^2-k^2]
    //
    // Const bias double:
    // [    0   ,     1   ,    2    ,  3 ]
    // [p_AinG_x, p_AinG_y, p_AinG_z,  k ]
    //
    // Unbiased single:
    // [    0    ,    1   ,    2   ,        3       ]
    // [p_AinG_x, p_AinG_y, p_AinG_z, norm(p_AinG)^2]
    //
    // Unbiased double:
    // [    0    ,    1   ,    2    ]
    // [p_AinG_x, p_AinG_y, p_AinG_z]
    //

    // Solution check (if exists beta squared must be positive)
    if ( check_beta_sq && !(lsSolution[3] > 0) ) {
        // TODO (gid) logging
        new_uwb_anchor.initialized = false;
        return false;
    }

    // Assign values to parameters
    Eigen::Vector3d p_AinG = lsSolution.segment(0, 3);
    double const_bias = 0.0;
    double distance_bias = 1.0;

    if ( check_beta_sq ) {
        p_AinG /= lsSolution[3];
        distance_bias = std::sqrt(lsSolution[3]);
        if ( lsSolution.size() > 4 ) {
            const_bias = lsSolution[4];
        }
    }
    else if ( lsSolution.size() > 3 ) {
        const_bias = lsSolution[3];
    }

    // Assign parameters to anchor
    new_uwb_anchor.p_AinG = p_AinG;
    new_uwb_anchor.bias_d = distance_bias;
    new_uwb_anchor.bias_c = const_bias;

    // Compute estimation Covariance
    Eigen::MatrixXd Cov =
            (std::pow((coeffs * lsSolution - meas).norm(), 2) / (coeffs.rows() * coeffs.cols())) *
            (svd.matrixV().inverse().transpose() * svd.singularValues().asDiagonal().inverse() *
             svd.singularValues().asDiagonal().inverse() * svd.matrixV().inverse());

    //    // Compare norm of singular values vector of covarnace matrix with threshold
    //    Eigen::JacobiSVD<Eigen::MatrixXd> svd_cov(Cov, Eigen::ComputeThinU | Eigen::ComputeThinV);

    //    // Check if (cond < params_.max_cond_num)
    //    if ( !(svd_cov.singularValues().norm() <= params_.cov_sv_threshold) ) {
    //        new_uwb_anchor.initialized = false;
    //        return false;
    //    }

    // Initialize anchor Covariance
    new_uwb_anchor.cov_p_AinG = Cov.block(0, 0, 3, 3);
    new_uwb_anchor.cov_bias_d = 0.0;
    new_uwb_anchor.cov_bias_c = 0.0;

    // Retrieve Covariance applying error propagation law if distant bias exixts
    if ( check_beta_sq ) {
        // Retrive P_AinG Covariance applying error propagation law and assign to Anchors_Covs
        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(3, 4);
        J.block(0, 0, 3, 3) = (1.0 / std::pow(distance_bias, 2)) * Eigen::Matrix3d::Identity();
        J.block(0, 3, 3, 1) = -p_AinG / std::pow(distance_bias, 2);
        new_uwb_anchor.cov_p_AinG = J * Cov.block(0, 0, 4, 4) * J.transpose();

        // Retrive Covariance of b and k applying error propagation law
        new_uwb_anchor.cov_bias_d = 1.0 / (4.0 * std::pow(distance_bias, 2)) * Cov(3, 3);
        if ( lsSolution.size() > 4 ) {
            new_uwb_anchor.cov_bias_c = Cov(4, 4);
        }
    }
    else if ( lsSolution.size() > 3 ) {
        new_uwb_anchor.cov_bias_c = Cov(3, 3);
    }

    // Set initialization to true
    new_uwb_anchor.initialized = true;

    // Elapsed time for initialization
    auto end_t = std::chrono::steady_clock::now();
    double calc_time = std::chrono::duration_cast<std::chrono::seconds>(end_t - start_t).count();

    // Regardless, add calculation to buffer
    anchor_buffer.push_back(anchor_id, calc_time, new_uwb_anchor);

    return true;
}


bool UwbInitializer::ls_single_full_bias(std::deque<std::pair<double, UwbData>> &uwb_data,
                                         Eigen::MatrixXd &A, Eigen::VectorXd &b)
{
    // Full bias single:
    // [      0     ,       1     ,       2     ,  3 , 4,          5              ]
    // [b^2*p_AinG_x, b^2*p_AinG_y, b^2*p_AinG_z, b^2, k, b^2*(norm(p_AinG)^2-k^2)]

    // Coefficient matrix and measurement vector initialization
    A = Eigen::MatrixXd::Zero(uwb_data.size(), 6);
    b = Eigen::VectorXd::Zero(uwb_data.size());

    // Fill the coefficient matrix and the measurement vector
    for (uint i = 0; i < uwb_data.size(); ++i) {
        // Get closest UWB module position
        Eigen::Vector3d closest_p_UinG = buffer_p_UinG_.get_closest(uwb_data.at(i).first - params_.t_pose_diff);
        Eigen::VectorXd row(A.cols());
        row << -2 * closest_p_UinG.x(),
                -2 * closest_p_UinG.y(),
                -2 * closest_p_UinG.z(),
                std::pow(closest_p_UinG.norm(), 2),
                2 * uwb_data.at(i).second.distance,
                1;
        A.row(i) = row.transpose();
        b(i) = std::pow(uwb_data.at(i).second.distance, 2);
    }

    // Exit if regularization is not requested
    if ( !(params_.b_perform_regularization) ) return true;

    // Check to have more than 6 rows in the coefficient matrix
    if ( !(A.rows() > A.cols()) ) {
        // TODO (gid) logger: cannot perform regularzation
        return true;
    }

    // Check lambda positive
    if ( !(params_.lamda > 0.0) ) {
        // TODO (gid) logger: cannot perform regularzation (negative lambda)
        return true;
    }

    // Initialize regularization matrix and vector
    Eigen::MatrixXd A_reg = Eigen::MatrixXd::Zero(A.cols(), A.cols());
    Eigen::VectorXd b_reg = Eigen::VectorXd::Zero(A.cols());

    // Regularize z coordinate if requested
    if (params_.b_regularize_z) {
        A_reg(2, 2) = std::sqrt(params_.lamda);
    }

    // Regularize beta (distance bias) if requested
    if (params_.b_regularize_b) {
        A_reg(3, 3) = std::sqrt(params_.lamda);
        b_reg(3) = std::sqrt(params_.lamda);
    }

    // Regularize k (constant bias) if requested
    if (params_.b_regularize_k) {
        A_reg(4, 4) = std::sqrt(params_.lamda);
    }

    // Concatenate coeffs matrix A
    Eigen::MatrixXd A_new(A.rows()+A_reg.rows(), A.cols());
    A_new << A, A_reg;

    // Concatenate meas vector b
    Eigen::VectorXd b_new(b.size()+b_reg.size());
    b_new << b, b_reg;

    // Overwrite A and b
    A = A_new;
    b = b_new;

    return true;
}


bool UwbInitializer::ls_single_dist_bias(std::deque<std::pair<double, UwbData> > &uwb_data,
                                         Eigen::MatrixXd &A, Eigen::VectorXd &b)
{
    // Dist bias single:
    // [      0     ,       1     ,       2     ,  3 ,         4         ]
    // [b^2*p_AinG_x, b^2*p_AinG_y, b^2*p_AinG_z, b^2, b^2*norm(p_AinG)^2]

    // Coefficient matrix and measurement vector initialization
    A = Eigen::MatrixXd::Zero(uwb_data.size(), 5);
    b = Eigen::VectorXd::Zero(uwb_data.size());

    // Fill the coefficient matrix and the measurement vector
    for (uint i = 0; i < uwb_data.size(); ++i) {
        // Get closest UWB module position
        Eigen::Vector3d closest_p_UinG = buffer_p_UinG_.get_closest(uwb_data.at(i).first - params_.t_pose_diff);
        Eigen::VectorXd row(A.cols());
        row << -2 * closest_p_UinG.x(),
                -2 * closest_p_UinG.y(),
                -2 * closest_p_UinG.z(),
                std::pow(closest_p_UinG.norm(), 2),
                1;
        A.row(i) = row.transpose();
        b(i) = std::pow(uwb_data.at(i).second.distance, 2);
    }

    // Exit if regularization is not requested
    if ( !(params_.b_perform_regularization) ) return true;

    // Check to have more than 5 rows in the coefficient matrix
    if ( !(A.rows() > A.cols()) ) {
        // TODO (gid) logger: cannot perform regularzation
        return true;
    }

    // Check lambda positive
    if ( !(params_.lamda > 0.0) ) {
        // TODO (gid) logger: cannot perform regularzation (negative lambda)
        return true;
    }

    // Initialize regularization matrix and vector
    Eigen::MatrixXd A_reg = Eigen::MatrixXd::Zero(A.cols(), A.cols());
    Eigen::VectorXd b_reg = Eigen::VectorXd::Zero(A.cols());

    // Regularize z coordinate if requested
    if (params_.b_regularize_z) {
        A_reg(2, 2) = std::sqrt(params_.lamda);
    }

    // Regularize beta (distance bias) if requested
    if (params_.b_regularize_b) {
        A_reg(3, 3) = std::sqrt(params_.lamda);
        b_reg(3) = std::sqrt(params_.lamda);
    }

    // Concatenate coeffs matrix A
    Eigen::MatrixXd A_new(A.rows()+A_reg.rows(), A.cols());
    A_new << A, A_reg;

    // Concatenate meas vector b
    Eigen::VectorXd b_new(b.size()+b_reg.size());
    b_new << b, b_reg;

    // Overwrite A and b
    A = A_new;
    b = b_new;

    return true;
}


bool UwbInitializer::ls_single_const_bias(std::deque<std::pair<double, UwbData> > &uwb_data,
                                          Eigen::MatrixXd &A, Eigen::VectorXd &b)
{
    // Const bias single:
    // [    0   ,     1   ,    2    ,  3 ,      4            ]
    // [p_AinG_x, p_AinG_y, p_AinG_z,  k , norm(p_AinG)^2-k^2]

    // Coefficient matrix and measurement vector initialization
    A = Eigen::MatrixXd::Zero(uwb_data.size(), 5);
    b = Eigen::VectorXd::Zero(uwb_data.size());

    // Fill the coefficient matrix and the measurement vector
    for (uint i = 0; i < uwb_data.size(); ++i) {
        // Get closest UWB module position
        Eigen::Vector3d closest_p_UinG = buffer_p_UinG_.get_closest(uwb_data.at(i).first - params_.t_pose_diff);
        Eigen::VectorXd row(A.cols());
        row << -2 * closest_p_UinG.x(),
                -2 * closest_p_UinG.y(),
                -2 * closest_p_UinG.z(),
                2 * uwb_data.at(i).second.distance,
                1;
        A.row(i) = row.transpose();
        b(i) = std::pow(uwb_data.at(i).second.distance, 2);
    }

    // Exit if regularization is not requested
    if ( !(params_.b_perform_regularization) ) return true;

    // Check to have more than 6 rows in the coefficient matrix
    if ( !(A.rows() > A.cols()) ) {
        // TODO (gid) logger: cannot perform regularzation
        return true;
    }

    // Check lambda positive
    if ( !(params_.lamda > 0.0) ) {
        // TODO (gid) logger: cannot perform regularzation (negative lambda)
        return true;
    }

    // Initialize regularization matrix and vector
    Eigen::MatrixXd A_reg = Eigen::MatrixXd::Zero(A.cols(), A.cols());
    Eigen::VectorXd b_reg = Eigen::VectorXd::Zero(A.cols());

    // Regularize z coordinate if requested
    if (params_.b_regularize_z) {
        A_reg(2, 2) = std::sqrt(params_.lamda);
    }

    // Regularize k (constant bias) if requested
    if (params_.b_regularize_k) {
        A_reg(3, 3) = std::sqrt(params_.lamda);
    }

    // Concatenate coeffs matrix A
    Eigen::MatrixXd A_new(A.rows()+A_reg.rows(), A.cols());
    A_new << A, A_reg;

    // Concatenate meas vector b
    Eigen::VectorXd b_new(b.size()+b_reg.size());
    b_new << b, b_reg;

    // Overwrite A and b
    A = A_new;
    b = b_new;

    return true;
}


bool UwbInitializer::ls_single_no_bias(std::deque<std::pair<double, UwbData>> &uwb_data,
                                       Eigen::MatrixXd &A, Eigen::VectorXd &b)
{
    // Unbiased single:
    // [    0    ,    1   ,    2   ,        3       ]
    // [p_AinG_x, p_AinG_y, p_AinG_z, norm(p_AinG)^2]

    // Coefficient matrix and measurement vector initialization
    A = Eigen::MatrixXd::Zero(uwb_data.size(), 4);
    b = Eigen::VectorXd::Zero(uwb_data.size());

    // Fill the coefficient matrix and the measurement vector
    for (uint i = 0; i < uwb_data.size(); ++i) {
        // Get closest UWB module position
        Eigen::Vector3d closest_p_UinG = buffer_p_UinG_.get_closest(uwb_data.at(i).first - params_.t_pose_diff);
        Eigen::VectorXd row(4);
        row << -2 * closest_p_UinG.x(),
                -2 * closest_p_UinG.y(),
                -2 * closest_p_UinG.z(),
                1;
        A.row(i) = row.transpose();
        b(i) = std::pow(uwb_data.at(i).second.distance, 2) - std::pow(closest_p_UinG.norm(), 2);
    }

    // Exit if regularization is not requested
    if ( !(params_.b_perform_regularization) ) return true;

    // Check to have more than 4 rows in the coefficient matrix
    if ( !(A.rows() > 4) ) {
        // TODO (gid) logger: cannot perform regularzation
        return true;
    }

    // Check lambda positive
    if ( !(params_.lamda > 0.0) ) {
        // TODO (gid) logger: cannot perform regularzation (negative lambda)
        return true;
    }

    // Initialize regularization matrix and vector
    Eigen::MatrixXd A_reg = Eigen::MatrixXd::Zero(A.cols(), A.cols());
    Eigen::VectorXd b_reg = Eigen::VectorXd::Zero(A.cols());

    // Regularize z coordinate if requested
    if (params_.b_regularize_z) {
        A_reg(2, 2) = std::sqrt(params_.lamda);
    }

    // Concatenate coeffs matrix A
    Eigen::MatrixXd A_new(A.rows()+A_reg.rows(), A.cols());
    A_new << A, A_reg;

    // Concatenate meas vector b
    Eigen::VectorXd b_new(b.size()+b_reg.size());
    b_new << b, b_reg;

    // Overwrite A and b
    A = A_new;
    b = b_new;

    return true;
}


bool UwbInitializer::ls_double_full_bias(std::deque<std::pair<double, UwbData>> &uwb_data,
                                         Eigen::MatrixXd &A, Eigen::VectorXd &b)
{
    // Full bias double:
    // [      0     ,       1     ,       2     ,  3 , 4]
    // [b^2*p_AinG_x, b^2*p_AinG_y, b^2*p_AinG_z, b^2, k]

    // Number of parameters for selected model and method
    const uint8_t n_params = 5;

    // Coefficient matrix and measurement vector initialization
    std::vector<double> coeffs_vec;
    std::vector<double> meas_vec;

    // Fill the coefficient matrix and the measurement vector
    for (uint i = 0; i < uwb_data.size() - params_.meas_baseline_idx_; ++i) {
        // baseline check
        double diff = uwb_data.at(i).second.distance -
                uwb_data.at(i + params_.meas_baseline_idx_).second.distance;
        if ( !(std::abs(diff) > params_.meas_baseline_m_) ) continue;

        // get closest UWB module position and check if closes was actually reached
        Eigen::Vector3d closest_p_UinG1, closest_p_UinG2;
        if (buffer_p_UinG_.get_closest(uwb_data.at(i).first - params_.t_pose_diff, closest_p_UinG1) &&
                buffer_p_UinG_.get_closest(uwb_data.at(i + params_.meas_baseline_idx_).first - params_.t_pose_diff, closest_p_UinG2))
        {
            Eigen::VectorXd row(n_params);
            row << -2 * (closest_p_UinG1.x() - closest_p_UinG2.x()),
                    -2 * (closest_p_UinG1.y() - closest_p_UinG2.y()),
                    -2 * (closest_p_UinG1.z() - closest_p_UinG2.z()),
                    std::pow(closest_p_UinG1.norm(), 2) - std::pow(closest_p_UinG2.norm(), 2),
                    2 * diff;

            coeffs_vec.push_back(row(0));
            coeffs_vec.push_back(row(1));
            coeffs_vec.push_back(row(2));
            coeffs_vec.push_back(row(3));
            coeffs_vec.push_back(row(4));
            meas_vec.push_back(std::pow(uwb_data.at(i).second.distance, 2) -
                               std::pow(uwb_data.at(i + params_.meas_baseline_idx_).second.distance, 2));

        }
    }

    // Assertation, check vectors size
    assert(coeffs_vec.size() == n_params * meas_vec.size());

    // Map vectors to Eigen matrices
    A = Eigen::MatrixXd(Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
                            coeffs_vec.data(), coeffs_vec.size() / n_params, n_params));
    b = Eigen::VectorXd(
                Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>(meas_vec.data(), meas_vec.size(), 1));

    // Exit if regularization is not requested
    if ( !(params_.b_perform_regularization) ) return true;

    // Check to have more than 5 rows in the coefficient matrix
    if ( !(coeffs_vec.size() > n_params * n_params) ) {
        // TODO (gid) logger: cannot perform regularzation
        return true;
    }

    // Check lambda positive
    if ( !(params_.lamda > 0.0) ) {
        // TODO (gid) logger: cannot perform regularzation (negative lambda)
        return true;
    }

    // Data augmentation for regularization (5x5 matrix for coeffs, 5x1 vector for meas)
    // Add 10 zero to fill first two rows of coeffs_vec (x and y coordinates)
    for (uint cnt_line = 0; cnt_line < 2 * n_params; ++cnt_line)
        coeffs_vec.push_back(0.0);

    // Regularize z coordinate if requested
    if (params_.b_regularize_z) {
        coeffs_vec.push_back(0.0);                          // b^2*p_AinG_x
        coeffs_vec.push_back(0.0);                          // b^2*p_AinG_y
        coeffs_vec.push_back(std::sqrt(params_.lamda));     // b^2*p_AinG_z
        coeffs_vec.push_back(0.0);                          // b^2
        coeffs_vec.push_back(0.0);                          // k
    }
    else {
        for (uint cnt_line = 0; cnt_line < n_params; ++cnt_line)
            coeffs_vec.push_back(0.0);
    }

    // Regularize beta (distance bias) if requested
    if (params_.b_regularize_b) {
        coeffs_vec.push_back(0.0);                          // b^2*p_AinG_x
        coeffs_vec.push_back(0.0);                          // b^2*p_AinG_y
        coeffs_vec.push_back(0.0);                          // b^2*p_AinG_z
        coeffs_vec.push_back(std::sqrt(params_.lamda));     // b^2
        coeffs_vec.push_back(0.0);                          // k
    }
    else {
        for (uint cnt_line = 0; cnt_line < n_params; ++cnt_line)
            coeffs_vec.push_back(0.0);
    }

    // Regularize k (constant bias) if requested
    if (params_.b_regularize_k) {
        coeffs_vec.push_back(0.0);                          // b^2*p_AinG_x
        coeffs_vec.push_back(0.0);                          // b^2*p_AinG_y
        coeffs_vec.push_back(0.0);                          // b^2*p_AinG_z
        coeffs_vec.push_back(0.0);                          // b^2
        coeffs_vec.push_back(std::sqrt(params_.lamda));     // k
    }
    else {
        for (uint cnt_line = 0; cnt_line < n_params; ++cnt_line)
            coeffs_vec.push_back(0.0);
    }

    // Add values to meas_vec (only if beta is regularized)
    if (params_.b_regularize_b) {
        meas_vec.push_back(0.0);                          // b^2*p_AinG_x
        meas_vec.push_back(0.0);                          // b^2*p_AinG_y
        meas_vec.push_back(0.0);                          // b^2*p_AinG_z
        meas_vec.push_back(std::sqrt(params_.lamda));     // b^2
        meas_vec.push_back(0.0);                          // k
    }
    else {
        for (uint cnt_line = 0; cnt_line < n_params; ++cnt_line)
            meas_vec.push_back(0.0);
    }


    // Map vectors to Eigen matrices
    A = Eigen::MatrixXd(Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
                            coeffs_vec.data(), coeffs_vec.size() / n_params, n_params));
    b = Eigen::VectorXd(
                Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>(meas_vec.data(), meas_vec.size(), 1));

    return true;
}

bool UwbInitializer::ls_double_dist_bias(std::deque<std::pair<double, UwbData> > &uwb_data,
                                         Eigen::MatrixXd &A, Eigen::VectorXd &b)
{
    // Dist bias double:
    // [      0     ,       1     ,       2     ,  3 ]
    // [b^2*p_AinG_x, b^2*p_AinG_y, b^2*p_AinG_z, b^2]

    // Number of parameters for selected model and method
    const uint8_t n_params = 4;

    // Coefficient matrix and measurement vector initialization
    std::vector<double> coeffs_vec;
    std::vector<double> meas_vec;

    // Fill the coefficient matrix and the measurement vector
    for (uint i = 0; i < uwb_data.size() - params_.meas_baseline_idx_; ++i) {
        // baseline check
        double diff = uwb_data.at(i).second.distance -
                uwb_data.at(i + params_.meas_baseline_idx_).second.distance;
        if ( !(std::abs(diff) > params_.meas_baseline_m_) ) continue;

        // get closest UWB module position and check if closes was actually reached
        Eigen::Vector3d closest_p_UinG1, closest_p_UinG2;
        if (buffer_p_UinG_.get_closest(uwb_data.at(i).first - params_.t_pose_diff, closest_p_UinG1) &&
                buffer_p_UinG_.get_closest(uwb_data.at(i + params_.meas_baseline_idx_).first - params_.t_pose_diff, closest_p_UinG2))
        {
            Eigen::VectorXd row(n_params);
            row << -2 * (closest_p_UinG1.x() - closest_p_UinG2.x()),
                    -2 * (closest_p_UinG1.y() - closest_p_UinG2.y()),
                    -2 * (closest_p_UinG1.z() - closest_p_UinG2.z()),
                    std::pow(closest_p_UinG1.norm(), 2) - std::pow(closest_p_UinG2.norm(), 2);

            coeffs_vec.push_back(row(0));
            coeffs_vec.push_back(row(1));
            coeffs_vec.push_back(row(2));
            coeffs_vec.push_back(row(3));
            meas_vec.push_back(std::pow(uwb_data.at(i).second.distance, 2) -
                               std::pow(uwb_data.at(i + params_.meas_baseline_idx_).second.distance, 2));

        }
    }

    // Assertation, check vectors size
    assert(coeffs_vec.size() == n_params * meas_vec.size());

    // Map vectors to Eigen matrices
    A = Eigen::MatrixXd(Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
                            coeffs_vec.data(), coeffs_vec.size() / n_params, n_params));
    b = Eigen::VectorXd(
                Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>(meas_vec.data(), meas_vec.size(), 1));

    // Exit if regularization is not requested
    if ( !(params_.b_perform_regularization) ) return true;

    // Check to have more than 4 rows in the coefficient matrix
    if ( !(coeffs_vec.size() > n_params * n_params) ) {
        // TODO (gid) logger: cannot perform regularzation
        return true;
    }

    // Check lambda positive
    if ( !(params_.lamda > 0.0) ) {
        // TODO (gid) logger: cannot perform regularzation (negative lambda)
        return true;
    }

    // Data augmentation for regularization (4x4 matrix for coeffs, 4x1 vector for meas)
    // Add 8 zero to fill first two rows of coeffs_vec (x and y coordinates)
    for (uint cnt_line = 0; cnt_line < 2 * n_params; ++cnt_line)
        coeffs_vec.push_back(0.0);

    // Regularize z coordinate if requested
    if (params_.b_regularize_z) {
        coeffs_vec.push_back(0.0);                          // b^2*p_AinG_x
        coeffs_vec.push_back(0.0);                          // b^2*p_AinG_y
        coeffs_vec.push_back(std::sqrt(params_.lamda));     // b^2*p_AinG_z
        coeffs_vec.push_back(0.0);                          // b^2
    }
    else {
        for (uint cnt_line = 0; cnt_line < n_params; ++cnt_line)
            coeffs_vec.push_back(0.0);
    }

    // Regularize beta (distance bias) if requested
    if (params_.b_regularize_b) {
        coeffs_vec.push_back(0.0);                          // b^2*p_AinG_x
        coeffs_vec.push_back(0.0);                          // b^2*p_AinG_y
        coeffs_vec.push_back(0.0);                          // b^2*p_AinG_z
        coeffs_vec.push_back(std::sqrt(params_.lamda));     // b^2
    }
    else {
        for (uint cnt_line = 0; cnt_line < n_params; ++cnt_line)
            coeffs_vec.push_back(0.0);
    }

    // Add values to meas_vec (only if beta is regularized)
    if (params_.b_regularize_b) {
        meas_vec.push_back(0.0);                          // b^2*p_AinG_x
        meas_vec.push_back(0.0);                          // b^2*p_AinG_y
        meas_vec.push_back(0.0);                          // b^2*p_AinG_z
        meas_vec.push_back(std::sqrt(params_.lamda));     // b^2
    }
    else {
        for (uint cnt_line = 0; cnt_line < 5; ++cnt_line)
            meas_vec.push_back(0.0);
    }


    // Map vectors to Eigen matrices
    A = Eigen::MatrixXd(Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
                            coeffs_vec.data(), coeffs_vec.size() / n_params, n_params));
    b = Eigen::VectorXd(
                Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>(meas_vec.data(), meas_vec.size(), 1));

    return true;
}

bool UwbInitializer::ls_double_const_bias(std::deque<std::pair<double, UwbData> > &uwb_data,
                                          Eigen::MatrixXd &A, Eigen::VectorXd &b)
{
    // Const bias double:
    // [    0   ,     1   ,    2    ,  3 ]
    // [p_AinG_x, p_AinG_y, p_AinG_z,  k ]

    // Number of parameters for selected model and method
    const uint8_t n_params = 4;

    // Coefficient matrix and measurement vector initialization
    std::vector<double> coeffs_vec;
    std::vector<double> meas_vec;

    // Fill the coefficient matrix and the measurement vector
    for (uint i = 0; i < uwb_data.size() - params_.meas_baseline_idx_; ++i) {
        // baseline check
        double diff = uwb_data.at(i).second.distance -
                uwb_data.at(i + params_.meas_baseline_idx_).second.distance;
        if ( !(std::abs(diff) > params_.meas_baseline_m_) ) continue;

        // get closest UWB module position and check if closes was actually reached
        Eigen::Vector3d closest_p_UinG1, closest_p_UinG2;
        if (buffer_p_UinG_.get_closest(uwb_data.at(i).first - params_.t_pose_diff, closest_p_UinG1) &&
                buffer_p_UinG_.get_closest(uwb_data.at(i + params_.meas_baseline_idx_).first - params_.t_pose_diff, closest_p_UinG2))
        {
            Eigen::VectorXd row(n_params);
            row << -2 * (closest_p_UinG1.x() - closest_p_UinG2.x()),
                    -2 * (closest_p_UinG1.y() - closest_p_UinG2.y()),
                    -2 * (closest_p_UinG1.z() - closest_p_UinG2.z()),
                    2 * diff;

            coeffs_vec.push_back(row(0));
            coeffs_vec.push_back(row(1));
            coeffs_vec.push_back(row(2));
            coeffs_vec.push_back(row(3));
            meas_vec.push_back(std::pow(uwb_data.at(i).second.distance, 2) -
                               std::pow(uwb_data.at(i + params_.meas_baseline_idx_).second.distance, 2));

        }
    }

    // Assertation, check vectors size
    assert(coeffs_vec.size() == n_params * meas_vec.size());

    // Map vectors to Eigen matrices
    A = Eigen::MatrixXd(Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
                            coeffs_vec.data(), coeffs_vec.size() / n_params, n_params));
    b = Eigen::VectorXd(
                Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>(meas_vec.data(), meas_vec.size(), 1));

    // Exit if regularization is not requested
    if ( !(params_.b_perform_regularization) ) return true;

    // Check to have more than 4 rows in the coefficient matrix
    if ( !(coeffs_vec.size() > n_params * n_params) ) {
        // TODO (gid) logger: cannot perform regularzation
        return true;
    }

    // Check lambda positive
    if ( !(params_.lamda > 0.0) ) {
        // TODO (gid) logger: cannot perform regularzation (negative lambda)
        return true;
    }

    // Data augmentation for regularization (4x4 matrix for coeffs, 4x1 vector for meas)
    // Add 8 zero to fill first two rows of coeffs_vec (x and y coordinates)
    for (uint cnt_line = 0; cnt_line < 2 * n_params; ++cnt_line)
        coeffs_vec.push_back(0.0);

    // Regularize z coordinate if requested
    if (params_.b_regularize_z) {
        coeffs_vec.push_back(0.0);                          // b^2*p_AinG_x
        coeffs_vec.push_back(0.0);                          // b^2*p_AinG_y
        coeffs_vec.push_back(std::sqrt(params_.lamda));     // b^2*p_AinG_z
        coeffs_vec.push_back(0.0);                          // k
    }
    else {
        for (uint cnt_line = 0; cnt_line < n_params; ++cnt_line)
            coeffs_vec.push_back(0.0);
    }

    // Regularize k (constant bias) if requested
    if (params_.b_regularize_k) {
        coeffs_vec.push_back(0.0);                          // b^2*p_AinG_x
        coeffs_vec.push_back(0.0);                          // b^2*p_AinG_y
        coeffs_vec.push_back(0.0);                          // b^2*p_AinG_z
        coeffs_vec.push_back(std::sqrt(params_.lamda));     // k
    }
    else {
        for (uint cnt_line = 0; cnt_line < n_params; ++cnt_line)
            coeffs_vec.push_back(0.0);
    }

    // Add values to meas_vec (only if beta is regularized)
    if (params_.b_regularize_b) {
        meas_vec.push_back(0.0);                          // b^2*p_AinG_x
        meas_vec.push_back(0.0);                          // b^2*p_AinG_y
        meas_vec.push_back(0.0);                          // b^2*p_AinG_z
        meas_vec.push_back(0.0);                          // k
    }
    else {
        for (uint cnt_line = 0; cnt_line < n_params; ++cnt_line)
            meas_vec.push_back(0.0);
    }


    // Map vectors to Eigen matrices
    A = Eigen::MatrixXd(Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
                            coeffs_vec.data(), coeffs_vec.size() / n_params, n_params));
    b = Eigen::VectorXd(
                Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>(meas_vec.data(), meas_vec.size(), 1));

    return true;
}

bool UwbInitializer::ls_double_no_bias(std::deque<std::pair<double, UwbData> > &uwb_data,
                                       Eigen::MatrixXd &A, Eigen::VectorXd &b)
{
    // Unbiased double:
    // [    0    ,    1   ,    2    ]
    // [p_AinG_x, p_AinG_y, p_AinG_z]

    // Number of parameters for selected model and method
    const uint8_t n_params = 3;

    // Coefficient matrix and measurement vector initialization
    std::vector<double> coeffs_vec;
    std::vector<double> meas_vec;

    // Fill the coefficient matrix and the measurement vector
    for (uint i = 0; i < uwb_data.size() - params_.meas_baseline_idx_; ++i) {
        // baseline check
        double diff = uwb_data.at(i).second.distance -
                uwb_data.at(i + params_.meas_baseline_idx_).second.distance;
        if ( !(std::abs(diff) > params_.meas_baseline_m_) ) continue;

        // get closest UWB module position and check if closes was actually reached
        Eigen::Vector3d closest_p_UinG1, closest_p_UinG2;
        if (buffer_p_UinG_.get_closest(uwb_data.at(i).first - params_.t_pose_diff, closest_p_UinG1) &&
                buffer_p_UinG_.get_closest(uwb_data.at(i + params_.meas_baseline_idx_).first - params_.t_pose_diff, closest_p_UinG2))
        {
            Eigen::VectorXd row(n_params);
            row << -2 * (closest_p_UinG1.x() - closest_p_UinG2.x()),
                    -2 * (closest_p_UinG1.y() - closest_p_UinG2.y()),
                    -2 * (closest_p_UinG1.z() - closest_p_UinG2.z());

            coeffs_vec.push_back(row(0));
            coeffs_vec.push_back(row(1));
            coeffs_vec.push_back(row(2));
            meas_vec.push_back(std::pow(uwb_data.at(i).second.distance, 2) -
                               std::pow(uwb_data.at(i + params_.meas_baseline_idx_).second.distance, 2));

        }
    }

    // Assertation, check vectors size
    assert(coeffs_vec.size() == n_params * meas_vec.size());

    // Map vectors to Eigen matrices
    A = Eigen::MatrixXd(Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
                            coeffs_vec.data(), coeffs_vec.size() / n_params, n_params));
    b = Eigen::VectorXd(
                Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>(meas_vec.data(), meas_vec.size(), 1));

    // Exit if regularization is not requested
    if ( !(params_.b_perform_regularization) ) return true;

    // Check to have more than 3 rows in the coefficient matrix
    if ( !(coeffs_vec.size() > n_params * n_params) ) {
        // TODO (gid) logger: cannot perform regularzation
        return true;
    }

    // Check lambda positive
    if ( !(params_.lamda > 0.0) ) {
        // TODO (gid) logger: cannot perform regularzation (negative lambda)
        return true;
    }

    // Data augmentation for regularization (3x3 matrix for coeffs, 3x1 vector for meas)
    // Add 6 zero to fill first two rows of coeffs_vec (x and y coordinates)
    for (uint cnt_line = 0; cnt_line < 2 * n_params; ++cnt_line)
        coeffs_vec.push_back(0.0);

    // Regularize z coordinate if requested
    if (params_.b_regularize_z) {
        coeffs_vec.push_back(0.0);                          // b^2*p_AinG_x
        coeffs_vec.push_back(0.0);                          // b^2*p_AinG_y
        coeffs_vec.push_back(std::sqrt(params_.lamda));     // b^2*p_AinG_z
    }
    else {
        for (uint cnt_line = 0; cnt_line < n_params; ++cnt_line)
            coeffs_vec.push_back(0.0);
    }

    // Add values to meas_vec (only if beta is regularized)
    if (params_.b_regularize_b) {
        meas_vec.push_back(0.0);                          // b^2*p_AinG_x
        meas_vec.push_back(0.0);                          // b^2*p_AinG_y
        meas_vec.push_back(0.0);                          // b^2*p_AinG_z
    }
    else {
        for (uint cnt_line = 0; cnt_line < n_params; ++cnt_line)
            meas_vec.push_back(0.0);
    }


    // Map vectors to Eigen matrices
    A = Eigen::MatrixXd(Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
                            coeffs_vec.data(), coeffs_vec.size() / n_params, n_params));
    b = Eigen::VectorXd(
                Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>(meas_vec.data(), meas_vec.size(), 1));

    return true;
}

}  // namespace UwbInit
