﻿// Copyright (C) 2021 Martin Scheiber, Alessandro Fornasier
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
    // reset buffers
    uwb_data_buffer_.reset();
    buffer_p_UinG_.reset();

    // reset positions
    cur_p_UinG_ = Eigen::Vector3d::Zero();
}


void UwbInitializer::feed_uwb(const std::vector<UwbData> uwb_measurements)
{
    // add measurements to data buffer
    for (uint i = 0; i < uwb_measurements.size(); ++i)
    {
        // check validity and if measurement is actual bigger than 0.0 (otherwise another error happend)
        if (uwb_measurements[i].valid && uwb_measurements[i].distance > 0.0)
        {
            uwb_data_buffer_.push_back(uwb_measurements[i].id, uwb_measurements[i].timestamp, uwb_measurements[i]);
        }
        else
        {
            // TODO (gid)
            //      INIT_DEBUG_STREAM("DISCARDING measurment " << uwb_measurements[i].distance << " from anchor "
            //                                                 << uwb_measurements[i].id);
        }
    }
}


void UwbInitializer::feed_pose(const double timestamp, const Eigen::Vector3d p_UinG)
{
    /// \todo TODO(scm): maybe use buffer here for timesyncing with UWB modules
    /// currently this method does not take delayed UWB measurements into account
    buffer_p_UinG_.push_back(timestamp, p_UinG);
}


bool UwbInitializer::init_anchors(UwbAnchorBuffer& anchor_buffer)
{
    // flag to determine if all anchors are successfully initialized
    bool is_successfull_initialized = true;

    // check if buffers are not empty
    if (buffer_p_UinG_.is_emtpy())
    {
        is_successfull_initialized = false;
        return is_successfull_initialized;
    }

    /// \todo TODO(scm): this can be improved by making DataBuffer a iterable class
    for (const auto& kv : uwb_data_buffer_.get_buffer())
    {
        // get ID of anchor and its data
        const auto anchor_id = kv.first;

        // check if anchor is already initialized
        if (!params_.b_do_continous_init && anchor_buffer.contains_id(anchor_id) &&
                anchor_buffer.is_initialized(anchor_id))
        {
            is_successfull_initialized &= true;
        }
        else
        {
            is_successfull_initialized &= solve_ls(anchor_buffer, anchor_id);
        }  // else anchor_initialized
    }    // for (const auto& kv : uwb_data_buffer_.get_buffer())

    // TODO (gid)
    //  ROS_INFO_STREAM_COND(is_successfull_initialized, "Successfully initialized all known anchors!");

    // return if initialization was successful
    return is_successfull_initialized;
}


bool UwbInitializer::solve_ls(UwbAnchorBuffer& anchor_buffer, const uint& anchor_id)
{

    // TODO (gid) get rid of it
    double calc_time = 0.0;

    // define result
    UwbAnchor new_uwb_anchor(anchor_id);

    // Define uwb data
    std::deque<std::pair<double, UwbData>> single_anchor_uwb_data;

    // First check
    if ( !(uwb_data_buffer_.get_buffer_values(anchor_id, single_anchor_uwb_data)) )
    {
        return false;
    }

    // Coefficient matrix and measurement vector initialization
    Eigen::MatrixXd coeffs;
    Eigen::VectorXd meas;

    ls_solver_(single_anchor_uwb_data, coeffs, meas);

    // Check the coefficient matrix condition number and solve the LS problem
    Eigen::BDCSVD<Eigen::MatrixXd> svd(coeffs, Eigen::ComputeThinU | Eigen::ComputeThinV);
    double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);

    // Second check (condition number) TODO (gid) add logger
    if ( !(cond < params_.max_cond_num) )
    {
        new_uwb_anchor.initialized = false;
        return false;
    }

    // [      0     ,       1     ,       2     ,  3 , 4,          5              ]
    // [b^2*p_AinG_x, b^2*p_AinG_y, b^2*p_AinG_z, b^2, k, b^2*(norm(p_AinG)^2-k^2)]
    Eigen::VectorXd LSSolution = svd.solve(meas);

    // Third check (beta squared must be positive)
    if ( !(LSSolution[3] > 0) )
    {
        new_uwb_anchor.initialized = false;
        return false;
    }

    Eigen::Vector3d p_AinG = LSSolution.segment(0, 3) / LSSolution[3];
    double distance_bias_squared = LSSolution[3];
    double const_bias = LSSolution[4];

    // Fourth check
    if ( !((LSSolution[5] - (distance_bias_squared * (std::pow(p_AinG.norm(), 2) - std::pow(const_bias, 2)))) < 1) )
    {
        new_uwb_anchor.initialized = false;
        return false;
    }

    // Assign valid estimated values
    new_uwb_anchor.bias_d = std::sqrt(distance_bias_squared);  // NOTE(scm): this is beta with beta=1+alpha
    new_uwb_anchor.bias_c = const_bias;
    new_uwb_anchor.p_AinG = p_AinG;

    // Compute estimation Covariance
    Eigen::MatrixXd Cov =
            (std::pow((coeffs * LSSolution - meas).norm(), 2) / (coeffs.rows() * coeffs.cols())) *
            (svd.matrixV().inverse().transpose() * svd.singularValues().asDiagonal().inverse() *
             svd.singularValues().asDiagonal().inverse() * svd.matrixV().inverse());

    // compare norm of singular values vector of covarnace matrix with threshold
    Eigen::JacobiSVD<Eigen::MatrixXd> svd_cov(Cov, Eigen::ComputeThinU | Eigen::ComputeThinV);

    if ( !(svd_cov.singularValues().norm() <= params_.cov_sv_threshold) )
    {
        new_uwb_anchor.initialized = false;
        return false;

    }

    // If all checks passed
    // Retrive P_AinG Covariance applying error propagation law and assign to Anchors_Covs
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(3, 4);
    J.block(0, 0, 3, 3) = (1.0 / distance_bias_squared) * Eigen::Matrix3d::Identity();
    J.block(0, 3, 3, 1) = -p_AinG / distance_bias_squared;
    new_uwb_anchor.cov_p_AinG = J * Cov.block(0, 0, 4, 4) * J.transpose();

    // Retrive Covariance of b and k applying error propagation law
    new_uwb_anchor.cov_bias_d = 1.0 / (4.0 * distance_bias_squared) * Cov(3, 3);
    new_uwb_anchor.cov_bias_c = Cov(4, 4);

    // set initialization to true
    new_uwb_anchor.initialized = true;

    // regardless, add calculation to buffer
    anchor_buffer.push_back(anchor_id, calc_time, new_uwb_anchor);

    return true;
}

void UwbInitializer::ls_single_full_bias(std::deque<std::pair<double, UwbData>> &uwb_data, Eigen::MatrixXd &A, Eigen::VectorXd &b)
{
    // Coefficient matrix and measurement vector initialization
    A = Eigen::MatrixXd::Zero(uwb_data.size(), 6);
    b = Eigen::VectorXd::Zero(uwb_data.size());

    // Fill the coefficient matrix and the measurement vector
    for (uint i = 0; i < uwb_data.size(); ++i)
    {
        // get closest UWB module position
        Eigen::Vector3d closest_p_UinG =
                buffer_p_UinG_.get_closest(uwb_data.at(i).first - params_.t_pose_diff);
        Eigen::VectorXd row(6);
        row << -2 * closest_p_UinG.x(), -2 * closest_p_UinG.y(), -2 * closest_p_UinG.z(),
                std::pow(closest_p_UinG.norm(), 2), 2 * uwb_data.at(i).second.distance, 1;
        A.row(i) = row.transpose();
        b(i) = std::pow(uwb_data.at(i).second.distance, 2);
    }

}

void UwbInitializer::ls_double_full_bias(std::deque<std::pair<double, UwbData> > &uwb_data, Eigen::MatrixXd &A, Eigen::VectorXd &b)
{
    // Coefficient matrix and measurement vector initialization
    std::vector<double> coeffs_vec;
    std::vector<double> meas_vec;

    // TODO (gid)
    //    INIT_DEBUG_STREAM("Anchor " << anchor_id << ": building coefficient matrix ...");

    // Fill the coefficient matrix and the measurement vector
    for (uint i = 0; i < uwb_data.size() - params_.meas_baseline_idx_; ++i)
    {
        // baseline check
        double diff = uwb_data.at(i).second.distance -
                uwb_data.at(i + params_.meas_baseline_idx_).second.distance;
        if (std::abs(diff) > params_.meas_baseline_m_)
        {
            // get closest UWB module position and check if closes was actually reached
            Eigen::Vector3d closest_p_UinG1, closest_p_UinG2;
            if (buffer_p_UinG_.get_closest(uwb_data.at(i).first - params_.t_pose_diff, closest_p_UinG1) &&
                    buffer_p_UinG_.get_closest(
                        uwb_data.at(i + params_.meas_baseline_idx_).first - params_.t_pose_diff, closest_p_UinG2))
            {
                Eigen::VectorXd row(5);
                row << std::pow(closest_p_UinG1.norm(), 2) - std::pow(closest_p_UinG2.norm(), 2),
                        -2 * (closest_p_UinG1.x() - closest_p_UinG2.x()), -2 * (closest_p_UinG1.y() - closest_p_UinG2.y()),
                        -2 * (closest_p_UinG1.z() - closest_p_UinG2.z()), 2 * diff;

                coeffs_vec.push_back(row(0));
                coeffs_vec.push_back(row(1));
                coeffs_vec.push_back(row(2));
                coeffs_vec.push_back(row(3));
                coeffs_vec.push_back(row(4));
                meas_vec.push_back(std::pow(uwb_data.at(i).second.distance, 2) -
                                   std::pow(uwb_data.at(i + params_.meas_baseline_idx_).second.distance, 2));

            }  // if (closest available)
        }    // if (baseline check)
    }      // for (all anchor measurements)

    // Assertation, check vectors size
    assert(coeffs_vec.size() == 5 * meas_vec.size());

    // Check to have more than 5 rows in the coefficient matrix
    if (coeffs_vec.size() > 5 * 5)
    {
        if (params_.lamda > 0.0)
        {
            // Data augmentation for regularization
            coeffs_vec.push_back(std::sqrt(params_.lamda));  // b^2
            // add 23 zero lines to fill 'diag matrix
            if (params_.b_regularize_z)
            {
                // with z regularization
                for (uint cnt_line = 0; cnt_line < 17; ++cnt_line)
                    coeffs_vec.push_back(0.0);                      // b^2*p_AinG except z
                coeffs_vec.push_back(std::sqrt(params_.lamda));  // b^2*p_AinG_z
                for (uint cnt_line = 18; cnt_line < 23; ++cnt_line)
                    coeffs_vec.push_back(0.0);  // b^2*p_AinG except z
            }
            else
            {
                for (uint cnt_line = 0; cnt_line < 23; ++cnt_line)
                    coeffs_vec.push_back(0.0);  // b^2*p_AinG
            }
            coeffs_vec.push_back(std::sqrt(params_.lamda));  // k

            // add values to meas_vec
            meas_vec.push_back(std::sqrt(params_.lamda));
            meas_vec.push_back(0.0);
            meas_vec.push_back(0.0);
            meas_vec.push_back(0.0);
            meas_vec.push_back(0.0);
        }

        // Map vectors to Eigen matrices
        A = Eigen::MatrixXd::Zero(coeffs_vec.size() / 5, 5);
        b = Eigen::VectorXd::Zero(meas_vec.size());
        A = Eigen::MatrixXd(Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
                                coeffs_vec.data(), coeffs_vec.size() / 5, 5));
        b = Eigen::VectorXd(
                    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>(meas_vec.data(), meas_vec.size(), 1));


    }  // bool UwbInitializer::ls_single_full_bias(...)


    bool UwbInitializer::ls_double_full_bias(UwbAnchorBuffer& anchor_buffer, const uint& anchor_id,
                                             const double& calc_time)
    {
        // TODO (gid)
        //  INIT_DEBUG_STREAM("Anchor " << anchor_id << ": using 'DOUBLE' method for initialization");

        // setup return value
        bool successfully_initialized = false;

        // get anchor measurement data and create result
        UwbAnchor new_uwb_anchor(anchor_id);
        std::deque<std::pair<double, UwbData>> single_anchor_uwb_data;
        if (uwb_data_buffer_.get_buffer_values(anchor_id, single_anchor_uwb_data))
        {
            // Coefficient matrix and measurement vector initialization
            std::vector<double> coeffs_vec;
            std::vector<double> meas_vec;

            // TODO (gid)
            //    INIT_DEBUG_STREAM("Anchor " << anchor_id << ": building coefficient matrix ...");

            // Fill the coefficient matrix and the measurement vector
            for (uint i = 0; i < single_anchor_uwb_data.size() - params_.meas_baseline_idx_; ++i)
            {
                // baseline check
                double diff = single_anchor_uwb_data.at(i).second.distance -
                        single_anchor_uwb_data.at(i + params_.meas_baseline_idx_).second.distance;
                if (std::abs(diff) > params_.meas_baseline_m_)
                {
                    // get closest UWB module position and check if closes was actually reached
                    Eigen::Vector3d closest_p_UinG1, closest_p_UinG2;
                    if (buffer_p_UinG_.get_closest(single_anchor_uwb_data.at(i).first - params_.t_pose_diff, closest_p_UinG1) &&
                            buffer_p_UinG_.get_closest(
                                single_anchor_uwb_data.at(i + params_.meas_baseline_idx_).first - params_.t_pose_diff, closest_p_UinG2))
                    {
                        Eigen::VectorXd row(5);
                        row << std::pow(closest_p_UinG1.norm(), 2) - std::pow(closest_p_UinG2.norm(), 2),
                                -2 * (closest_p_UinG1.x() - closest_p_UinG2.x()), -2 * (closest_p_UinG1.y() - closest_p_UinG2.y()),
                                -2 * (closest_p_UinG1.z() - closest_p_UinG2.z()), 2 * diff;

                        coeffs_vec.push_back(row(0));
                        coeffs_vec.push_back(row(1));
                        coeffs_vec.push_back(row(2));
                        coeffs_vec.push_back(row(3));
                        coeffs_vec.push_back(row(4));
                        meas_vec.push_back(std::pow(single_anchor_uwb_data.at(i).second.distance, 2) -
                                           std::pow(single_anchor_uwb_data.at(i + params_.meas_baseline_idx_).second.distance, 2));

                    }  // if (closest available)
                }    // if (baseline check)
            }      // for (all anchor measurements)

            //    INIT_DEBUG_STREAM("Anchor " << anchor_id << ": assigning coefficients and measurements\n"
            //                                << "\t coeffs_size: " << coeffs_vec.size() << "\n"
            //                                << "\t meas_size:   " << meas_vec.size());

            // Assertation, check vectors size
            assert(coeffs_vec.size() == 5 * meas_vec.size());

            // Check to have more than 5 rows in the coefficient matrix
            if (coeffs_vec.size() > 5 * 5)
            {
                if (params_.lamda > 0.0)
                {
                    // Data augmentation for regularization
                    coeffs_vec.push_back(std::sqrt(params_.lamda));  // b^2
                    // add 23 zero lines to fill 'diag matrix
                    if (params_.b_regularize_z)
                    {
                        // with z regularization
                        for (uint cnt_line = 0; cnt_line < 17; ++cnt_line)
                            coeffs_vec.push_back(0.0);                      // b^2*p_AinG except z
                        coeffs_vec.push_back(std::sqrt(params_.lamda));  // b^2*p_AinG_z
                        for (uint cnt_line = 18; cnt_line < 23; ++cnt_line)
                            coeffs_vec.push_back(0.0);  // b^2*p_AinG except z
                    }
                    else
                    {
                        for (uint cnt_line = 0; cnt_line < 23; ++cnt_line)
                            coeffs_vec.push_back(0.0);  // b^2*p_AinG
                    }
                    coeffs_vec.push_back(std::sqrt(params_.lamda));  // k

                    // add values to meas_vec
                    meas_vec.push_back(std::sqrt(params_.lamda));
                    meas_vec.push_back(0.0);
                    meas_vec.push_back(0.0);
                    meas_vec.push_back(0.0);
                    meas_vec.push_back(0.0);
                }

                // Map vectors to Eigen matrices
                Eigen::MatrixXd coeffs = Eigen::MatrixXd::Zero(coeffs_vec.size() / 5, 5);
                Eigen::VectorXd measurements = Eigen::VectorXd::Zero(meas_vec.size());
                coeffs = Eigen::MatrixXd(Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
                                             coeffs_vec.data(), coeffs_vec.size() / 5, 5));
                measurements = Eigen::VectorXd(
                            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>(meas_vec.data(), meas_vec.size(), 1));

                // INIT_DEBUG_STREAM("Anchor " << anchor_id << ": matrix=\n" << coeffs);
                // INIT_DEBUG_STREAM("Anchor " << anchor_id << ": vec=\n" << measurements);
                // TODO (gid)
                //      INIT_DEBUG_STREAM("Anchor " << anchor_id << ": calculating svd with matrix of size=" << coeffs.rows() << "x"
                //                                  << coeffs.cols());

                // Check the coefficient matrix condition number and solve the LS problem
                Eigen::BDCSVD<Eigen::MatrixXd> svd(coeffs, Eigen::ComputeThinU | Eigen::ComputeThinV);
                double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);
                if (cond < params_.max_cond_num)  // 3
                {
                    // TODO (gid)
                    //        INIT_DEBUG_STREAM("Anchor " << anchor_id << ": solving LS ...");

                    // [ 0 ,       1     ,       2     ,  3          , 4]
                    // [b^2, b^2*p_AinG_x, b^2*p_AinG_y, b^2*p_AinG_z, k]
                    Eigen::VectorXd LSSolution = svd.solve(measurements);

                    // check that the squared value is positive
                    if (LSSolution[0] > 0)
                    {
                        Eigen::Vector3d p_AinG = LSSolution.segment(1, 3) / LSSolution[0];
                        double distance_bias_squared = LSSolution[0];
                        double const_bias = LSSolution[4];

                        // TODO (gid)
                        //          INIT_DEBUG_STREAM("A" << anchor_id << " solution:\n"
                        //                                << "\tLS:                " << LSSolution.transpose() << "\n"
                        //                                << "\tp_AinG:            " << p_AinG.transpose() << "\n"
                        //                                << "\tbeta_sq:           " << distance_bias_squared << "\n"
                        //                                << "\td_bias(alpha):     " << std::sqrt(distance_bias_squared) - 1.0 << "\n"
                        //                                << "\tconst_bias(gamma): " << const_bias << "\n");
                        //                                << "\tpos_error:         " << (p_AinG - p_AinG_gt_[anchor_id]).transpose() << "("
                        //                                << (p_AinG - p_AinG_gt_[anchor_id]).norm() << ") m");

                        // Assign valid estimated values
                        new_uwb_anchor.bias_d = std::sqrt(distance_bias_squared);  // NOTE(scm): this is beta with beta=1+alpha
                        new_uwb_anchor.bias_c = const_bias;
                        new_uwb_anchor.p_AinG = p_AinG;

                        // Compute estimation Covariance
                        Eigen::MatrixXd Cov =
                                (std::pow((coeffs * LSSolution - measurements).norm(), 2) / (coeffs.rows() - coeffs.cols())) *
                                ((svd.matrixV().transpose()).inverse() * svd.singularValues().asDiagonal().inverse() *
                                 svd.singularValues().asDiagonal().inverse() * svd.matrixV().inverse());
                        // TODO (gid)
                        //          INIT_DEBUG_STREAM("\n\tCov:        " << Cov);

                        // compare norm of singular values vector of covarnace matrix with threshold
                        Eigen::JacobiSVD<Eigen::MatrixXd> svd_cov(Cov, Eigen::ComputeThinU | Eigen::ComputeThinV);

                        // debug
                        // TODO (gid)
                        //          INIT_INFO_STREAM("\tsingular values:   " << svd_cov.singularValues().transpose());
                        //          INIT_INFO_STREAM("\tsingular v norm:   " << svd_cov.singularValues().norm());
                        if (svd_cov.singularValues().norm() <= params_.cov_sv_threshold)
                        {

                            // Retrive P_AinG Covariance applying error propagation law and assign to Anchors_Covs
                            Eigen::MatrixXd J = Eigen::MatrixXd::Zero(3, 4);
                            J.block(0, 0, 3, 1) = -p_AinG / distance_bias_squared;
                            J.block(0, 1, 3, 3) = (1.0 / distance_bias_squared) * Eigen::Matrix3d::Identity();
                            // TODO (gid)
                            //            INIT_DEBUG_STREAM("\n\tJ:        " << J);
                            new_uwb_anchor.cov_p_AinG = J * Cov.block(0, 0, 4, 4) * J.transpose();

                            // Retrive Covariance of b and k applying error propagation law
                            new_uwb_anchor.cov_bias_d = 1.0 / (4.0 * distance_bias_squared) * Cov(0, 0);
                            new_uwb_anchor.cov_bias_c = Cov(4, 4);

                            // set initialization to true
                            new_uwb_anchor.initialized = true;
                            successfully_initialized = true;
                        }
                        else
                        {
                            // TODO (gid)
                            //            INIT_WARN_STREAM("Anchor " << anchor_id << ": issue with cov svd threshold ("
                            //                                       << svd_cov.singularValues().norm() << ")");
                            new_uwb_anchor.initialized = false;
                            successfully_initialized = false;
                        }  // if (svd_cov.singularValues().norm() <= params_.cov_sv_threshold_)
                    }
                    else  // if (LSSolution[0] > 0)
                    {
                        // TODO (gid)
                        //          INIT_WARN_STREAM("Anchor " << anchor_id << ": issue with LS positiveness (bias_squared) (" << LSSolution[0]
                        //                                     << ")");
                        new_uwb_anchor.initialized = false;
                        successfully_initialized = false;
                    }  // if (LSSolution[0] > 0)
                }
                else  // if (cond < params_.max_cond_num)
                {
                    // TODO (gid)
                    //        INIT_WARN_STREAM("Anchor " << anchor_id << ": issue with condition number (" << cond << ")" << std::endl);
                    new_uwb_anchor.initialized = false;
                    successfully_initialized = false;
                }  // if (cond < params_.max_cond_num)
            }
            else
            {
                //      INIT_WARN_STREAM("Anchor " << anchor_id << ": issue with measurement baselines (c:" << coeffs_vec.size()
                //                                 << " m:" << meas_vec.size() << ")" << std::endl);
                // TODO (gid)
                //      INIT_WARN_STREAM("Anchor " << anchor_id << ": issue with measurement baselines (rows:" << coeffs_vec.size() / 5
                //                                 << ")" << std::endl);
                new_uwb_anchor.initialized = false;
                successfully_initialized = false;
            }
        }
        else
        {
            successfully_initialized = false;
        }

        // regardless, add calculation to buffer
        anchor_buffer.push_back(anchor_id, calc_time, new_uwb_anchor);

        return successfully_initialized;
    }  // bool UwbInitializer::ls_double_full_bias(...)


    bool UwbInitializer::ls_single_no_bias(UwbAnchorBuffer& anchor_buffer, const uint& anchor_id,
                                           const double& calc_time)
    {
        // TODO (gid)
        //  INIT_DEBUG_STREAM("Anchor " << anchor_id << ": using 'NO_BIAS' method for initialization");

        // setup return value
        bool successfully_initialized = false;

        // get anchor measurement data and create result
        UwbAnchor new_uwb_anchor(anchor_id);
        std::deque<std::pair<double, UwbData>> single_anchor_uwb_data;
        if (uwb_data_buffer_.get_buffer_values(anchor_id, single_anchor_uwb_data))
        {
            // Coefficient matrix and measurement vector initialization
            Eigen::MatrixXd coeffs = Eigen::MatrixXd::Zero(single_anchor_uwb_data.size(), 4);
            Eigen::VectorXd measurements = Eigen::VectorXd::Zero(single_anchor_uwb_data.size());

            // Fill the coefficient matrix and the measurement vector
            for (uint i = 0; i < single_anchor_uwb_data.size(); ++i)
            {
                // get closest UWB module position
                Eigen::Vector3d closest_p_UinG =
                        buffer_p_UinG_.get_closest(single_anchor_uwb_data.at(i).first - params_.t_pose_diff);

                Eigen::VectorXd row(4);
                row << -2 * closest_p_UinG.x(), -2 * closest_p_UinG.y(), -2 * closest_p_UinG.z(), 1;

                coeffs.row(i) = row.transpose();
                measurements(i) = std::pow(single_anchor_uwb_data.at(i).second.distance, 2) - std::pow(closest_p_UinG.norm(), 2);
            }

            // Check the coefficient matrix condition number and solve the LS problem
            Eigen::BDCSVD<Eigen::MatrixXd> svd(coeffs, Eigen::ComputeThinU | Eigen::ComputeThinV);
            double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);
            if (cond < params_.max_cond_num)  // 3
            {
                // [    0    ,    1   ,    2   ,        3       ]
                // [p_AinG_x, p_AinG_y, p_AinG_z, norm(p_AinG)^2]
                Eigen::VectorXd LSSolution = svd.solve(measurements);

                Eigen::Vector3d p_AinG = LSSolution.segment(0, 3);
                double distance_bias_squared = 0.0;
                double const_bias = 0.0;

                // TODO (gid)
                //      INIT_DEBUG_STREAM("A" << anchor_id << " solution:\n"
                //                            << "\tLS:         " << LSSolution.transpose() << "\n"
                //                            << "\tp_AinG:     " << p_AinG.transpose() << "\n"
                //                            << "\td_bias_sq:  " << distance_bias_squared << "\n"
                //                            << "\tconst_bias: " << const_bias);

                if ((LSSolution[3] - (std::pow(p_AinG.norm(), 2))) < 1)
                {
                    // Assign valid estimated values
                    new_uwb_anchor.bias_d = std::sqrt(distance_bias_squared);  // NOTE(scm): this is beta with beta=1+alpha
                    new_uwb_anchor.bias_c = const_bias;
                    new_uwb_anchor.p_AinG = p_AinG;

                    // Compute estimation Covariance
                    Eigen::MatrixXd Cov =
                            (std::pow((coeffs * LSSolution - measurements).norm(), 2) / (coeffs.rows() * coeffs.cols())) *
                            (svd.matrixV().inverse().transpose() * svd.singularValues().asDiagonal().inverse() *
                             svd.singularValues().asDiagonal().inverse() * svd.matrixV().inverse());
                    // TODO (gid)
                    //        INIT_DEBUG_STREAM("\n\tCov:        " << Cov);

                    // compare norm of singular values vector of covarnace matrix with threshold
                    Eigen::JacobiSVD<Eigen::MatrixXd> svd_cov(Cov, Eigen::ComputeThinU | Eigen::ComputeThinV);

                    // debug
                    // TODO (gid)
                    //        INIT_INFO_STREAM("\tsingular values:   " << svd_cov.singularValues().transpose());
                    //        INIT_INFO_STREAM("\tsingular v norm:   " << svd_cov.singularValues().norm());
                    if (svd_cov.singularValues().norm() <= params_.cov_sv_threshold)
                    {

                        // Retrive P_AinG Covariance
                        new_uwb_anchor.cov_p_AinG = Cov.block(0, 0, 3, 3);

                        // Retrive Covariance of b and k applying error propagation law
                        new_uwb_anchor.cov_bias_d = 0.0;
                        new_uwb_anchor.cov_bias_c = 0.0;

                        // set initialization to true
                        new_uwb_anchor.initialized = true;
                        successfully_initialized = true;

                    }
                    else
                    {
                        // TODO (gid)
                        //          INIT_WARN_STREAM("Anchor " << anchor_id << ": issue with cov svd threshold ("
                        //                                     << svd_cov.singularValues().norm() << ")");
                        new_uwb_anchor.initialized = false;
                        successfully_initialized = false;
                    }  // if (svd_cov.singularValues().norm() <= params_.cov_sv_threshold_)
                }
                else
                {
                    // TODO (gid)
                    //        INIT_WARN_STREAM("Anchor " << anchor_id << ": issue with LS solution ("
                    //                                   << (LSSolution[3] - (std::pow(p_AinG.norm(), 2))) << ")");
                    new_uwb_anchor.initialized = false;
                    successfully_initialized = false;
                }
            }
            else
            {
                // TODO (gid)
                //      INIT_WARN_STREAM("Anchor " << anchor_id << ": issue with condition number (" << cond << ")" << std::endl);
                new_uwb_anchor.initialized = false;
                successfully_initialized = false;
            }  // if anchor data found
        }
        else
        {
            successfully_initialized = false;
        }

        // regardless, add calculation to buffer
        anchor_buffer.push_back(anchor_id, calc_time, new_uwb_anchor);

        return successfully_initialized;
    }  // bool UwbInitializer::ls_single_no_bias(...)

}  // namespace UwbInit
