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

#include "uav_init/uwb_init.hpp"

#include "utils/logging.hpp"

namespace uav_init
{
void UwbInitializer::feed_uwb(const std::vector<UwbData> uwb_measurements)
{
  // add measurements to data buffer
  for (uint i = 0; i < uwb_measurements.size(); ++i)
  {
    // check validity and if measurement is actual bigger than 0.0 (otherwise another error happend)
    if (uwb_measurements[i].valid && uwb_measurements[i].distance > 0.0)
    {
      INIT_DEBUG_STREAM("Adding measurment " << uwb_measurements[i].distance << " from anchor "
                                             << uwb_measurements[i].id);
      //      uwb_data_buffer_[uwb_measurements[i].id].push_back(uwb_measurements[i]);
      uwb_data_buffer_.push_back(uwb_measurements[i].id, uwb_measurements[i].timestamp, uwb_measurements[i]);
    }
    else
    {
      INIT_DEBUG_STREAM("DISCARDING measurment " << uwb_measurements[i].distance << " from anchor "
                                                 << uwb_measurements[i].id);
    }
  }
}

void UwbInitializer::feed_pose(const double timestamp, const Eigen::Vector3d p_UinG)
{
  /// \todo TODO(scm): maybe use buffer here for timesyncing with UWB modules
  /// currently this method does not take delayed UWB measurements into account
  buffer_p_UinG_.push_back(timestamp, p_UinG);
}

bool UwbInitializer::try_to_initialize_anchors(UwbAnchorBuffer& anchor_buffer)
{
  // flag to determine if all anchors are successfully initialized
  bool is_successfull_initialized = true;

  // check if buffers are not empty
  if (buffer_p_UinG_.is_emtpy())
  {
    INIT_DEBUG_STREAM("buffer_p_UinG still emtpy, not initializing.");
    is_successfull_initialized = false;
    return is_successfull_initialized;
  }
  // time of calc
  double calc_time = ros::Time::now().toSec();

  /// \todo TODO(scm): this can be improved by making DataBuffer a iterable class
  for (const auto& kv : uwb_data_buffer_.get_buffer())
  {
    // get ID of anchor and its data
    const auto anchor_id = kv.first;
    INIT_DEBUG_STREAM("A" << anchor_id << ": calculating solution ...");

    // check if anchor is already initialized
    if (!params_.b_do_continous_init && anchor_buffer.contains_id(anchor_id) &&
        anchor_buffer.is_initialized(anchor_id))
    {
      // anchor already initialized
      INIT_DEBUG_STREAM("A" << anchor_id << ": already initialized");
      is_successfull_initialized &= true;
    }
    else
    {
      //      switch (init_method_)
      //      {
      //        case InitMethod::SINGLE:
      //          is_successfull_initialized &= initialize_single(anchor_buffer, anchor_id, calc_time);
      //          break;
      //        case InitMethod::DOUBLE:
      //          is_successfull_initialized &= initialize_double(anchor_buffer, anchor_id, calc_time);
      //          break;
      //        case InitMethod::NO_BIAS:
      //          is_successfull_initialized &= initialize_biasfree(anchor_buffer, anchor_id, calc_time);
      //          break;
      //      }
      is_successfull_initialized &= fx_init_(anchor_buffer, anchor_id, calc_time);
    }  // else anchor_initialized
  }    // for (const auto& kv : uwb_data_buffer_.get_buffer())

  ROS_INFO_STREAM_COND(is_successfull_initialized, "Successfully initialized all known anchors!");
  if (is_successfull_initialized)
    INIT_INFO_STREAM("Successfully initialized all known anchors!");

  // return if initialization was successful
  return is_successfull_initialized;
}  // bool UwbInitializer::try_to_initialize_anchors(...)

bool UwbInitializer::initialize_single_all(UwbAnchorBuffer& anchor_buffer, const uint& anchor_id,
                                           const double& calc_time)
{
  // setup return value
  bool successfully_initialized = true;

  // get anchor measurement data and create result
  UwbAnchor new_uwb_anchor(anchor_id);
  std::deque<std::pair<double, UwbData>> single_anchor_uwb_data;
  if (uwb_data_buffer_.get_buffer_values(anchor_id, single_anchor_uwb_data))
  {
    // Coefficient matrix and measurement vector initialization
    Eigen::MatrixXd coeffs = Eigen::MatrixXd::Zero(single_anchor_uwb_data.size(), 6);
    Eigen::VectorXd measurements = Eigen::VectorXd::Zero(single_anchor_uwb_data.size());

    // Fill the coefficient matrix and the measurement vector
    for (uint i = 0; i < single_anchor_uwb_data.size(); ++i)
    {
      // get closest UWB module position
      Eigen::Vector3d closest_p_UinG =
          buffer_p_UinG_.get_closest(single_anchor_uwb_data.at(i).first - params_.t_pose_diff);
      Eigen::VectorXd row(6);
      row << -2 * closest_p_UinG.x(), -2 * closest_p_UinG.y(), -2 * closest_p_UinG.z(),
          std::pow(closest_p_UinG.norm(), 2), 2 * single_anchor_uwb_data.at(i).second.distance, 1;
      // NOTE(scm): as written in the paper (probably wrong)
      //          row << -2 * closest_p_UinG.x(), -2 * closest_p_UinG.y(), -2 * closest_p_UinG.z(),
      //              std::pow(closest_p_UinG.norm(), 2), 2 * std::pow(single_anchor_uwb_data.at(i).second.distance,
      //              2), 1;
      coeffs.row(i) = row.transpose();
      measurements(i) = std::pow(single_anchor_uwb_data.at(i).second.distance, 2);
    }

    // Check the coefficient matrix condition number and solve the LS problem
    Eigen::BDCSVD<Eigen::MatrixXd> svd(coeffs, Eigen::ComputeThinU | Eigen::ComputeThinV);
    double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);
    //        Eigen::FullPivHouseholderQR<Eigen::MatrixXd> qr = coeffs.fullPivHouseholderQr();
    //        Eigen::JacobiSVD<Eigen::MatrixXd> svd(qr.matrixQR(), Eigen::ComputeThinU | Eigen::ComputeThinV);
    //        double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);
    if (cond < params_.max_cond_num)  // 3
    {
      // [      0     ,       1     ,       2     ,  3 , 4,          5              ]
      // [b^2*p_AinG_x, b^2*p_AinG_y, b^2*p_AinG_z, b^2, k, b^2*(norm(p_AinG)^2-k^2)]
      Eigen::VectorXd LSSolution = svd.solve(measurements);
      //          Eigen::VectorXd LSSolution = qr.solve(measurements);

      // check that the squared value is positive
      if (LSSolution[3] > 0)
      {
        Eigen::Vector3d p_AinG = LSSolution.segment(0, 3) / LSSolution[3];
        double distance_bias_squared = LSSolution[3];
        double const_bias = LSSolution[4];

        INIT_DEBUG_STREAM("A" << anchor_id << " solution:\n"
                              << "\tLS:         " << LSSolution.transpose() << "\n"
                              << "\tp_AinG:     " << p_AinG.transpose() << "\n"
                              << "\td_bias_sq:  " << distance_bias_squared << "\n"
                              << "\tconst_bias: " << const_bias);

        if ((LSSolution[5] - (distance_bias_squared * (std::pow(p_AinG.norm(), 2) - std::pow(const_bias, 2)))) < 1)
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
          //        Eigen::Matrix4d distance_bias_squared_P_AinG_Cov = Cov.block(0, 0, 3, 3);
          //        double distance_bias_squared_Cov = Cov(4, 4);
          //        double const_bias_Cov = Cov(5, 5);
          INIT_DEBUG_STREAM("\n\tCov:        " << Cov);

          // Retrive P_AinG Covariance applying error propagation law and assign to Anchors_Covs
          Eigen::MatrixXd J = Eigen::MatrixXd::Zero(1, 4);
          J(0, 0) = 1.0 / distance_bias_squared;
          J.block(0, 1, 1, 3) = -p_AinG.transpose() / distance_bias_squared;
          INIT_DEBUG_STREAM("\n\tJ:        " << J);
          //        Anchors_Covs.insert({ anchor_id, J * Cov.block(0, 0, 4, 4) * J.transpose() });

          // Retrive Covariance of b and k applying error propagation law
          new_uwb_anchor.cov_bias_d = 1.0 / (4.0 * distance_bias_squared) * Cov(3, 3);
          new_uwb_anchor.cov_bias_c = Cov(4, 4);

          // set initialization to true
          new_uwb_anchor.initialized = true;
        }
        else
        {
          INIT_WARN_STREAM("Anchor " << anchor_id << ": issue with LS solution ("
                                     << (LSSolution[5] -
                                         (distance_bias_squared * std::pow(p_AinG.norm(), 2) - std::pow(const_bias, 2)))
                                     << ")");
          new_uwb_anchor.initialized = false;
          successfully_initialized = false;
        }
      }
      else
      {
        INIT_WARN_STREAM("Anchor " << anchor_id << ": issue with LS positiveness (bias_squared) (" << LSSolution[3]
                                   << ")");
        new_uwb_anchor.initialized = false;
        successfully_initialized = false;
      }
    }
    else
    {
      INIT_WARN_STREAM("Anchor " << anchor_id << ": issue with condition number (" << cond << ")" << std::endl);
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
}  // bool UwbInitializer::initialize_single(...)

bool UwbInitializer::initialize_double_all(UwbAnchorBuffer& anchor_buffer, const uint& anchor_id,
                                           const double& calc_time)
{
  INIT_DEBUG_STREAM("Anchor " << anchor_id << ": using 'DOUBLE' method for initialization");
  // setup return value
  bool successfully_initialized = true;

  // get anchor measurement data and create result
  UwbAnchor new_uwb_anchor(anchor_id);
  std::deque<std::pair<double, UwbData>> single_anchor_uwb_data;
  if (uwb_data_buffer_.get_buffer_values(anchor_id, single_anchor_uwb_data))
  {
    // Coefficient matrix and measurement vector initialization
    //    Eigen::MatrixXd coeffs_pre = Eigen::MatrixXd::Zero(single_anchor_uwb_data.size(), 5);
    //    Eigen::VectorXd measurements_pre = Eigen::VectorXd::Zero(single_anchor_uwb_data.size());
    //    Eigen::MatrixXd coeffs = Eigen::MatrixXd::Zero(single_anchor_uwb_data.size(), 5);
    //    Eigen::VectorXd measurements = Eigen::VectorXd::Zero(single_anchor_uwb_data.size());

    // Coefficient matrix and measurement vector initialization
    std::vector<double> coeffs_vec;
    std::vector<double> meas_vec;

    INIT_DEBUG_STREAM("Anchor " << anchor_id << ": building coefficient matrix ...");

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
      INIT_DEBUG_STREAM("Anchor " << anchor_id << ": calculating svd with matrix of size=" << coeffs.rows() << "x"
                                  << coeffs.cols());

      // Check the coefficient matrix condition number and solve the LS problem
      Eigen::BDCSVD<Eigen::MatrixXd> svd(coeffs, Eigen::ComputeThinU | Eigen::ComputeThinV);
      double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);
      if (cond < params_.max_cond_num)  // 3
      {
        INIT_DEBUG_STREAM("Anchor " << anchor_id << ": solving LS ...");

        // [ 0 ,       1     ,       2     ,  3          , 4]
        // [b^2, b^2*p_AinG_x, b^2*p_AinG_y, b^2*p_AinG_z, k]
        Eigen::VectorXd LSSolution = svd.solve(measurements);

        // check that the squared value is positive
        if (LSSolution[0] > 0)
        {
          Eigen::Vector3d p_AinG = LSSolution.segment(1, 3) / LSSolution[0];
          double distance_bias_squared = LSSolution[0];
          double const_bias = LSSolution[4];

          INIT_DEBUG_STREAM("A" << anchor_id << " solution:\n"
                                << "\tLS:                " << LSSolution.transpose() << "\n"
                                << "\tp_AinG:            " << p_AinG.transpose() << "\n"
                                << "\tbeta_sq:           " << distance_bias_squared << "\n"
                                << "\td_bias(alpha):     " << std::sqrt(distance_bias_squared) - 1.0 << "\n"
                                << "\tconst_bias(gamma): " << const_bias << "\n"
                                << "\tpos_error:         " << (p_AinG - p_AinG_gt_[anchor_id]).transpose() << "("
                                << (p_AinG - p_AinG_gt_[anchor_id]).norm() << ") m");

          // Assign valid estimated values
          new_uwb_anchor.bias_d = std::sqrt(distance_bias_squared);  // NOTE(scm): this is beta with beta=1+alpha
          new_uwb_anchor.bias_c = const_bias;
          new_uwb_anchor.p_AinG = p_AinG;

          // Compute estimation Covariance
          Eigen::MatrixXd Cov =
              (std::pow((coeffs * LSSolution - measurements).norm(), 2) / (coeffs.rows() - coeffs.cols())) *
              ((svd.matrixV().transpose()).inverse() * svd.singularValues().asDiagonal().inverse() *
               svd.singularValues().asDiagonal().inverse() * svd.matrixV().inverse());

          INIT_DEBUG_STREAM("\n\tCov:        " << Cov);

          // compare norm of singular values vector of covarnace matrix with threshold
          Eigen::JacobiSVD<Eigen::MatrixXd> svd_cov(Cov, Eigen::ComputeThinU | Eigen::ComputeThinV);

          // debug
          INIT_INFO_STREAM("\tsingular values:   " << svd_cov.singularValues().transpose());
          INIT_INFO_STREAM("\tsingular v norm:   " << svd_cov.singularValues().norm());
          if (svd_cov.singularValues().norm() <= params_.cov_sv_threshold)
          {
            // set initialization to true
            new_uwb_anchor.initialized = true;
            successfully_initialized = true;
          }
          else
          {
            INIT_WARN_STREAM("Anchor " << anchor_id << ": issue with cov svd threshold ("
                                       << svd_cov.singularValues().norm() << ")");
            new_uwb_anchor.initialized = false;
            successfully_initialized = false;
          }  // if (svd_cov.singularValues().norm() <= params_.cov_sv_threshold_)
        }
        else  // if (LSSolution[0] > 0)
        {
          INIT_WARN_STREAM("Anchor " << anchor_id << ": issue with LS positiveness (bias_squared) (" << LSSolution[0]
                                     << ")");
          new_uwb_anchor.initialized = false;
          successfully_initialized = false;
        }  // if (LSSolution[0] > 0)
      }
      else  // if (cond < params_.max_cond_num)
      {
        INIT_WARN_STREAM("Anchor " << anchor_id << ": issue with condition number (" << cond << ")" << std::endl);
        new_uwb_anchor.initialized = false;
        successfully_initialized = false;
      }  // if (cond < params_.max_cond_num)
    }
    else
    {
      //      INIT_WARN_STREAM("Anchor " << anchor_id << ": issue with measurement baselines (c:" << coeffs_vec.size()
      //                                 << " m:" << meas_vec.size() << ")" << std::endl);
      INIT_WARN_STREAM("Anchor " << anchor_id << ": issue with measurement baselines (rows:" << coeffs_vec.size() / 5
                                 << ")" << std::endl);
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
}  // bool UwbInitializer::initialize_single(...)

bool UwbInitializer::initialize_single_nobias(UwbAnchorBuffer& anchor_buffer, const uint& anchor_id,
                                              const double& calc_time)
{
  INIT_DEBUG_STREAM("Anchor " << anchor_id << ": using 'NO_BIAS' method for initialization");

  // setup return value
  bool successfully_initialized = true;

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
    //        Eigen::FullPivHouseholderQR<Eigen::MatrixXd> qr = coeffs.fullPivHouseholderQr();
    //        Eigen::JacobiSVD<Eigen::MatrixXd> svd(qr.matrixQR(), Eigen::ComputeThinU | Eigen::ComputeThinV);
    //        double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);
    if (cond < params_.max_cond_num)  // 3
    {
      // [      0     ,       1     ,       2     ,  3 , 4,          5              ]
      // [b^2*p_AinG_x, b^2*p_AinG_y, b^2*p_AinG_z, b^2, k, b^2*(norm(p_AinG)^2-k^2)]
      Eigen::VectorXd LSSolution = svd.solve(measurements);
      //          Eigen::VectorXd LSSolution = qr.solve(measurements);

      Eigen::Vector3d p_AinG = LSSolution.segment(0, 3);
      double distance_bias_squared = 0.0;
      double const_bias = 0.0;

      INIT_DEBUG_STREAM("A" << anchor_id << " solution:\n"
                            << "\tLS:         " << LSSolution.transpose() << "\n"
                            << "\tp_AinG:     " << p_AinG.transpose() << "\n"
                            << "\td_bias_sq:  " << distance_bias_squared << "\n"
                            << "\tconst_bias: " << const_bias);

      if ((LSSolution[3] - (std::pow(p_AinG.norm(), 2))) < 1)
      {
        // Assign valid estimated values
        new_uwb_anchor.bias_d = std::sqrt(distance_bias_squared);  // NOTE(scm): this is beta with beta=1+alpha
        new_uwb_anchor.bias_c = const_bias;
        new_uwb_anchor.p_AinG = p_AinG;

        //        // Compute estimation Covariance
        //        Eigen::MatrixXd Cov =
        //            (std::pow((coeffs * LSSolution - measurements).norm(), 2) / (coeffs.rows() * coeffs.cols())) *
        //            (svd.matrixV().inverse().transpose() * svd.singularValues().asDiagonal().inverse() *
        //             svd.singularValues().asDiagonal().inverse() * svd.matrixV().inverse());
        //        //        Eigen::Matrix4d distance_bias_squared_P_AinG_Cov = Cov.block(0, 0, 3, 3);
        //        //        double distance_bias_squared_Cov = Cov(4, 4);
        //        //        double const_bias_Cov = Cov(5, 5);
        //        INIT_DEBUG_STREAM("\n\tCov:        " << Cov);

        //        // Retrive P_AinG Covariance applying error propagation law and assign to Anchors_Covs
        //        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(1, 4);
        //        J(0, 0) = 1.0 / distance_bias_squared;
        //        J.block(0, 1, 1, 3) = -p_AinG.transpose() / distance_bias_squared;
        //        INIT_DEBUG_STREAM("\n\tJ:        " << J);
        //        //        Anchors_Covs.insert({ anchor_id, J * Cov.block(0, 0, 4, 4) * J.transpose() });

        //        // Retrive Covariance of b and k applying error propagation law
        //        new_uwb_anchor.cov_bias_d = 1.0 / (4.0 * distance_bias_squared) * Cov(3, 3);
        //        new_uwb_anchor.cov_bias_c = Cov(4, 4);

        // set initialization to true
        new_uwb_anchor.initialized = true;
      }
      else
      {
        INIT_WARN_STREAM("Anchor " << anchor_id << ": issue with LS solution ("
                                   << (LSSolution[3] - (std::pow(p_AinG.norm(), 2))) << ")");
        new_uwb_anchor.initialized = false;
        successfully_initialized = false;
      }
    }
    else
    {
      INIT_WARN_STREAM("Anchor " << anchor_id << ": issue with condition number (" << cond << ")" << std::endl);
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
}  // bool UwbInitializer::initialize_single(...)

}  // namespace uav_init
