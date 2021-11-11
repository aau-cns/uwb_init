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
    // check validity
    if (uwb_measurements[i].valid)
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
  // TODO(scm): maybe use buffer here for timesyncing with UWB modules
  // currently this method does not take delayed UWB measurements into account
  buffer_p_UinG_.push_back(timestamp, p_UinG);
}

bool UwbInitializer::try_to_initialize_anchors(UwbAnchorBuffer& anchor_buffer)
{
  // flag to determine if all anchors are successfully initialized
  bool is_successfull_initialized = true;

  // time of calc
  double calc_time = ros::Time::now().toSec();

  // TODO(scm): this can be improved by making DataBuffer a iterable class
  for (const auto& kv : uwb_data_buffer_.get_buffer())
  {
    // get ID of anchor and its data
    const auto anchor_id = kv.first;
    INIT_DEBUG_STREAM("A" << anchor_id << ": calculating solution ...");

    // check if anchor is already initialized
    if (anchor_buffer.contains_id(anchor_id) && anchor_buffer.is_initialized(anchor_id))
    {
      // anchor already initialized
      INIT_DEBUG_STREAM("A" << anchor_id << ": already initialized");
      is_successfull_initialized &= true;
    }
    else
    {
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
          Eigen::Vector3d closest_p_UinG = buffer_p_UinG_.get_closest(single_anchor_uwb_data.at(i).first);
          Eigen::VectorXd row(6);
          row << -2 * closest_p_UinG.x(), -2 * closest_p_UinG.y(), -2 * closest_p_UinG.z(),
              std::pow(closest_p_UinG.norm(), 2), 2 * single_anchor_uwb_data.at(i).second.distance, 1;
          coeffs.row(i) = row.transpose();
          measurements(i) = std::pow(single_anchor_uwb_data.at(i).second.distance, 2);
        }

        // Check the coefficient matrix condition number and solve the LS problem
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(coeffs, Eigen::ComputeThinU | Eigen::ComputeThinV);
        double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);
        if (cond < params_.max_cond_num)  // 3
        {
          // [      0     ,       1     ,       2     ,  3 , 4,          5          ]
          // [b^2*p_AinG_x, b^2*p_AinG_y, b^2*p_AinG_z, b^2, k, b^2*norm(p_AinG)-k^2]
          Eigen::VectorXd LSSolution = svd.solve(measurements);
          Eigen::Vector3d p_AinG = LSSolution.segment(0, 3) / LSSolution[3];
          double distance_bias_squared = LSSolution[3];
          double const_bias = LSSolution[4];

          INIT_DEBUG_STREAM("A" << anchor_id << " solution:\n"
                                << "\tLS:         " << LSSolution.transpose() << "\n"
                                << "\tp_AinG:     " << p_AinG.transpose() << "\n"
                                << "\td_bias_sq:  " << distance_bias_squared << "\n"
                                << "\tconst_bias: " << const_bias);

          if ((LSSolution[5] - (distance_bias_squared * std::pow(p_AinG.norm(), 2) - std::pow(const_bias, 2))) < 1)
          {
            // Assign valid estimated values
            new_uwb_anchor.bias_d = std::sqrt(distance_bias_squared);
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
                                       << (LSSolution[5] - (distance_bias_squared * std::pow(p_AinG.norm(), 2) -
                                                            std::pow(const_bias, 2)))
                                       << ")");
            new_uwb_anchor.initialized = false;
            is_successfull_initialized &= false;
          }
        }
        else
        {
          INIT_WARN_STREAM("Anchor " << anchor_id << ": issue with condition number (" << cond << ")" << std::endl);
          new_uwb_anchor.initialized = false;
          is_successfull_initialized &= false;
        }  // if anchor data found
      }
      else
      {
        is_successfull_initialized &= false;
      }

      // regardless, add calculation to buffer
      anchor_buffer.push_back(anchor_id, calc_time, new_uwb_anchor);
    }  // else anchor_initialized
  }    // for (const auto& kv : uwb_data_buffer_.get_buffer())

  // return if initialization was successful
  return is_successfull_initialized;
}  // bool UwbInitializer::try_to_initialize_anchors(...)

}  // namespace uav_init
