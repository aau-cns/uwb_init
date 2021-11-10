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

#include "uav_init/uwb_init.hpp"

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
      ROS_DEBUG_STREAM("Adding measurment " << uwb_measurements[i].distance << " from anchor "
                                            << uwb_measurements[i].id);
      uwb_data_buffer_[uwb_measurements[i].id].push_back(uwb_measurements[i]);
    }
    else
    {
      ROS_DEBUG_STREAM("DISCARDING measurment " << uwb_measurements[i].distance << " from anchor "
                                                << uwb_measurements[i].id);
    }
  }
}

void UwbInitializer::feed_pose(const double timestamp, const Eigen::Vector3d p_UinG)
{
  // TODO(scm): maybe use buffer here for timesyncing with UWB modules
  // currently this method does not take delayed UWB measurements into account
  //  cur_p_UinG_ = p_UinG;
  buffer_p_UinG_.push_back(timestamp, p_UinG);
}

bool UwbInitializer::try_to_initialize_anchors(std::map<size_t, Eigen::Vector3d>& p_ANCHORSinG, double& distance_bias,
                                               double& const_bias, std::map<size_t, Eigen::Matrix3d>& Anchors_Covs,
                                               double& distance_bias_Cov, double& const_bias_Cov)
{
  // Initialize useful variables
  std::vector<double> distance_biases, const_biases;
  std::vector<double> distance_biases_Covs, const_biases_Covs;

  // For each anchor
  for (uint anchor_id = 0; anchor_id < n_anchors_; ++anchor_id)
  {
    ROS_DEBUG_STREAM("A" << anchor_id << ": calculating solution ...");
    std::vector single_anchor_uwb_data = uwb_data_buffer_.at(anchor_id);

    // Coefficient matrix and measurement vector initialization
    Eigen::MatrixXd coeffs = Eigen::MatrixXd::Zero(single_anchor_uwb_data.size(), 6);
    Eigen::VectorXd measurements = Eigen::VectorXd::Zero(single_anchor_uwb_data.size());

    // Fill the coefficient matrix and the measurement vector
    for (uint i = 0; i < single_anchor_uwb_data.size(); ++i)
    {
      // get closest UWB module position
      Eigen::Vector3d closest_p_UinG = buffer_p_UinG_.get_closest(single_anchor_uwb_data.at(i).timestamp);
      Eigen::VectorXd row(6);
      row << -2 * closest_p_UinG.x(), -2 * closest_p_UinG.y(), -2 * closest_p_UinG.z(),
          std::pow(closest_p_UinG.norm(), 2), 2 * single_anchor_uwb_data.at(i).distance, 1;
      coeffs.row(i) = row.transpose();
      measurements(i) = std::pow(single_anchor_uwb_data.at(i).distance, 2);
    }

    ROS_DEBUG_STREAM("A" << anchor_id << " created problem:\n"
                         << "\tcoeffs:\n" << coeffs << "\n"
                         << "\tmeasurements:\n" << measurements);

    // Alessandro: [check] efficiency
    // Check the coefficient matrix condition number and solve the LS problem
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(coeffs, Eigen::ComputeThinU | Eigen::ComputeThinV);
    double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);
    if (cond < 10000)  // 3
    {
      // [      0     ,       1     ,       2     ,  3 , 4,          5          ]
      // [b^2*p_AinG_x, b^2*p_AinG_y, b^2*p_AinG_z, b^2, k, b^2*norm(p_AinG)-k^2]
      Eigen::VectorXd LSSolution = svd.solve(measurements);
      Eigen::Vector3d p_AinG = LSSolution.segment(0, 3) / LSSolution[3];
      double distance_bias_squared = LSSolution[3];
      double const_bias = LSSolution[4];

      ROS_DEBUG_STREAM("A" << anchor_id << " solution:\n"
                           << "\tLS:         " << LSSolution.transpose() << "\n"
                           << "\tp_AinG:     " << p_AinG.transpose() << "\n"
                           << "\td_bias_sq:  " << distance_bias_squared << "\n"
                           << "\tconst_bias: " << const_bias);

      if ((LSSolution[5] - (distance_bias_squared * std::pow(p_AinG.norm(), 2) - std::pow(const_bias, 2))) < 1)
      {
        // Assign valid estimated values
        distance_biases.push_back(sqrt(distance_bias_squared));
        const_biases.push_back(const_bias);
        p_ANCHORSinG.insert({ anchor_id, p_AinG });

        // Compute estimation Covariance
        Eigen::MatrixXd Cov =
            (std::pow((coeffs * LSSolution - measurements).norm(), 2) / (coeffs.rows() * coeffs.cols())) *
            (svd.matrixV().inverse().transpose() * svd.singularValues().asDiagonal().inverse() *
             svd.singularValues().asDiagonal().inverse() * svd.matrixV().inverse());
        //        Eigen::Matrix4d distance_bias_squared_P_AinG_Cov = Cov.block(0, 0, 3, 3);
        //        double distance_bias_squared_Cov = Cov(4, 4);
        //        double const_bias_Cov = Cov(5, 5);
        ROS_DEBUG_STREAM("\n\tCov:        " << Cov);

        // Retrive P_AinG Covariance applying error propagation law and assign to Anchors_Covs
        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(1, 4);
        J(0, 0) = 1.0 / distance_bias_squared;
        J.block(0, 1, 1, 3) = -p_AinG.transpose() / distance_bias_squared;
        ROS_DEBUG_STREAM("\n\tJ:        " << J);
        //        Anchors_Covs.insert({ anchor_id, J * Cov.block(0, 0, 4, 4) * J.transpose() });

        // Retrive Covariance of b and k applying error propagation law
        distance_biases_Covs.push_back(1 / (4 * distance_bias_squared) * Cov(3, 3));
        const_biases_Covs.push_back(Cov(4, 4));
      }
      else
      {
        ROS_WARN_STREAM("Anchor " << anchor_id << ": issue with LS solution ("
                                  << (LSSolution[5] -
                                      (distance_bias_squared * std::pow(p_AinG.norm(), 2) - std::pow(const_bias, 2)))
                                  << ")");
        return false;
      }
    }
    else
    {
      ROS_WARN_STREAM("Anchor " << anchor_id << ": issue with condition number (" << cond << ")" << std::endl);
      return false;
    }
  }

  assert(distance_biases.size() == n_anchors_);
  assert(const_biases.size() == n_anchors_);

  // Average the biases estimation and covariances
  distance_bias = accumulate(distance_biases.begin(), distance_biases.end(), 0.0) / distance_biases.size();
  const_bias = accumulate(const_biases.begin(), const_biases.end(), 0.0) / const_biases.size();
  distance_bias_Cov =
      accumulate(distance_biases_Covs.begin(), distance_biases_Covs.end(), 0.0) / distance_biases_Covs.size();
  const_bias_Cov = accumulate(const_biases_Covs.begin(), const_biases_Covs.end(), 0.0) / const_biases_Covs.size();

  // DONE :)
  return true;
}

}  // namespace uav_init
