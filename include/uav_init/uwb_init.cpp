// Copyright (C) 2021 Martin Scheiber,
// Control of Networked Systems, Universitaet Klagenfurt, Austria
//
// You can contact the author at <martin.scheiber@aau.at>
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
void UwbInitializer::feed_uwb(double timestamp, std::map<size_t, double> uwb_ranges, Eigen::Vector3d p_UinG)
{
  for (const auto& it : uwb_ranges)
  {
    // Append it to our vector
    uwb_data.insert({ it.first, std::make_tuple(p_UinG, timestamp, it.second) });
  }
}

bool UwbInitializer::try_to_initialize_anchors(std::map<size_t, Eigen::Vector3d>& p_ANCHORSinG, double& distance_bias,
                                               double& const_bias, std::map<size_t, Eigen::Matrix3d>& Anchors_Covs,
                                               double& distance_bias_Cov, double& const_bias_Cov)
{
  // Initialize useful variables
  std::vector<double> distance_biases, const_biases;
  std::vector<double> distance_biases_Covs, const_biases_Covs;

  // For each anchor
  for (int anchor_id = 0; anchor_id < _n_anchors; ++anchor_id)
  {
    // Extract block of measurement related with anchor_id
    // Use the First solution if C++17 is not available
    // const auto range = uwb_data.equal_range(anchor_id);
    // for (auto it = range.first; it != range.second; ++it) {
    //  single_anchor_uwb_data.push_back((*it).second);
    //}
    // Alessandro: [check] extract is for C++17 only
    const auto nh = uwb_data.extract(anchor_id);
    single_anchor_uwb_data.push_back(nh.mapped());

    // Coefficient matrix and measurement vector initialization
    Eigen::MatrixXd Coeff = Eigen::MatrixXd::Zero(single_anchor_uwb_data.size(), 6);
    Eigen::VectorXd measurements = Eigen::VectorXd::Zero(single_anchor_uwb_data.size());

    // Fill the coefficient matrix and the measurement vector
    int cnt = 0;
    for (const auto& it : single_anchor_uwb_data)
    {
      Eigen::Vector3d row;
      row << -2 * std::get<0>(it).x(), -2 * std::get<0>(it).y(), -2 * std::get<0>(it).z(),
          std::pow(std::get<0>(it).norm(), 2), 2 * std::get<2>(it), 1;
      Coeff.row(cnt) = row;
      measurements(cnt) = std::pow(std::get<2>(it), 2);
      cnt++;
    }

    // Alessandro: [check] efficiency
    // Check the coefficient matrix condition number and solve the LS problem
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Coeff, Eigen::ComputeThinU | Eigen::ComputeThinV);
    double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);
    if (cond < 3)
    {
      // [      0     ,       1     ,       2     ,  3 , 4,          5          ]
      // [b^2*p_AinG_x, b^2*p_AinG_y, b^2*p_AinG_z, b^2, k, b^2*norm(p_AinG)-k^2]
      Eigen::VectorXd LSSolution = svd.solve(measurements);
      Eigen::Vector3d p_AinG = LSSolution.segment(0, 3) / LSSolution[3];
      double distance_bias_squared = LSSolution[3];
      double const_bias = LSSolution[4];

      if ((LSSolution[5] - (distance_bias_squared * std::pow(p_AinG.norm(), 2) - std::pow(const_bias, 2))) < 1)
      {
        // Assign valid estimated values
        distance_biases.push_back(sqrt(distance_bias_squared));
        const_biases.push_back(const_bias);
        p_ANCHORSinG.insert({ anchor_id, p_AinG });

        // Compute estimation Covariance
        Eigen::MatrixXd Cov =
            (std::pow((Coeff * LSSolution - measurements).norm(), 2) / (Coeff.rows() * Coeff.cols())) *
            (svd.matrixV().inverse().transpose() * svd.singularValues().asDiagonal().inverse() *
             svd.singularValues().asDiagonal().inverse() * svd.matrixV().inverse());
        Eigen::Matrix4d distance_bias_squared_P_AinG_Cov = Cov.block(0, 0, 3, 3);
        double distance_bias_squared_Cov = Cov(4, 4);
        double const_bias_Cov = Cov(5, 5);

        // Retrive P_AinG Covariance applying error propagation law and assign to Anchors_Covs
        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(1, 4);
        J(0, 0) = 1 / distance_bias_squared;
        J.block(0, 1, 1, 3) = -p_AinG / distance_bias_squared;
        Anchors_Covs.insert({ anchor_id, J * Cov.block(0, 0, 4, 4) * J.transpose() });

        // Retrive Covariance of b and k applying error propagation law
        distance_biases_Covs.push_back(1 / (4 * distance_bias_squared) * Cov(3, 3));
        const_biases_Covs.push_back(Cov(4, 4));
      }
      else
      {
        return false;
      }
    }
    else
    {
      return false;
    }
  }

  assert(distance_biases.size() == _n_anchors);
  assert(const_biases.size() == _n_anchors);

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
