// Copyright (C) 2022 Alessandro Fornasier, Giulio Delama, and Martin Scheiber,
// Control of Networked Systems, University of Klagenfurt, Austria.
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
// You can contact the authors at
// <alessandro.fornasier@aau.at>, <giulio.delama@aau.at>, and
// <martin.scheiber@aau.at>

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <Eigen/Eigen>
#include <concepts>

namespace uwb_init
{
template <typename T>
concept EigenVectord = (std::same_as<T, Eigen::Vector2d> || std::same_as<T, Eigen::Vector3d> ||
                        std::same_as<T, Eigen::Vector4d> || std::same_as<T, Eigen::VectorXd>);

template <typename T>
concept EigenMatrixd = (std::same_as<T, Eigen::Matrix2d> || std::same_as<T, Eigen::Matrix3d> ||
                        std::same_as<T, Eigen::Matrix4d> || std::same_as<T, Eigen::MatrixXd>);

/**
 * @brief linear interpolation
 *
 * @tparam T type of data to be interpolated
 * @param x0 first data point
 * @param x1 second data point
 * @param alpha interpolation parameter [0,1]
 */
template <typename T>
requires(std::floating_point<T> || EigenVectord<T>) T lerp(const T& x0, const T& x1, const double& alpha)
{
    return (1 - alpha) * x0 + alpha * x1;
}

/**
 * @brief Check if a matrix is positive definite via Cholesky decomposition (LLT)
 *
 * @param A matrix
 * @return true if matrix is PD (Positive Definite)
 * @return false otherwise
 */
bool isPD(const Eigen::MatrixXd& A)
{
    const Eigen::LLT<Eigen::MatrixXd> llt(A);
    if (!A.isApprox(A.transpose()) || llt.info() == Eigen::NumericalIssue)
    {
        return false;
    }
    return true;
}

///**
//  * @brief Check if a matrix is semi positive definite via Cholesky decomposition (LLT)
//  *
//  * @param A matrix
//  * @return true if matrix is SPD (Semi Positive Definite)
//  * @return false otherwise
//  */
//bool isSPD(const Eigen::MatrixXd& A)
//{
//    const auto ldlt = A.selfadjointView<Eigen::Upper>().ldlt();
//    if (!A.isApprox(A.transpose()) || ldlt.info() == Eigen::NumericalIssue || !ldlt.isPositive())
//    {
//        return false;
//    }
//    return true;
//}

}  // namespace uwb_init

#endif  // UTILS_HPP_
