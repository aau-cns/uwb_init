// Copyright (C) 2022 Alessandro Fornasier, Giulio Delama and Martin Scheiber.
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
// You can contact the authors at <alessandro.fornasier@aau.at>,
// <giulio.delama@aau.at> and <martin.scheiber@aau.at>

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <Eigen/Eigen>
#include <array>
#include <iterator>
#include <random>
#include <type_traits>
#include <vector>
#include <algorithm>

namespace uwb_init
{
/**
 * @brief linear interpolation
 *
 * @tparam T type of data to be interpolated
 * @param x0 first data point
 * @param x1 second data point
 * @param alpha interpolation parameter [0,1]
 */
template <typename T, typename std::enable_if<std::is_floating_point_v<T> || std::is_base_of_v<Eigen::MatrixBase<T>, T>,
                                              T>::type* = nullptr>
T lerp(const T& x0, const T& x1, const double& alpha)
{
  return (1 - alpha) * x0 + alpha * x1;
}

// typename std::enable_if<, T>::type* = nullptr

/**
 * @brief Check if a matrix is positive definite via Cholesky decomposition (LLT)
 *
 * @param A matrix
 * @return true if matrix is PD (Positive Definite)
 * @return false otherwise
 */
inline bool isPD(const Eigen::MatrixXd& A)
{
  const Eigen::LLT<Eigen::MatrixXd> llt(A);
  if (!A.isApprox(A.transpose()) || llt.info() == Eigen::NumericalIssue)
  {
    return false;
  }
  return true;
}

/**
 * @brief Check if a matrix is semi positive definite via Cholesky decomposition (LLT)
 *
 * @param A matrix
 * @return true if matrix is SPD (Semi Positive Definite)
 * @return false otherwise
 */
inline bool isSPD(const Eigen::MatrixXd& A)
{
  const auto ldlt = A.selfadjointView<Eigen::Upper>().ldlt();
  if (!A.isApprox(A.transpose()) || ldlt.info() == Eigen::NumericalIssue || !ldlt.isPositive())
  {
    return false;
  }
  return true;
}

/**
 * @brief Stream a vector or an array
 * @tparam T type of data to be streamed
 * @param stream (reference to std::ostream)
 * @param x data to be streamed (const reference to T)
 */
template <typename T,
          typename std::enable_if<std::is_same_v<T, std::vector<typename T::value_type>> ||
                                      std::is_same_v<T, std::array<typename T::value_type, std::tuple_size<T>::value>>,
                                  T>::type* = nullptr>
std::ostream& operator<<(std::ostream& stream, const T& v)
{
  // Beginning bracket
  stream << "[";

  // Copy element of vector into output stream
  std::copy(v.begin(), v.end() - 1, std::ostream_iterator<typename T::value_type>(stream, ", "));

  // Last element and end bracket
  stream << *(v.end() - 1) << "]";

  return stream;
}

std::vector<size_t> randperm(size_t const num_samples, const size_t max_range);
std::vector<size_t> randperm(size_t const num_samples, const size_t max_range, std::mt19937& gen);

double roundn(double const num, size_t const digits);
float roundn(float const num, size_t const digits);

}  // namespace uwb_init

#endif  // UTILS_HPP_
