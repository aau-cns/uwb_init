// Copyright (C) 2022 Giulio Delama.
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
// You can contact the author at <giulio.delama@aau.at>

#ifndef UWB_INIT_NLS_OPTIONS_HPP_
#define UWB_INIT_NLS_OPTIONS_HPP_

#include <Eigen/Eigen>

namespace uwb_init
{
///
/// \brief The NlsSolverOptions struct is an object containing all 'static' parameters used
/// for the nonlinear least squares problem solver.
///
struct NlsSolverOptions
{
  /// Lambda parameter for Levenberg-Marquardt algorithm (suggested default value: 1e-2)
  double lambda_;

  /// Lambda factor for Levenberg-Marquardt algorithm (suggested default value: 10)
  double lambda_factor_;

  /// Stopping condition for step size (suggested default value: 1e-6)
  double step_cond_;

  /// Stoping condition for residual (Mean Squared Error) (suggested default value: 1e-6)
  double res_cond_;

  /// Stopping condition for maximum number of iterations (suggested defualt value: 1e3)
  uint max_iter_;

  /// Check covariance is SPD
  bool check_cov_;

  NlsSolverOptions(const double& lambda, const double& lambda_factor, const double& step_cond, const double& res_cond,
                   const uint& max_iter, const bool check_cov)
    : lambda_(lambda)
    , lambda_factor_(lambda_factor)
    , step_cond_(step_cond)
    , res_cond_(res_cond)
    , max_iter_(max_iter)
    , check_cov_(check_cov)
  {
  }

};  // struct NlsSolverOptions
}  // namespace uwb_init

#endif  // UWB_INIT_NLS_OPTIONS_HPP_
