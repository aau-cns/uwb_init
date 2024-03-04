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

#include <Eigen/Dense>
#include <options/BiasType.hpp>
#include <options/RANSAC_Options.hpp>


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
  unsigned int max_iter_;

  /// Check covariance is SPD
  bool check_cov_;

  /// use RANSAC
  bool use_RANSAC_;

  /// position uncertainty (default suggested value: 0.03)
  double sigma_pos_;

  /// uwb uncertainty (default suggested value: 0.1)
  double sigma_meas_;

  /// ranging bias types to be solved for (none, gamma, gamma+beta)
  BiasType bias_type_;

  RANSAC_Options ransac_opts_;

  NlsSolverOptions(const double lambda=1e-2, const double lambda_factor=10.0, const double step_cond=1e-6,
                   const double res_cond=1e-6, const unsigned int max_iter=1e3, const bool check_cov=true,
                   const bool use_RANSAC = true, const double sigma_pos=0.03, const double sigma_meas=0.1,
                   const BiasType bias_type = BiasType::ALL_BIAS, const RANSAC_Options ransac_opts = RANSAC_Options(0.99, 10, 0.15))
    : lambda_(lambda)
    , lambda_factor_(lambda_factor)
    , step_cond_(step_cond)
    , res_cond_(res_cond)
    , max_iter_(max_iter)
    , check_cov_(check_cov)
    , use_RANSAC_(use_RANSAC)
    ,sigma_pos_(sigma_pos)
    , sigma_meas_(sigma_meas)
    , bias_type_(bias_type)
    , ransac_opts_(ransac_opts)
  {
  }

};  // struct NlsSolverOptions
}  // namespace uwb_init

#endif  // UWB_INIT_NLS_OPTIONS_HPP_
