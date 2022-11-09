// Copyright (C) 2022 Giulio Delamar, Control of Networked Systems,
// University of Klagenfurt, Austria.
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
/// \brief The UwbInitOptions struct is an object containing all 'static' parameters used
/// for the nonlinear least squares problem solver.
///
struct NlsSolverOptions
{
  /// norm of step ( theta(k+1) = theta(k) + zeta(i)*d_theta )
  Eigen::VectorXd step_vec{ Eigen::VectorXd::LinSpaced(10, 0.1, 1) };

  /// stopping condition for the relative norm of step
  double step_cond{ 1e-4 };

  /// stopping condition for residual (mean squared error r'*r/(m-p))
  double res_cond{ 1e-2 };

  /// stopping condition for maximum number of iterations
  uint max_iter{ 100000 };

};  // struct NlsSolverOptions
}  // namespace uwb_init

#endif  // UWB_INIT_NLS_OPTIONS_HPP_
