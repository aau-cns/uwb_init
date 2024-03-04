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

#ifndef UWB_INIT_LS_OPTIONS_HPP_
#define UWB_INIT_LS_OPTIONS_HPP_

#include "options/BiasType.hpp"
#include "options/RANSAC_Options.hpp"
namespace uwb_init
{
///
/// \brief The LsSolverOptions struct is an object containing all 'static' parameters used
/// for the linear least squares problem solver.
///
struct LsSolverOptions
{
  /// position uncertainty (default suggested value: 0.03)
  double sigma_pos_;

  /// uwb uncertainty (default suggested value: 0.1)
  double sigma_meas_;

  /// check covariancee is SPD
  bool check_cov_;

  /// use RANSAC
  bool use_RANSAC_;

  /// ranging bias types to be solved for (none, gamma, gamma+beta)
  BiasType bias_type_;

  RANSAC_Options ransac_opts_;

  LsSolverOptions(const double sigma_pos=0.03,
                  const double sigma_meas=0.1,
                  const bool check_cov=true,
                  const bool use_RANSAC = true,
                  const BiasType bias_type = BiasType::ALL_BIAS,
                  const RANSAC_Options ransac_opts = RANSAC_Options(0.99, 10, 0.15))
    : sigma_pos_(sigma_pos)
    , sigma_meas_(sigma_meas)
    , check_cov_(check_cov)
    , use_RANSAC_(use_RANSAC)
    , bias_type_(bias_type)
    , ransac_opts_(ransac_opts)
  {
  }

};  // namespace uwb_init
}  // namespace uwb_init

#endif  // UWB_INIT_LS_OPTIONS_HPP_
