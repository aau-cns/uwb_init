// Copyright (C) 2021 Martin Scheiber, Control of Networked Systems, University of Klagenfurt, Austria.
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
// You can contact the authors at <martin.scheiber@aau.at> and <giulio.delama@aau.at>

#ifndef UWB_INIT_UWB_OPTIONS_HPP_
#define UWB_INIT_UWB_OPTIONS_HPP_

#include <Eigen/Eigen>

namespace uwb_init
{
///
/// \brief The UwbInitOptions struct is an object containing all 'static' parameters used.
///
struct UwbInitOptions
{
  ///
  /// \brief The InitMethod enum describes the method used for initialization
  ///
  enum class InitMethod
  {
    SINGLE,  //!< use only one measurement to construct LLS matrix
    DOUBLE,  //!< use a pair of measurements to construct LLS matrix
  };

  ///
  /// \brief The InitVariables enum describes the type of variables to initialize
  ///
  enum class InitVariables
  {
    NO_BIAS,     //!< use no bias for initialization, i.e. position only
    CONST_BIAS,  //!< only use constant bias and position in initialization
  };

  // WRAPPER AND CALIBRATION ==================================================

  /// translational offset of the UWB module w.r.t. the IMU (or body frame) in meter
  Eigen::Vector3d p_ItoU{ Eigen::VectorXd::Zero(3) };

  /// determines the method to use for initialization \see UwbInit::UwbInitOptions::InitMethod
  InitMethod init_method{ InitMethod::SINGLE };

  /// determines the variables to initialize in initialization routine \see UwbInit::UwbInitOptions::InitVariables
  InitVariables init_variables{ InitVariables::NO_BIAS };

  // NONLINEAR OPTIMIZATION ==================================================

  /// flag to determine if nonlinear optimization should be performed
  bool b_nonlin_opt{ true };

  /// norm of step ( theta(k+1) = theta(k) + zeta(i)*d_theta )
  Eigen::VectorXd step_vec{ Eigen::VectorXd::LinSpaced(10, 1, 100) };

  /// stopping condition for norm of step
  double step_cond{ 1e-4 };

  /// stopping condition for residual
  double res_cond{ 1e-2 };

  /// stopping condition for maximum number of iterations
  uint max_iter{ 100000 };

  // GET INFO ================================================================

  /// get initialization method
  inline std::string InitMethod()
  {
    switch (init_method)
    {
      case InitMethod::SINGLE:
        return "InitMethod::SINGLE";
      case InitMethod::DOUBLE:
        return "InitMethod::DOUBLE";
      default:
        return "InitMethod::UNDEFINED";
    }
  }

  /// get initialization variables
  inline std::string InitVariables()
  {
    switch (init_variables)
    {
      case InitVariables::NO_BIAS:
        return "InitVariables::NO_BIAS";
      case InitVariables::CONST_BIAS:
        return "InitVariables::CONST_BIAS";
      default:
        return "InitVariables::UNDEFINED";
    }
  }

};  // struct UwbInitOptions
}  // namespace uwb_init

#endif  // UWB_INIT_UWB_OPTIONS_HPP_
