// Copyright (C) 2022 Alessandro Fornasier, Giulio Delama.
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
// You can contact the authors at <alessandro.fornasier@aau.at> and
// <giulio.delama@aau.at>

#ifndef UWB_INIT_UWB_OPTIONS_HPP_
#define UWB_INIT_UWB_OPTIONS_HPP_

#include <Eigen/Eigen>

namespace uwb_init
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
/// \brief Return a string with the corresponding init method
///
constexpr const char* InitMethodString(InitMethod e)
{
  switch (e)
  {
    case InitMethod::SINGLE:
      return "InitMethod::SINGLE";
    case InitMethod::DOUBLE:
      return "InitMethod::DOUBLE";
    default:
      return "InitMethod::UNKNOWN";
  }
};

///
/// \brief The BiasType enum describes the type of biase used in the measurement model
///
enum class BiasType
{
  NO_BIAS,     //!< use no bias for initialization, i.e. position only
  CONST_BIAS,  //!< only use constant bias and position in initialization
  ALL_BIAS,    //!< use constant and distance bias in initialization
};

///
/// \brief Return a string with the corresponding bias type
///
constexpr const char* BiasTypeString(BiasType e)
{
  switch (e)
  {
    case BiasType::NO_BIAS:
      return "BiasType::NO_BIAS";
    case BiasType::CONST_BIAS:
      return "BiasType::CONST_BIAS";
    case BiasType::ALL_BIAS:
      return "BiasType::ALL_BIAS";
    default:
      return "";
  }
};

///
/// \brief The UwbInitOptions struct is an object containing all 'static' parameters used.
///
struct UwbInitOptions
{
  /// determines the method to use for initialization
  InitMethod init_method_;

  /// determines the type of bias used in the measurement model, to be estimated during the initialization routine
  BiasType bias_type_;

  /// constant bias prior covariance
  double const_bias_prior_cov_;

  /// distance bias prior covariance
  double dist_bias_prior_cov_;

  /// Minimum number of anchors to use for initialization
  uint min_num_anchors_;

  // Enable LS computation
  bool enable_ls_;

  // Compute covariance
  bool compute_covariance_;

  UwbInitOptions(const InitMethod& method, const BiasType& bias_type, const double const_bias_prior_cov,
                 const double dist_bias_prior_cov, const uint min_num_anchors, const bool enable_ls,
                 const bool compute_covariance)
    : init_method_(method)
    , bias_type_(bias_type)
    , const_bias_prior_cov_(const_bias_prior_cov)
    , dist_bias_prior_cov_(dist_bias_prior_cov)
    , min_num_anchors_(min_num_anchors)
    , enable_ls_(enable_ls)
    , compute_covariance_(compute_covariance)
  {
  }

};  // struct UwbInitOptions
}  // namespace uwb_init

#endif  // UWB_INIT_UWB_OPTIONS_HPP_
