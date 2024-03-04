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
#include <math.h>       /* log */
#include <Eigen/Eigen>

namespace uwb_init
{
///
/// \brief The InitMethod enum describes the method used for initialization
///
enum class InitMethod
{
  SINGLE = 0,  //!< use only one measurement to construct LLS matrix
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
}

///
/// \brief The BiasType enum describes the type of biase used in the measurement model
///
enum class BiasType
{
  NO_BIAS = 0,     //!< use no bias for initialization, i.e. position only
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
}


struct RANSAC_Options {
  double p = 0.99; // propability to obtain a inlier subset
  uint s = 10;  // samples needed for the model
  double e = 0.15;  // relative percentage of outliers
  uint n = 21; // number of iterations needed to achieve p ; %

  RANSAC_Options(uint const n=21, double const p = 0.99, uint const s = 10, double const e = 0.15) : p(p), s(s), e(e), n(n) {
    assert(e < 1.0 && e >= 0.0);
    assert(p < 1.0 && p >= 0.0);
  }
  RANSAC_Options(double const p, uint const s, double const e) : p(p), s(s), e(e)
  {
    assert(e < 1.0 && e >= 0.0);
    assert(p < 1.0 && p >= 0.0);
    n = RANSAC_Options::num_iterations(p,e,s);
  }

  static uint num_iterations(double const p, double const e, unsigned int const s)
  {
    assert(e < 1.0 && e >= 0.0);
    assert(p < 1.0 && p >= 0.0);

    double e_pow = std::pow((1-e), double(s));
    return (uint) std::ceil(log(1.0-p)/log(1.0-e_pow));
  }

  size_t num_samples_needed()
  {
    return n*s;
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

  RANSAC_Options RANSAC_cfg_;

  /// constant bias prior covariance
  double const_bias_prior_cov_;

  /// distance bias prior covariance
  double dist_bias_prior_cov_;

  /// Minimum number of anchors to use for initialization
  unsigned int min_num_anchors_;

  // Enable LS computation
  bool enable_ls_;

  // Compute covariance
  bool compute_covariance_;

  UwbInitOptions(const InitMethod& method,
                 const BiasType& bias_type,
                 RANSAC_Options const& RANSAC_cfg,
                 const double const_bias_prior_cov = 0.1,
                 const double dist_bias_prior_cov = 0.1,
                 const unsigned int min_num_anchors = 1,
                 const bool enable_ls = true,
                 const bool compute_covariance = true)
    : init_method_(method)
    , bias_type_(bias_type)
    , RANSAC_cfg_(RANSAC_cfg)
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
