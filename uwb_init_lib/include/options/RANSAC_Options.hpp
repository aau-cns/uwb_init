/******************************************************************************
 * FILENAME:     RANSAC_Options.hpp
 * PURPOSE:      %{Cpp:License:ClassName}
 * AUTHOR:       jungr
 * MAIL:         roland.jung@ieee.org
 * VERSION:      v0.0.1
 * CREATION:     04.03.2024
 *
 *  Copyright (C) 2024
 *  All rights reserved. See the LICENSE file for details.
 ******************************************************************************/
#ifndef UWB_INIT_RANSAC_OPTIONS_HPP
#define UWB_INIT_RANSAC_OPTIONS_HPP
#include <math.h>       /* log */
#include <Eigen/Dense>

namespace uwb_init
{

struct RANSAC_Options {
  double p = 0.99; // propability to obtain a inlier subset
  uint s = 10;  // samples needed for the model
  double e = 0.15;  // relative percentage of outliers
  uint n = 21; // number of iterations needed to achieve p ; %

  /// number of standard deviation used for outlier rejection
  double thres_num_std = 3.0;

  RANSAC_Options(uint const n=21, double const p = 0.99, uint const s = 10,
                 double const e = 0.15, const double thres_num_std = 3.0);
  RANSAC_Options(double const p, uint const s, double const e);

  static uint num_iterations(double const p, double const e, unsigned int const s);

  size_t num_samples_needed();

  std::string str();
};

}
#endif // RANSAC_OPTIONS_HPP
