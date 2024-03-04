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
#include <Eigen/Eigen>

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

  std::string str() {
    std::stringstream ss;
    ss << "propability to obtain inlier=" << p;
    ss << ", samples needed for model=" << s;
    ss << ", perc. outliers=" << e;
    ss << ", num iterations=" << n;
    return ss.str();
  }
};

#endif // RANSAC_OPTIONS_HPP
