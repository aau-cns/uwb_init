/******************************************************************************
 * FILENAME:     RANSAC_Options.cpp
 * PURPOSE:      %{Cpp:License:ClassName}
 * AUTHOR:       jungr
 * MAIL:         roland.jung@ieee.org
 * VERSION:      v0.0.1
 * CREATION:     04.03.2024
 *
 *  Copyright (C) 2024
 *  All rights reserved. See the LICENSE file for details.
 ******************************************************************************/
#include <options/RANSAC_Options.hpp>


RANSAC_Options::RANSAC_Options(const uint n, const double p, const uint s, const double e, const double thres_num_std) :
  p(p), s(s), e(e), n(n), thres_num_std(thres_num_std)
{
  assert(e < 1.0 && e >= 0.0);
  assert(p < 1.0 && p >= 0.0);
}

RANSAC_Options::RANSAC_Options(const double p, const uint s, const double e) : p(p), s(s), e(e)
{
  assert(e < 1.0 && e >= 0.0);
  assert(p < 1.0 && p >= 0.0);
  n = RANSAC_Options::num_iterations(p,e,s);
}

uint RANSAC_Options::num_iterations(const double p, const double e, const unsigned int s)
{
  assert(e < 1.0 && e >= 0.0);
  assert(p < 1.0 && p >= 0.0);

  double e_pow = std::pow((1-e), double(s));
  return (uint) std::ceil(log(1.0-p)/log(1.0-e_pow));
}

size_t RANSAC_Options::num_samples_needed()
{
  return n*s;
}

std::string RANSAC_Options::str() {
  std::stringstream ss;
  ss << "propability to obtain inlier=" << p;
  ss << ", samples needed for model=" << s;
  ss << ", perc. outliers=" << e;
  ss << ", num iterations=" << n;
  ss << ", thres_num_std=" << thres_num_std;
  return ss.str();
}
