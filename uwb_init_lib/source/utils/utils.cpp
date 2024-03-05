/******************************************************************************
 * FILENAME:     utils.cpp
 * PURPOSE:      %{Cpp:License:ClassName}
 * AUTHOR:       jungr
 * MAIL:         roland.jung@ieee.org
 * VERSION:      v0.0.1
 * CREATION:     04.03.2024
 *
 *  Copyright (C) 2024
 *  All rights reserved. See the LICENSE file for details.
 ******************************************************************************/
#include <utils/utils.hpp>


std::vector<size_t> uwb_init::randperm(const size_t num_samples, size_t const max_range)
{
  if(num_samples < max_range)
  {
    std::vector<size_t> indices;
    indices.resize(max_range);
    for(size_t idx=0; idx < max_range; idx++) {
      indices[idx] = idx;
    }
    std::random_shuffle(indices.begin(), indices.end());
    return std::vector<size_t>(indices.begin(), indices.begin()+num_samples);
  }
  return std::vector<size_t>();
}

std::vector<size_t> uwb_init::randperm(const size_t num_samples, size_t const max_range, std::mt19937 &gen) {
  if(num_samples < max_range)
  {
    std::vector<size_t> indices;
    indices.resize(max_range);
    for(size_t idx=0; idx < max_range; idx++) {
      indices[idx] = idx;
    }
    std::shuffle(indices.begin(), indices.end(), gen);
    return std::vector<size_t>(indices.begin(), indices.begin()+num_samples);
  }
  else
  {
    std::vector<size_t> perm;
    perm.resize(num_samples);
    std::uniform_int_distribution<> distr(0, max_range); // define the range
    for(size_t iter= 0; iter < num_samples; iter++) {

      // find a unique random value:
      size_t rand_val = 0;
      do {
        rand_val = distr(gen);
      } while(std::find(perm.begin(), perm.end(), rand_val) != perm.end());
      perm[iter] = rand_val;
    }
    return perm;
  }
}

double uwb_init::roundn(const double num, const size_t digits) {
  double const scaling = double(std::pow( 10, digits ));
  return std::ceil( ( num * scaling) - double(0.499999999) ) / scaling;
}

float uwb_init::roundn(const float num, const size_t digits) {
  float const scaling = float(std::pow( 10, digits ));
  return std::ceil( ( num * scaling) - float(0.499999999) ) / scaling;
}
