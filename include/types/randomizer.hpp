// Copyright (C) 2021 Martin Scheiber, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <martin.scheiber@aau.at>

#ifndef RANDOMIZER_HPP_
#define RANDOMIZER_HPP_

#include <chrono>
#include <random>

namespace uav_init
{
///
/// \brief The Randomizer class is an object that provides a uniformly distributed random number within an interval.
///
/// \see https://stackoverflow.com/a/13445752
/// \see https://stackoverflow.com/a/13446015
///
class Randomizer
{
private:
  std::random_device dev_;
  std::mt19937::result_type seed_{ dev_() };
  std::mt19937 gen_;
  std::uniform_int_distribution<std::mt19937::result_type> dist_;

  int range_min_{ 0 };
  int range_max_{ RAND_MAX };

  void init_seed()
  {
    // seed value is designed specifically to make initialization
    // parameters of std::mt19937 (instance of std::mersenne_twister_engine<>)
    // different across executions of application
    seed_ = static_cast<std::mt19937::result_type>(
        dev_() ^ ((std::mt19937::result_type)std::chrono::duration_cast<std::chrono::seconds>(
                      std::chrono::system_clock::now().time_since_epoch())
                      .count() +
                  (std::mt19937::result_type)std::chrono::duration_cast<std::chrono::microseconds>(
                      std::chrono::high_resolution_clock::now().time_since_epoch())
                      .count()));

    gen_ = std::mt19937(seed_);
    dist_ = std::uniform_int_distribution<std::mt19937::result_type>(range_min_, range_max_);
  }

public:
  Randomizer()
  {
    init_seed();
  };
  Randomizer(const int& range_min, const int& range_max) : range_min_(range_min), range_max_(range_max)
  {
    init_seed();
  };

  int get_randi()
  {
    return static_cast<int>(dist_(gen_));
  }
};  // class Randomizer
}  // namespace uav_init

#endif  // RANDOMIZER_HPP_
