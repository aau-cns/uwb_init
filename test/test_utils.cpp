// Copyright (C) 202w Giulio Delama, Alessandro Fornasier
// Control of Networked Systems, Universitaet Klagenfurt, Austria
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
// You can contact the author at <giulio.delama@aau.at> and
// <alessandro.fornasier@aau.at>

#include <gtest/gtest.h>
#include <iostream>

#include "uwb_init.hpp"

using namespace uwb_init;

template <typename T>
requires(std::derived_from<T, Eigen::MatrixBase<T>>) void EXPECT_EIGEN_EQ(T const& obj, T val, double eps)
{
  double norm_diff = (obj - val).norm();
  EXPECT_LT(norm_diff, eps);
}

double eps = 1e-4;

TEST(utils, lerp)
{
  // Test utils::lerp() with floating - point values
  EXPECT_NEAR(lerp(49.0, 50.0, 0.0), 49.0, eps);
  EXPECT_NEAR(lerp(49.0, 50.0, 0.63), 49.63, eps);
  EXPECT_NEAR(lerp(49.0, 50.0, 1.0), 50.0, eps);

  // Test utils::lerp() with Eigen::vectorXd values
  EXPECT_EIGEN_EQ(lerp(Eigen::Vector3d(49, 49, 49), Eigen::Vector3d(50, 50, 50), 0.0), Eigen::Vector3d(49, 49, 49),
                  eps);
  EXPECT_EIGEN_EQ(lerp(Eigen::Vector3d(49, 49, 49), Eigen::Vector3d(50, 50, 50), 0.63),
                  Eigen::Vector3d(49.63, 49.63, 49.63), eps);
  EXPECT_EIGEN_EQ(lerp(Eigen::Vector3d(49, 49, 49), Eigen::Vector3d(50, 50, 50), 1.0), Eigen::Vector3d(50, 50, 50),
                  eps);
}

TEST(utils, isxpd)
{
  Eigen::Matrix2d pd{ { 5, 0 }, { 0, 3 } };
  Eigen::Matrix2d spd{ { 5, 0 }, { 0, 0 } };
  Eigen::Matrix2d nd{ { -5, 0 }, { 0, -3 } };

  // Test utils::isPD()
  EXPECT_TRUE(isPD(pd));
  EXPECT_FALSE(isPD(spd));
  EXPECT_FALSE(isPD(nd));

  // Test utils::isSPD()
  EXPECT_TRUE(isSPD(pd));
  EXPECT_TRUE(isSPD(spd));
  EXPECT_FALSE(isSPD(nd));
}

TEST(TimedBuffer, get)
{
  PositionBuffer pos;
  for (uint i = 0; i < 100; ++i)
  {
    double val = static_cast<double>(i);
    Eigen::Vector3d position(val, val, val);
    double t = 0.01 * val;
    pos.push_back(t, position);
  }

  // Test TimedBuffer::get_closest()
  EXPECT_EIGEN_EQ(pos.get_closest(0.498), Eigen::Vector3d(50, 50, 50), eps);
  EXPECT_EIGEN_EQ(pos.get_closest(0.492), Eigen::Vector3d(49, 49, 49), eps);
  EXPECT_EIGEN_EQ(pos.get_at_timestamp(0.49), Eigen::Vector3d(49, 49, 49), eps);
  EXPECT_EIGEN_EQ(pos.get_at_timestamp(0.49572), Eigen::Vector3d(49.572, 49.572, 49.572), eps);
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}