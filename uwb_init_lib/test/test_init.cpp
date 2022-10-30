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

#include <iostream>

#include "uwb_init.hpp"

using namespace uwb_init;

int main()
{
  // Test linear interpolation with floating-point values
  // std::cout << lerp(49.0, 50.0, 0.0) << std::endl;   // Should be 49
  // std::cout << lerp(49.0, 50.0, 0.63) << std::endl;  // Should be 49.63
  // std::cout << lerp(49.0, 50.0, 1.0) << std::endl;   // Should be 50

  // Test linear interpolation with Eigen::vectorXd values
  // std::cout << lerp(Eigen::Vector3d(49, 49, 49), Eigen::Vector3d(50, 50, 50), 0.0) << std::endl;   // Should be 49
  // std::cout << lerp(Eigen::Vector3d(49, 49, 49), Eigen::Vector3d(50, 50, 50), 0.63) << std::endl;  // Should be 49.63
  // std::cout << lerp(Eigen::Vector3d(49, 49, 49), Eigen::Vector3d(50, 50, 50), 1.0) << std::endl;   // Should be 50

  // Test TimedBuffer (PositionBuffer) functionalities
  PositionBuffer pos;
  for (uint i = 0; i < 100; ++i)
  {
    double val = static_cast<double>(i);
    Eigen::Vector3d position(val, val, val);
    double t = 0.01 * val;
    pos.push_back(t, position);
  }
  // std::cout << pos.get_closest(0.498) << std::endl;          // Should be 50
  // std::cout << pos.get_closest(0.492) << std::endl;          // Should be 49
  // std::cout << pos.get_at_timestamp(0.49) << std::endl;      // Should be 49
  // std::cout << pos.get_at_timestamp(0.49572) << std::endl;   // Should be 49.572

  return 0;
}
