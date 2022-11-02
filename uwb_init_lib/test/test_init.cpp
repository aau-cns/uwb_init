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
  // Test initialization
  UwbInitOptions options;
  UwbInitializer uwbInit(options);
  for (uint i = 0; i < 500; ++i)
  {
    double val = static_cast<double>(i);
    Eigen::Vector3d position(val, val, val);
    double t = 0.008 * val;
    uwbInit.feed_pose(t, position);
  }
  for (uint i = 0; i < 80; ++i)
  {
    double val = static_cast<double>(i);
    UwbData uwb1(1, val, 1);
    UwbData uwb2(1, val, 2);
    double t = 0.05 * val;
    std::vector<UwbData> uwb_meas = { uwb1, uwb2 };
    uwbInit.feed_uwb(t, uwb_meas);
  }

  std::cout << "Initialize Anchors..." << std::endl;
  uwbInit.init_anchors();

  std::cout << "Refine Anchors..." << std::endl;
  uwbInit.refine_anchors();

  return 0;
}
