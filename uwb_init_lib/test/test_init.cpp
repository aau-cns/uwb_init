﻿// Copyright (C) 202w Giulio Delama, Alessandro Fornasier
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
  UwbInitializer uwbInit;

  Eigen::Vector3d pos1(-0.693060316437549, 1.01533101774554, 5.18688703603644);
  Eigen::Vector3d pos2(1.45382231308303, -1.16674301765261, 4.76922688330564);
  Eigen::Vector3d pos3(-0.887035110721775, 1.79240626612294, 3.44519150152269);
  Eigen::Vector3d pos4(1.49836403594994, 2.83023565086719, 1.05941698299522);
  Eigen::Vector3d pos5(-0.597594224993961, -1.46142656169532, 1.42811451933108);
  Eigen::Vector3d pos6(-0.0424687941560576, 0.0246474864128924, 5.33853197631807);
  Eigen::Vector3d pos7(-1.18578530212440, 0.136551865521415, 0.196816729261889);
  Eigen::Vector3d pos8(-0.857750302293823, -2.40139677562462, 2.90896002179294);
  Eigen::Vector3d pos9(-1.64333667173238, -0.663158026573088, 0.993430776713046);
  Eigen::Vector3d pos10(-1.55134529868046, -0.710414381215266, 5.87619464407085);

  double t1 = 0.0;
  double t2 = 0.1;
  double t3 = 0.2;
  double t4 = 0.3;
  double t5 = 0.4;
  double t6 = 0.5;
  double t7 = 0.6;
  double t8 = 0.7;
  double t9 = 0.8;
  double t10 = 0.9;

  uwbInit.feed_pose(t1, pos1);
  uwbInit.feed_pose(t2, pos2);
  uwbInit.feed_pose(t3, pos3);
  uwbInit.feed_pose(t4, pos4);
  uwbInit.feed_pose(t5, pos5);
  uwbInit.feed_pose(t6, pos6);
  uwbInit.feed_pose(t7, pos7);
  uwbInit.feed_pose(t8, pos8);
  uwbInit.feed_pose(t9, pos9);
  uwbInit.feed_pose(t10, pos10);

  uwb_init::UwbData uwb11(1, 5.87758160026499, 1);
  uwb_init::UwbData uwb21(1, 5.81972230335446, 1);
  uwb_init::UwbData uwb31(1, 4.62565317281793, 1);
  uwb_init::UwbData uwb41(1, 3.88292489620468, 1);
  uwb_init::UwbData uwb51(1, 2.66885233230775, 1);
  uwb_init::UwbData uwb61(1, 5.71229564405359, 1);
  uwb_init::UwbData uwb71(1, 1.77304677176494, 1);
  uwb_init::UwbData uwb81(1, 4.42381408929201, 1);
  uwb_init::UwbData uwb91(1, 2.43424343857411, 1);
  uwb_init::UwbData uwb101(1, 6.71454916722409, 1);

  uwb_init::UwbData uwb12(1, 4.88724965925883, 2);
  uwb_init::UwbData uwb22(1, 6.87995736027867, 2);
  uwb_init::UwbData uwb32(1, 3.08445898375879, 2);
  uwb_init::UwbData uwb42(1, 3.46400123895514, 2);
  uwb_init::UwbData uwb52(1, 5.69315683673476, 2);
  uwb_init::UwbData uwb62(1, 5.63639445469960, 2);
  uwb_init::UwbData uwb72(1, 4.24972559288145, 2);
  uwb_init::UwbData uwb82(1, 6.58440908728125, 2);
  uwb_init::UwbData uwb92(1, 4.79230257877376, 2);
  uwb_init::UwbData uwb102(1, 6.73118632471322, 2);

  std::vector<uwb_init::UwbData> uwb1, uwb2, uwb3, uwb4, uwb5, uwb6, uwb7, uwb8, uwb9, uwb10;
  uwb1.push_back(uwb11);
  uwb1.push_back(uwb12);
  uwb2.push_back(uwb21);
  uwb2.push_back(uwb22);
  uwb3.push_back(uwb31);
  uwb3.push_back(uwb32);
  uwb4.push_back(uwb41);
  uwb4.push_back(uwb42);
  uwb5.push_back(uwb51);
  uwb5.push_back(uwb52);
  uwb6.push_back(uwb61);
  uwb6.push_back(uwb62);
  uwb7.push_back(uwb71);
  uwb7.push_back(uwb72);
  uwb8.push_back(uwb81);
  uwb8.push_back(uwb82);
  uwb9.push_back(uwb91);
  uwb9.push_back(uwb92);
  uwb10.push_back(uwb101);
  uwb10.push_back(uwb102);

  uwbInit.feed_uwb(t1, uwb1);
  uwbInit.feed_uwb(t2, uwb2);
  uwbInit.feed_uwb(t3, uwb3);
  uwbInit.feed_uwb(t4, uwb4);
  uwbInit.feed_uwb(t5, uwb5);
  uwbInit.feed_uwb(t6, uwb6);
  uwbInit.feed_uwb(t7, uwb7);
  uwbInit.feed_uwb(t8, uwb8);
  uwbInit.feed_uwb(t9, uwb9);
  uwbInit.feed_uwb(t10, uwb10);

  uwbInit.init_anchors();
  /// Matlab results for comparison:
  /// Anchor[1]: p_AinG = -0.0360222839749992, 0.00666158295331482, 0.0148258701503742
  /// Anchor[2]: p_AinG = -1.13714438550190, 3.37236670313509, 1.67803660542359
  /// Gamma[1] = 0.556469019410618
  /// Gamma[2] = 0.755554156299525

  uwbInit.refine_anchors();
  /// Matlab results for comparison:
  /// Anchor[1]: p_AinG = -0.0380870806986696, -2.37178679088623e-05, -0.0252082955931267
  /// Anchor[2]: p_AinG = -1.16075065885813, 3.38068568736152, 1.70699373711279
  /// Beta[1] = 1.00255411130761
  /// Beta[2] = 1.02633647024746
  /// Gamma[1] = 0.523334067226774
  /// Gamma[2] = 0.591954806048058

  /// Real values:
  /// Anchor[1]: p_AinG = 0, 0, 0
  /// Anchor[2]: p_AinG = -1, 3.50000000000000, 1.50000000000000
  /// Beta = 1.02
  /// Gamma = 0.5
  /// Measurement noise = 0.1
  /// Position noise = 0.03

  return 0;
}
