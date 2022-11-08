// Copyright (C) 2022 Giulio Delama, Alessandro Fornasier, Martin Scheiber
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
// You can contact the authors at <giulio.delama@aau.at>,
// <alessandro.fornasier@aau.at>, and <martin.scheiber@aau.at>

#include <ros/ros.h>

#include <Eigen/Eigen>

#include "utils/parse_ros.hpp"
#include "uwb_wrapper.hpp"

// Main function
int main(int argc, char** argv)
{
  // Launch ros node
  ros::init(argc, argv, "uwb_init_cpp");
  ros::NodeHandle nh("~");

  // // parse parameters
  // uwb_init_ros::UwbInitRosOptions params = uwb_init_ros::parse_ros_params(nh);

  // // Instanciate UwbInitCpp
  // uwb_init_ros::UwbInitWrapper UwbInitWrapper(nh, params);

  // ROS Spin
  ros::spin();

  // Done!
  std::cout << std::endl;
  INIT_INFO_STREAM("Done!");
  return EXIT_SUCCESS;
}
