// Copyright (C) 2022 Alessandro Fornasier,
// Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.
//
// You can contact the author at <alessandro.fornasier@aau.at>

#include <ros/ros.h>
#include <Eigen/Eigen>

#include "uwb_init_ros.hpp"

// Main function
int main(int argc, char** argv)
{
  // Launch ros node
  ros::init(argc, argv, "uwb_init_ros");
  ros::NodeHandle nh("~");

  // Parameters from launchfile
  std::string param;

  // Check existance of parameter
  if (!nh.getParam("param", param))
  {
    std::cout << std::endl;
    ROS_ERROR("No param defined");
    std::exit(EXIT_FAILURE);
  }

  // Instanciate UwbInitRos
  UwbInitRos UwbInitRos(nh);

  // ROS Spin
  ros::spin();

  // Done!
  std::cout << std::endl;
  ROS_INFO("Done!");
  return EXIT_SUCCESS;
}
