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

UwbInitRos::UwbInitRos(ros::NodeHandle& nh) : nh_(nh)
{
  // Subscribers

  // Print topics where we are subscribing to
  ROS_INFO("Hi!");

  // Publishers

  // Print topics where we are publishing on
  ROS_INFO("Hi!");

  // Write your code here...
}
