// Copyright (C) 2021 Martin Scheiber,
// Control of Networked Systems, Universitaet Klagenfurt, Austria
//
// You can contact the author at <martin.scheiber@aau.at>
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

#include "uwb_wrapper.hpp"

#include <ros/ros.h>
#include <Eigen/Eigen>

#include "utils/colors.hpp"

namespace uav_init
{
UwbInitWrapper::UwbInitWrapper(ros::NodeHandle& nh) : nh_(nh)
{
  // Subscribers
  sub_posestamped = nh.subscribe("/ov_msckf/poseimu", 1, &UwbInitWrapper::callback_posestamped, this);

  // Print topics where we are subscribing to
  std::cout << std::endl;
  std::cout << "Subscribing: " << sub_posestamped.getTopic().c_str() << std::endl;

  // Publishers

  // Print topics where we are publishing on
  std::cout << std::endl;

  // Write your code here...
}

void UwbInitWrapper::callback_posestamped(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  // Write your code here ...
}

}  // namespace uav_init
