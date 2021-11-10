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

#include <ros/ros.h>

#include <Eigen/Eigen>

#include "utils/colors.hpp"
#include "uwb_wrapper.hpp"

// Main function
int main(int argc, char** argv)
{
  // Launch ros node
  ros::init(argc, argv, "uwb_init_cpp");
  ros::NodeHandle nh("~");

#ifdef NDEBUG
  // nondebug
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) {
    ros::console::notifyLoggerLevelsChanged();
  }
#else
  // debug
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
    ros::console::notifyLoggerLevelsChanged();
  }
#endif

  // Instanciate UwbInitCpp
  uav_init::UwbInitWrapper UwbInitWrapper(nh);

  // ROS Spin
  double check_duration = 5.0;
  double t_last_check = ros::Time::now().toSec();
  while(ros::ok())
  {
    ros::spinOnce();
    if (ros::Time::now().toSec() - t_last_check >= check_duration)
    {
      t_last_check = ros::Time::now().toSec();
      UwbInitWrapper.perform_initialization();
    }
  }
//  ros::spin();

  // Done!
  std::cout << std::endl;
  ROS_INFO("Done!");
  return EXIT_SUCCESS;
}
