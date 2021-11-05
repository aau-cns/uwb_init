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


#ifndef uwb_init_cpp_H
#define uwb_init_cpp_H


#include <ros/ros.h>
#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include "utils/colors.hpp"


class UwbInitCpp {


  public:


  /**
   * @brief Constructor
   * @param Ros NodeHandle
   */
  UwbInitCpp(ros::NodeHandle &nh);


  private:


  /**
   * @brief Callbacks
   * @param Message const pointer
   */
  void callback_posestamped(const geometry_msgs::PoseStamped::ConstPtr& msg);


  /// Ros node handler
  ros::NodeHandle nh_;


  /// Subscribers
  ros::Subscriber sub_posestamped;


  /// Publishers
  

  /// Messages
  

};

#endif  // uwb_init_cpp_H
