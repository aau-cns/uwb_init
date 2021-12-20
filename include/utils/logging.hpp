// Copyright (C) 2021 Martin Scheiber, Control of Networked Systems, University of Klagenfurt, Austria.
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
// You can contact the author at <martin.scheiber@aau.at>

#ifndef UAV_INIT_LOGGING_HPP_
#define UAV_INIT_LOGGING_HPP_

#include <ros/ros.h>

namespace uav_init
{
#ifdef FULL_VERBOSE
#define INIT_PRINT_STREAM(args) ROS_INFO_STREAM(args)
#else  // FULL_VERBOSE
#define INIT_PRINT_STREAM(args) ROS_DEBUG_STREAM(args)
#endif  // FULL_VERBOSE

#if NDEBUG
#define INIT_DEBUG_STREAM(args) ROS_DEBUG_STREAM("uav_init == " << args)
#define INIT_INFO_STREAM(args) INIT_PRINT_STREAM("uav_init == " << args)
#define INIT_WARN_STREAM(args) ROS_WARN_STREAM("uav_init == " << args)
#define INIT_ERROR_STREAM(args) ROS_ERROR_STREAM("uav_init == " << args)
#else
#define INIT_DEBUG_STREAM(args) ROS_DEBUG_STREAM("uav_init == " << args)
#define INIT_INFO_STREAM(args) ROS_INFO_STREAM("uav_init == " << args)
#define INIT_WARN_STREAM(args) ROS_WARN_STREAM("uav_init == " << args)
#define INIT_ERROR_STREAM(args) ROS_ERROR_STREAM("uav_init == " << args)
#endif

static inline void print_all_topics()
{
  //  print published/subscribed topics
  ros::V_string topics;
  std::string nodeName = ros::this_node::getName();
  std::string topicsStr = "rosnode: " + nodeName;

  // subscribed topics
  topicsStr += ":\n\tsubscribed to topics:\n";
  ros::this_node::getSubscribedTopics(topics);
  for (unsigned int i = 0; i < topics.size(); i++)
    topicsStr += ("\t\t" + topics.at(i) + "\n");

  // advertised topics
  topicsStr += "\tadvertised topics:\n";
  ros::this_node::getAdvertisedTopics(topics);
  for (unsigned int i = 0; i < topics.size(); i++)
    topicsStr += ("\t\t" + topics.at(i) + "\n");

  INIT_PRINT_STREAM("" << topicsStr);
}

}  // namespace uav_init

#endif  // UAV_INIT_LOGGING_HPP_
