/******************************************************************************
 * FILENAME:     ROSNode.hpp
 * PURPOSE:      %{Cpp:License:ClassName}
 * AUTHOR:       jungr
 * MAIL:         roland.jung@ieee.org
 * VERSION:      v0.0.1
 * CREATION:     14.08.2023
 *
 *  Copyright (C) 2023
 *  All rights reserved. See the LICENSE file for details.
 ******************************************************************************/
#include "uwb_init_trigger/ROSnode.hpp"

typedef uwb_init_trigger::ROSnode nodeType;

/**
 * The main function of the
 *
 * @param[in] argc argument count
 * @param[in] argv char* to argument list
 *
 */
int main(int argc, char **argv)
{
  std::string name(argv[0]);
  ROS_INFO("%s started", name.c_str());
  ros::init(argc, argv, name.c_str());

  ros::NodeHandle nh("~");
  nodeType nodeType(nh);
  nodeType.run();

  ROS_INFO("terminated");
}
