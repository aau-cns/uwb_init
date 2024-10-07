/******************************************************************************
 * FILENAME:     ROSnode.h
 * PURPOSE:      %{Cpp:License:ClassName}
 * AUTHOR:       jungr
 * MAIL:         roland.jung@ieee.org
 * VERSION:      v0.0.1
 * CREATION:     14.08.2023
 *
 *  Copyright (C) 2023
 *  All rights reserved. See the LICENSE file for details.
 ******************************************************************************/
#ifndef UWB_INIT_TRIGGER_ROSNODE_HPP
#define UWB_INIT_TRIGGER_ROSNODE_HPP
#include <vector>
#include <utility>
#include <string>
#include <memory>
#include <iostream>

#include <ros_common/INode.hpp>
#include <ros_common/ITROSdynamic.hpp>

#include <uwb_init_trigger/dyn_configConfig.h>
#include <uwb_msgs/UwbAnchorArrayStamped.h>
#include <std_msgs/Bool.h>

namespace uwb_init_trigger {

typedef uwb_init_trigger::dyn_configConfig Config;

struct options_t {
  std::string service_start_{"/uwb_init/start"};
  std::string service_reset_{"/uwb_init/reset"};
  std::string service_init_{"/uwb_init/initialize"};
  std::string service_wps_{"/uwb_init/compute_wps"};
  std::string service_refine_{"/uwb_init/refine"};
  bool m_bAutoTrigger{false};
  double timeout_start_rec{0.0};
  double timeout_start_cal{0.0};
};

class ROSnode : public ros_common::INode, public ros_common::ITROSdynamic<Config>
{
  public:
    ROSnode(ros::NodeHandle & _nh);
    ~ROSnode();

    void cb_anchor_pos(uwb_msgs::UwbAnchorArrayStampedConstPtr pMsg);
    void cb_start(std_msgs::BoolConstPtr pMsg);
    void cb_init(std_msgs::BoolConstPtr pMsg);

  private:
    // MEMBERS:
    bool m_bVerbose{false};
    bool m_bStarted{false};

    ros::Time time_node_started;
    ros::Time time_started;
    ros::Time time_calibrated;
    options_t opts;

    ros::Publisher m_pub_waypoint_fn;        /// publishes on "waypoint_fn"
    ros::Publisher m_pub_mission_request;    /// publishes on "mission_request"
    ros::Subscriber m_sub_mission_responce;  /// subscribes on "mission_response"
    ros::Subscriber m_sub_anchor_pos;        /// subscribes on "uwb_anchor_pos"

    ros::Subscriber m_sub_start;        /// subscribes on "start"
    ros::Subscriber m_sub_init;        /// subscribes on "init"


    /// Service Client
    ros::ServiceClient m_clt_start;
    ros::ServiceClient m_clt_reset;
    ros::ServiceClient m_clt_init;
    ros::ServiceClient m_clt_wps;
    ros::ServiceClient m_clt_refine;

    // DERIVED METHODS:
    bool update_() override final;  // ros_common::ITROSdynamic::update_
    void init_topics() override final;
    void load_config() override final;
    void init() override final;
    void process() override final;

};  // ROSnode
}  // namespace uwb_init_trigger

#endif // ROSNODE_HPP
