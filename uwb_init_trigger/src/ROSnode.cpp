#include "std_srvs/Empty.h"
#include <string>
#include <uwb_init_trigger/ROSnode.hpp>
namespace uwb_init_trigger {

ROSnode::ROSnode(ros::NodeHandle& _nh) : INode(_nh), m_bVerbose(true) { mbSpinningProcess = false; }

ROSnode::~ROSnode()
{

}

void ROSnode::cb_anchor_pos(uwb_msgs::UwbAnchorArrayStampedConstPtr pMsg) { mbHasNewMessage = true; }

void ROSnode::cb_start(std_msgs::BoolConstPtr pMsg) {
  std_srvs::Empty req;
  if (!m_clt_start.call(req)) {
    ROS_WARN("uwb_init_trigger: call uwb_init->start failed");
  } else {
    ROS_WARN("uwb_init_trigger: call uwb_init->start success");
  }
}

void ROSnode::cb_init(std_msgs::BoolConstPtr pMsg) {
  std_srvs::Empty req;
  if (!m_clt_init.call(req)) {
    ROS_WARN("uwb_init_trigger: call uwb_init->init failed");
  } else {
    ROS_WARN("uwb_init_trigger: call uwb_init->init success");
  }
}

bool ROSnode::update_()
{
  ROS_INFO("Reconfigure: start request %s", mDynConfig.uwb_init_start ? "True" : "False");

  std_srvs::Empty req;
  if (mDynConfig.uwb_init_reset) {
    if (!m_clt_reset.call(req)) {
      ROS_WARN("uwb_init_trigger: call uwb_init->reset failed");
    }
    mDynConfig.uwb_init_reset = false;
  }

  if (mDynConfig.uwb_init_start) {
    if (!m_clt_start.call(req)) {
      ROS_WARN("uwb_init_trigger: call uwb_init->start failed");
    }
    mDynConfig.uwb_init_start = false;
  }
  if (mDynConfig.uwb_init_initialize) {
    if (!m_clt_init.call(req)) {
      ROS_WARN("uwb_init_trigger: call uwb_init->initialize failed");
    }
    mDynConfig.uwb_init_initialize = false;
  }
  if (mDynConfig.uwb_init_compute_wps) {
    if (!m_clt_wps.call(req)) {
      ROS_WARN("uwb_init_trigger: call uwb_init->wps failed");
    }
    mDynConfig.uwb_init_compute_wps = false;
  }
  if (mDynConfig.uwb_init_refine) {
    if (!m_clt_wps.call(req)) {
      ROS_WARN("uwb_init_trigger: call uwb_init->refine failed");
    }
    mDynConfig.uwb_init_refine = false;
  }

  return true;
}

// 1)
void ROSnode::init_topics() {
  size_t const queue_size = 10;
  m_sub_anchor_pos = mNh.subscribe("/uwb_anchor_pos", queue_size, &ROSnode::cb_anchor_pos, this);
  m_sub_start= mNh.subscribe("/start", queue_size, &ROSnode::cb_start, this);
  m_sub_init= mNh.subscribe("/init", queue_size, &ROSnode::cb_init, this);
}

// 2)
void ROSnode::load_config()
{
  mNh.param<bool>("bVerbose", m_bVerbose, true);
  std::string text = "";
  mNh.param<std::string>("strExample_text", text, text);
  ROS_INFO("%s", text.c_str());

  // Get services names
  mNh.param<std::string>("start_service", opts.service_start_, opts.service_start_);
  mNh.param<std::string>("reset_service", opts.service_reset_, opts.service_reset_);
  mNh.param<std::string>("initialization_service", opts.service_init_, opts.service_init_);
  mNh.param<std::string>("planner_service", opts.service_wps_, opts.service_wps_);
  mNh.param<std::string>("refine_service", opts.service_refine_, opts.service_refine_);

  mNh.param<bool>("auto_trigger", opts.m_bAutoTrigger, opts.m_bAutoTrigger);
  mNh.param<double>("timeout_rec", opts.timeout_start_rec, opts.timeout_start_rec);
  mNh.param<double>("timeout_cal", opts.timeout_start_cal, opts.timeout_start_cal);

  // calc absolute timeout from time_started:
  opts.timeout_start_cal += opts.timeout_start_rec;

  // we need a spinning process to check the timer
  mbSpinningProcess = opts.m_bAutoTrigger;

  if (opts.m_bAutoTrigger) {
    ROS_INFO(" auto trigger enables: rec starts in %f [s]; cal in %f [s]", opts.timeout_start_rec,
             opts.timeout_start_cal);
  }

  m_clt_start = mNh.serviceClient<std_srvs::Empty>(opts.service_start_);
  m_clt_reset = mNh.serviceClient<std_srvs::Empty>(opts.service_reset_);
  m_clt_init = mNh.serviceClient<std_srvs::Empty>(opts.service_init_);
  m_clt_wps = mNh.serviceClient<std_srvs::Empty>(opts.service_wps_);
  m_clt_refine = mNh.serviceClient<std_srvs::Empty>(opts.service_refine_);

  time_started = ros::Time::now();
}

// 3)
void ROSnode::init()
{

}

// 4) is called when mbHasNewMessage == true or mbSpinningProcess == true
void ROSnode::process()
{
  if (opts.m_bAutoTrigger) {
    ros::Time processTimeNow = ros::Time::now();

    if (opts.timeout_start_rec > 0) {
      ros::Duration processTime = processTimeNow - time_started;
      if (processTime.toSec() > opts.timeout_start_rec) {
        ROS_INFO("Trigger recording");
        std_srvs::Empty req;
        if (!m_clt_reset.call(req)) {
          ROS_WARN("uwb_init_trigger: call uwb_init->reset failed");
        }
        if (!m_clt_start.call(req)) {
          ROS_WARN("uwb_init_trigger: call uwb_init->start failed");
        }
        opts.timeout_start_rec = 0;
      }
    }

    if (opts.timeout_start_cal > 0) {
      ros::Duration processTime = processTimeNow - time_started;
      if (processTime.toSec() > opts.timeout_start_cal) {
        ROS_INFO("Trigger calibration");
        std_srvs::Empty req;
        if (!m_clt_init.call(req)) {
          ROS_WARN("uwb_init_trigger: call uwb_init->initialize failed");
        }

        // wait a bit: for the other process to complete, but is thread safe
        ros::Time::sleepUntil(processTimeNow + ros::Duration(1.0));
        if (!m_clt_reset.call(req)) {
          ROS_WARN("uwb_init_trigger: call uwb_init->reset failed");
        }
        opts.timeout_start_cal = 0;
      }
    }
  }
}

}  // namespace uwb_init_trigger
