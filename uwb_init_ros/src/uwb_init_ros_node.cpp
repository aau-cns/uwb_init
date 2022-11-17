// Copyright (C) 2022 Alessandro Fornasier, Giulio Delama
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
// You can contact the author at <alessandro.fornasier@aau.at> and
// <giulio.delama@aau.at>

#include <ros/ros.h>
#include <Eigen/Eigen>
#include "uwb_init_ros.hpp"

// Main function
int main(int argc, char** argv)
{
  // Launch ros node
  ros::init(argc, argv, "uwb_init_ros");
  ros::NodeHandle nh("~");

  // Instantiate options
  uwb_init_ros::UwbInitRosOptions opts;

  // Get topics to subscribe to from parameter server
  if (!nh.getParam("estimated_pose_topic", opts.estimated_pose_topic_))
  {
    ROS_ERROR("Missing estimated_pose_topic parameter");
    EXIT_FAILURE;
  }
  if (!nh.getParam("uwb_range_topic", opts.uwb_range_topic_))
  {
    ROS_ERROR("Missing uwb_range_topic parameter");
    EXIT_FAILURE;
  }

  // Get services names
  if (!nh.getParam("initialization_service", opts.service_init_))
  {
    ROS_ERROR("Missing initialization_service parameter");
    EXIT_FAILURE;
  }

  // Get init options from parameter server
  std::string method, bias_type;
  nh.param<std::string>("method", method, "double");
  nh.param<std::string>("bias_type", bias_type, "constant");

  if (method == "single")
  {
    opts.init_options_.init_method_ = uwb_init::InitMethod::SINGLE;
  }
  else if (method == "double")
  {
    opts.init_options_.init_method_ = uwb_init::InitMethod::DOUBLE;
  }
  else
  {
    ROS_ERROR("Invalidmethod! Please use single or double");
    EXIT_FAILURE;
  }

  if (bias_type == "unbiased")
  {
    opts.init_options_.bias_type_ = uwb_init::BiasType::NO_BIAS;
  }
  else if (bias_type == "constant")
  {
    opts.init_options_.bias_type_ = uwb_init::BiasType::CONST_BIAS;
  }
  else
  {
    ROS_ERROR("Invalid bias type! Please use unbiased or constant");
    EXIT_FAILURE;
  }

  // Get LS solver options from parameter server
  nh.param<double>("position_std", opts.ls_solver_options_.sigma_pos_, opts.ls_solver_options_.sigma_pos_);
  nh.param<double>("uwb_range_std", opts.ls_solver_options_.sigma_meas_, opts.ls_solver_options_.sigma_meas_);

  // Get NLS solver options from parameter server
  int max_iter;
  nh.param<double>("step_norm_stop_condition", opts.nls_solver_options_.step_cond_,
                   opts.nls_solver_options_.step_cond_);
  nh.param<double>("residual_mse_stop_condition", opts.nls_solver_options_.res_cond_,
                   opts.nls_solver_options_.res_cond_);
  nh.param<int>("max_iterations", max_iter, static_cast<int>(opts.nls_solver_options_.max_iter_));
  opts.nls_solver_options_.max_iter_ = static_cast<uint>(max_iter);

  std::vector<double> step_vec;
  std::vector<double> step_vec_default = { 0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0 };
  nh.param<std::vector<double>>("step_vec", step_vec, step_vec_default);
  opts.nls_solver_options_.step_vec_ =
      Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(step_vec.data(), step_vec.size());

  // Get logger level from parameter server
  std::string logger_level;
  nh.param<std::string>("logger_level", logger_level, "info");

  if (logger_level == "full")
  {
    opts.level_ = uwb_init::LoggerLevel::FULL;
    ROS_INFO("Logger level: full (DEBUG MODE)");
  }
  else if (logger_level == "info")
  {
    opts.level_ = uwb_init::LoggerLevel::INFO;
    ROS_INFO("Logger level: info (NORMAL MODE)");
  }
  else if (logger_level == "warning")
  {
    opts.level_ = uwb_init::LoggerLevel::WARN;
    ROS_INFO("Logger level: warnings and errors only");
  }
  else if (logger_level == "error")
  {
    opts.level_ = uwb_init::LoggerLevel::ERR;
    ROS_INFO("Logger level: errors only");
  }
  else if (logger_level == "inactive")
  {
    opts.level_ = uwb_init::LoggerLevel::INACTIVE;
    ROS_INFO("Logger level: inactive");
  }
  else
  {
    ROS_ERROR("Invalid logger level! Please use full, info, warning, error or inactive");
    EXIT_FAILURE;
  }

  // Get calibration UWB tag -> IMU
  std::vector<double> p_ItoU;
  std::vector<double> p_ItoU_default = { 0.0, 0.0, 0.0 };
  nh.param<std::vector<double>>("p_ItoU", p_ItoU, p_ItoU_default);
  Eigen::Vector3d p_UinI(p_ItoU.data());
  opts.p_UinI_ = p_UinI;
  ROS_INFO_STREAM("Calibration p_UinI = " << p_UinI.transpose());

  // Instanciate UwbInitRos
  uwb_init_ros::UwbInitRos UwbInitRos(nh, opts);

  // ROS Spin
  ros::spin();

  // Done!
  ROS_INFO("Done!");
  return EXIT_SUCCESS;
}
