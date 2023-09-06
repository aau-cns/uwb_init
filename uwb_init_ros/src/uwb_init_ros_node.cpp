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

#include "uwb_init_ros.hpp"

// Main function
int main(int argc, char** argv)
{
  // Launch ros node
  ros::init(argc, argv, "uwb_init_ros");
  ros::NodeHandle nh("~");

  // Instantiate options
  uwb_init_ros::UwbInitRosOptions opts;

  // at least one uwb topic needs to be provided:
  bool pose_topic_set = false;
  // Get topics to subscribe to from parameter server
  if (nh.getParam("estimated_pose_cov_topic", opts.estimated_pose_cov_topic_)) {
    pose_topic_set = true;
  }
  if (nh.getParam("estimated_pose_topic", opts.estimated_pose_topic_)) {
    pose_topic_set = true;
  }
  if (nh.getParam("estimated_transform_topic", opts.estimated_transform_topic_)) {
    pose_topic_set = true;
  }

  if (!pose_topic_set) {
    ROS_ERROR("Missing estimated_pose_topic parameter");
    std::exit(EXIT_FAILURE);
  }

  // at least one uwb topic needs to be provided:
  bool uwb_topic_set = false;
  if (nh.getParam("uwb_range_topic", opts.uwb_range_topic_))
  {
    uwb_topic_set = true;
  }
  if (nh.getParam("uwb_twr_topic", opts.uwb_twr_topic_))
  {
    uwb_topic_set = true;
  }

  if (!uwb_topic_set)
  {
    ROS_ERROR("Missing uwb_range_topic or uwb_twr_topic parameter");
    std::exit(EXIT_FAILURE);
  }

  // Get services names
  if (!nh.getParam("start_service", opts.service_start_))
  {
    ROS_ERROR("Missing start_service parameter");
    std::exit(EXIT_FAILURE);
  }

  if (!nh.getParam("reset_service", opts.service_reset_))
  {
    ROS_ERROR("Missing reset_service parameter");
    std::exit(EXIT_FAILURE);
  }

  if (!nh.getParam("initialization_service", opts.service_init_))
  {
    ROS_ERROR("Missing initialization_service parameter");
    std::exit(EXIT_FAILURE);
  }

  if (!nh.getParam("planner_service", opts.service_wps_))
  {
    ROS_ERROR("Missing planner_service parameter");
    std::exit(EXIT_FAILURE);
  }

  if (!nh.getParam("refine_service", opts.service_refine_))
  {
    ROS_ERROR("Missing refine_service parameter");
    std::exit(EXIT_FAILURE);
  }

  // Get topics to publish to from parameter server
  if (!nh.getParam("uwb_anchors_topic", opts.uwb_anchors_topic_))
  {
    ROS_ERROR("Missing uwb_anchors_topic parameter");
    std::exit(EXIT_FAILURE);
  }

  if (!nh.getParam("waypoints_topic", opts.waypoints_topic_))
  {
    ROS_ERROR("Missing waypoints_topic parameter");
    std::exit(EXIT_FAILURE);
  }

  // Get frame ids from parameter server
  if (!nh.getParam("frame_id_anchor", opts.frame_id_anchors_))
  {
    ROS_ERROR("Missing frame_id_anchor parameter");
    std::exit(EXIT_FAILURE);
  }
  if (!nh.getParam("frame_id_waypoint", opts.frame_id_waypoints_))
  {
    // in case no frame_id for waypoints is given, assume the same frame_id for both
    opts.frame_id_waypoints_ = opts.frame_id_anchors_;
    ROS_INFO("Missing frame_id_waypoint parameter, using %s", opts.frame_id_waypoints_.c_str());
  }

  // parse waypoint nav type
  std::string wp_nav_type_str;
  if (!nh.getParam("waypoint_nav_type", wp_nav_type_str))
  {
    ROS_ERROR("Missing waypoint_nav_type parameter");
    std::exit(EXIT_FAILURE);
  }
  else
  {
    if (wp_nav_type_str == "global")
      opts.wp_nav_type_ = (uint)mission_sequencer::MissionWaypointArray::GLOBAL;
    else if (wp_nav_type_str == "local")
      opts.wp_nav_type_ = (uint)mission_sequencer::MissionWaypointArray::LOCAL;
    else if (wp_nav_type_str == "position" || wp_nav_type_str == "pos" || wp_nav_type_str == "cur_pos")
      opts.wp_nav_type_ = (uint)mission_sequencer::MissionWaypointArray::CUR_POS;
    else if (wp_nav_type_str == "pose" || wp_nav_type_str == "cur_pose")
      opts.wp_nav_type_ = (uint)mission_sequencer::MissionWaypointArray::CUR_POSE;
    else
    {
      ROS_ERROR("Unknown waypoint_nav_type type '%s'.", wp_nav_type_str.c_str());
      ROS_INFO("  Pleas use either 'global', 'local', 'position' (current), or 'pose' (current)");
      std::exit(EXIT_FAILURE);
    }
  }

  // Get init options from parameter server
  bool opt_enable_ls, opt_compute_covariance;
  nh.param<bool>("enable_ls", opt_enable_ls, false);
  nh.param<bool>("compute_covariance", opt_compute_covariance, false);
  std::string method, bias_type;
  nh.param<std::string>("method", method, "double");
  nh.param<std::string>("bias_type", bias_type, "constant");
  uwb_init::InitMethod opt_init_method;
  uwb_init::BiasType opt_bias_type;
  if (method == "single")
  {
    opt_init_method = uwb_init::InitMethod::SINGLE;
  }
  else if (method == "double")
  {
    opt_init_method = uwb_init::InitMethod::DOUBLE;
  }
  else
  {
    ROS_ERROR("Invalidmethod! Please use single or double");
    std::exit(EXIT_FAILURE);
  }
  if (bias_type == "unbiased")
  {
    opt_bias_type = uwb_init::BiasType::NO_BIAS;
  }
  else if (bias_type == "constant")
  {
    opt_bias_type = uwb_init::BiasType::CONST_BIAS;
  }
  else if (bias_type == "all")
  {
    opt_bias_type = uwb_init::BiasType::ALL_BIAS;
  }
  else
  {
    ROS_ERROR("Invalid bias type! Please use unbiased, constant or all");
    std::exit(EXIT_FAILURE);
  }

  // Get minimum number of anchors from parameter server
  int min_num_anchors;
  uint opt_min_num_anchors;
  nh.param<int>("min_num_anchors_", min_num_anchors, 4);
  opt_min_num_anchors = uint(min_num_anchors);

  // Get publishing options from parameter server
  nh.param<bool>("publish_first_solution", opts.publish_first_solution_, false);

  // Get anchors file path from parameter server
  if (!nh.getParam("anchors_file_path", opts.anchors_file_path_))
  {
    ROS_ERROR("Missing anchors_file_path parameter");
    std::exit(EXIT_FAILURE);
  }

  // Get publish_anchors_tf option from parameter server
  nh.param<bool>("publish_anchors_tf", opts.publish_anchors_tf_, true);

  // Get UWB range options from parameter server
  nh.param<double>("uwb_min_range", opts.uwb_min_range_, 0.0);
  nh.param<double>("uwb_max_range", opts.uwb_max_range_, 100.0);

  // Get biases pror covariances from parameter server
  double opt_const_bias_prior_cov, opt_dist_bias_prior_cov;
  nh.param<double>("const_bias_prior_cov", opt_const_bias_prior_cov, 0.01);
  nh.param<double>("dist_bias_prior_cov", opt_dist_bias_prior_cov, 0.01);

  // Get LS solver options from parameter server
  double opt_sigma_pos, opt_sigma_mes;
  bool opt_check_ls_cov_spd;
  nh.param<double>("position_std", opt_sigma_pos, 0.05);
  nh.param<double>("uwb_range_std", opt_sigma_mes, 0.1);
  nh.param<bool>("check_ls_cov_spd", opt_check_ls_cov_spd, false);

  // Get NLS solver options from parameter server
  double opt_lambda, opt_lambda_scale_factor, opt_step_cond, opt_res_cond;
  int max_iter;
  bool opt_check_nls_cov_spd;
  nh.param<double>("levenberg_marquardt_lambda", opt_lambda, 0.01);
  nh.param<double>("lambda_scale_factor", opt_lambda_scale_factor, 10.0);
  nh.param<double>("step_norm_stop_condition", opt_step_cond, 0.000001);
  nh.param<double>("residual_mse_stop_condition", opt_res_cond, 0.000001);
  nh.param<int>("max_iterations", max_iter, 1000);
  uint opt_max_iter = static_cast<uint>(max_iter);
  nh.param<bool>("check_nls_cov_spd", opt_check_nls_cov_spd, false);

  // Get waypoint generation options from parameter server
  int cell_len, pop_size, itr_num, x_n, y_n, z_n;
  double opt_pc, opt_pm, opt_side_x, opt_side_y, opt_side_z, opt_z_min, opt_C_e_x, opt_C_e_y;
  nh.param<int>("cell_len", cell_len, 10);
  uint opt_cell_len = static_cast<uint>(cell_len);
  nh.param<int>("pop_size", pop_size, 10);
  uint opt_pop_size = static_cast<uint>(pop_size);
  nh.param<int>("generations_n", itr_num, 3000);
  uint opt_itr_num = static_cast<uint>(itr_num);
  nh.param<int>("x_grid", x_n, 2);
  uint opt_x_n = static_cast<uint>(x_n);
  nh.param<int>("y_grid", y_n, 2);
  uint opt_y_n = static_cast<uint>(y_n);
  nh.param<int>("z_grid", z_n, 4);
  uint opt_z_n = static_cast<uint>(z_n);
  nh.param<double>("crossover_probability", opt_pc, 0.5);
  nh.param<double>("mutation_probability", opt_pm, 0.2);
  nh.param<double>("side_x_length", opt_side_x, 4);
  nh.param<double>("side_y_length", opt_side_y, 5);
  nh.param<double>("side_z_length", opt_side_z, 6);
  nh.param<double>("min_z", opt_z_min, 1);
  nh.param<double>("C_e_x", opt_C_e_x, 0);
  nh.param<double>("C_e_y", opt_C_e_y, 0);

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
    std::exit(EXIT_FAILURE);
  }

  // Get calibration UWB tag -> IMU
  std::vector<double> p_ItoU;
  std::vector<double> p_ItoU_default = { 0.0, 0.0, 0.0 };
  nh.param<std::vector<double>>("p_ItoU", p_ItoU, p_ItoU_default);
  Eigen::Vector3d p_UinI(p_ItoU.data());
  opts.p_UinI_ = p_UinI;
  ROS_INFO_STREAM("Calibration p_UinI = " << p_UinI.transpose());

  // Get black list of UWB anchors
  std::vector<double> uwb_id_black_list_default;
  nh.param<std::vector<double>>("uwb_id_black_list", uwb_id_black_list_default);

  std::stringstream ss_id;
  for (auto const& id : uwb_id_black_list_default) {
    opts.uwb_id_black_list.push_back(static_cast<size_t>(id));
    ss_id << id << ",";
  }
  ROS_INFO_STREAM("UWB IDs on black list: = " << ss_id.str());

  // Get waypoint flight options from parameter server
  nh.param<double>("wp_yaw", opts.wp_yaw_, 0.0);
  nh.param<double>("wp_holdtime", opts.wp_holdtime_, 1.0);

  // Make options
  opts.init_options_ = std::make_shared<uwb_init::UwbInitOptions>(
      opt_init_method, opt_bias_type, opt_const_bias_prior_cov, opt_dist_bias_prior_cov, opt_min_num_anchors,
      opt_enable_ls, opt_compute_covariance);
  opts.ls_solver_options_ =
      std::make_unique<uwb_init::LsSolverOptions>(opt_sigma_pos, opt_sigma_mes, opt_check_ls_cov_spd);
  opts.nls_solver_options_ = std::make_unique<uwb_init::NlsSolverOptions>(
      opt_lambda, opt_lambda_scale_factor, opt_step_cond, opt_res_cond, opt_max_iter, opt_check_nls_cov_spd);
  opts.planner_options_ = std::make_unique<uwb_init::PlannerOptions>(
      opt_cell_len, opt_pop_size, opt_itr_num, opt_pc, opt_pm, opt_x_n, opt_y_n, opt_z_n, opt_side_x, opt_side_y,
      opt_side_z, opt_z_min, opt_C_e_x, opt_C_e_y);

  // Instanciate UwbInitRos
  uwb_init_ros::UwbInitRos UwbInitRos(nh, std::move(opts));

  // ROS Spin
  ros::spin();

  // Done!
  ROS_INFO("Done!");
  return EXIT_SUCCESS;
}
