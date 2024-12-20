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

namespace uwb_init_ros
{
UwbInitRos::UwbInitRos(const ros::NodeHandle& nh, UwbInitRosOptions&& options)
  : nh_(nh)
  , options_(std::move(options))
  , uwb_init_(options_.level_, std::move(options_.init_options_), std::move(options_.ls_solver_options_),
              std::move(options_.nls_solver_options_), std::move(options_.planner_options_))
{
  size_t const queue_sz = 10;
  // Subscribers
  if (!options_.estimated_pose_cov_topic_.empty()) {
    estimated_pose_cov_sub_
      = nh_.subscribe(options_.estimated_pose_cov_topic_, queue_sz, &UwbInitRos::callbackPoseWithCov, this);
    ROS_INFO("Subsribing to %s", estimated_pose_cov_sub_.getTopic().c_str());
  }
  if (!options_.estimated_pose_topic_.empty()) {
    estimated_pose_sub_ = nh_.subscribe(options_.estimated_pose_topic_, queue_sz, &UwbInitRos::callbackPose, this);
    ROS_INFO("Subsribing to %s", estimated_pose_sub_.getTopic().c_str());
  }
  if (!options_.estimated_transform_topic_.empty()) {
    estimated_transform_sub_
      = nh_.subscribe(options_.estimated_transform_topic_, queue_sz, &UwbInitRos::callbackTransform, this);
    ROS_INFO("Subsribing to %s", estimated_transform_sub_.getTopic().c_str());
  }
  if (!options_.estimated_odometry_topic_.empty()) {
    estimated_odom_sub_
      = nh_.subscribe(options_.estimated_odometry_topic_, queue_sz, &UwbInitRos::callbackOdometry, this);
    ROS_INFO("Subsribing to %s", estimated_odom_sub_.getTopic().c_str());
  }

  if (!options_.uwb_range_topic_.empty()) {
    uwb_range_sub_ = nh_.subscribe(options_.uwb_range_topic_, queue_sz, &UwbInitRos::callbackUwbRanges, this);
    ROS_INFO("Subsribing to %s", uwb_range_sub_.getTopic().c_str());
  }

  if (!options_.uwb_twr_topics_.empty()) {
    for (auto const& topic : options_.uwb_twr_topics_) {
      uwb_twr_subs_.push_back(nh_.subscribe(topic, queue_sz, &UwbInitRos::callbackUwbTwoWayRanges, this));
      ROS_INFO("Subsribing to %s", topic.c_str());
    }
  }



  // Publishers
  uwb_anchors_pub_ = nh_.advertise<uwb_msgs::UwbAnchorArrayStamped>(options_.uwb_anchors_topic_, 1);
  waypoints_pub_ = nh_.advertise<mission_sequencer::MissionWaypointArray>(options_.waypoints_topic_, 1);

  // Services
  start_srv_ = nh_.advertiseService(options_.service_start_, &UwbInitRos::callbackServiceStart, this);
  reset_srv_ = nh_.advertiseService(options_.service_reset_, &UwbInitRos::callbackServiceReset, this);
  init_srv_ = nh_.advertiseService(options_.service_init_, &UwbInitRos::callbackServiceInit, this);
  wps_srv_ = nh_.advertiseService(options_.service_wps_, &UwbInitRos::callbackServiceWps, this);
  refine_srv_ = nh_.advertiseService(options_.service_refine_, &UwbInitRos::callbackServiceRefine, this);

  feed_stationary_anchor_pos();

}

void UwbInitRos::callbackPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  // Get pose
  Eigen::Vector3d p_IinG(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  Eigen::Quaterniond q_GI(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y,
                          msg->pose.orientation.z);

  // Feed p_UinG
  if (collect_measurements_)
  {
    std::scoped_lock lock{mtx_service_};
    for (auto const& e : options_.dict_p_UinI_)
    {
      // Compute position of UWB module in Global frame
      p_UinG_ = p_IinG + q_GI.toRotationMatrix() * e.second;
      uwb_init_.feed_position(msg->header.stamp.toSec(), p_UinG_, e.first);
    }
  }
}

void UwbInitRos::callbackPoseWithCov(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  // Get pose
  Eigen::Vector3d p_IinG(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  Eigen::Quaterniond q_GI(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                          msg->pose.pose.orientation.z);

  // Feed p_UinG
  if (collect_measurements_)
  {
    std::scoped_lock lock{mtx_service_};
    for (auto const& e : options_.dict_p_UinI_)
    {
      // Compute position of UWB module in Global frame
      p_UinG_ = p_IinG + q_GI.toRotationMatrix() * e.second;
      uwb_init_.feed_position(msg->header.stamp.toSec(), p_UinG_, e.first);
    }
  }
}

void UwbInitRos::callbackTransform(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
  // Get pose
  Eigen::Vector3d p_IinG(msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z);
  Eigen::Quaterniond q_GI(msg->transform.rotation.w, msg->transform.rotation.x, msg->transform.rotation.y,
                          msg->transform.rotation.z);

  // Feed p_UinG
  if (collect_measurements_)
  {
    std::scoped_lock lock{mtx_service_};
    // Feed p_UinG
    for (auto const& e : options_.dict_p_UinI_)
    {
      // Compute position of UWB module in Global frame
      p_UinG_ = p_IinG + q_GI.toRotationMatrix() * e.second;
      uwb_init_.feed_position(msg->header.stamp.toSec(), p_UinG_, e.first);
    }
  }
}

void UwbInitRos::callbackOdometry(const nav_msgs::Odometry::ConstPtr &msg) {
  // Get pose
  Eigen::Vector3d p_IinG(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  Eigen::Quaterniond q_GI(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                          msg->pose.pose.orientation.z);

          // Feed p_UinG
  if (collect_measurements_)
  {
    std::scoped_lock lock{mtx_service_};
    // Feed p_UinG
    for (auto const& e : options_.dict_p_UinI_)
    {
      // Compute position of UWB module in Global frame
      p_UinG_ = p_IinG + q_GI.toRotationMatrix() * e.second;
      uwb_init_.feed_position(msg->header.stamp.toSec(), p_UinG_, e.first);
    }
  }
}

void UwbInitRos::callbackUwbRanges(const mdek_uwb_driver::UwbConstPtr& msg)
{
  // Feed measurements
  if (collect_measurements_)
  {
    // Parse message into correct data structure
    std::vector<uwb_init::UwbData> data;
    // Fill data
    for (const auto& it : msg->ranges)
    {
      // Check if the id is valid (contains only numbers)
      if (!containsChar(it.id))
      {
        // Convert id
        std::stringstream sstream(it.id);
        uint id;
        sstream >> id;

        // Valid is true if the range is within the specified range
        double valid = ((it.distance >= options_.uwb_min_range_) && (it.distance <= options_.uwb_max_range_));

        if (!uwb_id_on_black_list(id)) {
          // Fill vector
          data.emplace_back(uwb_init::UwbData(valid, it.distance, id));
        }
      }
      else
      {
        ROS_WARN("Received UWB message containing characters in the id field. Measurement discarded");
      }
    }
    std::scoped_lock lock{mtx_service_};
    uwb_init_.feed_uwb(msg->header.stamp.toSec(), data);
  }
}

void UwbInitRos::callbackUwbTwoWayRanges(const uwb_msgs::TwoWayRangeStampedConstPtr& msg)
{
  // Feed measurements
  if (collect_measurements_)
  {
    std::scoped_lock lock{mtx_service_};
    // case TAG -> ANCHOR
    if(uwb_id_is_tag(msg->UWB_ID1) && !uwb_id_on_black_list(msg->UWB_ID2) && !uwb_id_is_tag(msg->UWB_ID2))
    {
      // Parse message into correct data structure
      // Valid is true if the range is greater than threshold (0.0)
      bool valid = (msg->range_raw >= options_.uwb_min_range_) && (msg->range_raw <= options_.uwb_max_range_);
      if (valid)
      {
        uwb_init::UwbData data(true, msg->range_raw, msg->UWB_ID2, msg->UWB_ID1);
        uwb_init_.feed_uwb(msg->header.stamp.toSec(), data);
      }
    }
    // case ANCHOR -> TAG
    else if(!uwb_id_is_tag(msg->UWB_ID1) && !uwb_id_on_black_list(msg->UWB_ID1) && uwb_id_is_tag(msg->UWB_ID2))
    {
      // Parse message into correct data structure
      // Valid is true if the range is greater than threshold (0.0)
      bool valid = (msg->range_raw >= options_.uwb_min_range_) && (msg->range_raw <= options_.uwb_max_range_);
      if (valid)
      {
        uwb_init::UwbData data(true, msg->range_raw, msg->UWB_ID1, msg->UWB_ID2);
        uwb_init_.feed_uwb(msg->header.stamp.toSec(), data);
      }
    }
    else {
      ROS_WARN_THROTTLE(10, "wbInitRos::cbUwbTwoWayRanges():  UWB_ID1[%d] to UWB_ID2[%d] is on black list or a tag!", msg->UWB_ID1, msg->UWB_ID2);
      return;
    }
  }
}

bool UwbInitRos::callbackServiceStart([[maybe_unused]] std_srvs::Empty::Request& req,
                                      [[maybe_unused]] std_srvs::Empty::Response& res)
{
  std::scoped_lock lock{mtx_service_};
  ROS_INFO("Start service called.");
  // Clear buffers at each start
  uwb_init_.clear_buffers();
  collect_measurements_ = true;
  feed_stationary_anchor_pos();
  return true;
}

bool UwbInitRos::callbackServiceReset([[maybe_unused]] std_srvs::Empty::Request& req,
                                      [[maybe_unused]] std_srvs::Empty::Response& res)
{
  std::scoped_lock lock{mtx_service_};
  ROS_INFO("Reset service called.");
  uwb_init_.reset();
  collect_measurements_ = false;
  return true;
}

bool UwbInitRos::callbackServiceInit([[maybe_unused]] std_srvs::Empty::Request& req,
                                     [[maybe_unused]] std_srvs::Empty::Response& res)
{
  std::scoped_lock lock{mtx_service_};
  // Stop collecting measurements
  collect_measurements_ = false;

  // Initialize anchors
  ROS_INFO("Inizialization service called. Stopped data collection.");
  return initializeAnchors();
}

bool UwbInitRos::callbackServiceWps([[maybe_unused]] std_srvs::Empty::Request& req,
                                    [[maybe_unused]] std_srvs::Empty::Response& res)
{
  std::scoped_lock lock{mtx_service_};
  ROS_INFO("Waypoints generation service called.");
  return computeWaypoints();
}

bool UwbInitRos::callbackServiceRefine([[maybe_unused]] std_srvs::Empty::Request& req,
                                       [[maybe_unused]] std_srvs::Empty::Response& res)
{
  std::scoped_lock lock{mtx_service_};
  // Stop collecting measurements
  collect_measurements_ = false;

  // Refine anchors
  ROS_INFO("Refinement service called.");
  return refineAnchors();
}

void UwbInitRos::publishAnchors(const uwb_init::NLSSolutions& sols) {
  // Create message
  uwb_msgs::UwbAnchorArrayStamped anchors_msg_;
  anchors_msg_.header.stamp = ros::Time::now();
  anchors_msg_.header.frame_id = options_.frame_id_anchors_;
  anchors_msg_.header.seq = 0;
  for (const auto& it : sols)
  {
    ROS_INFO("Anchor [%d] succesfully refined at [%f, %f, %f]", it.first, it.second.anchor_.p_AinG_.x(),
             it.second.anchor_.p_AinG_.y(), it.second.anchor_.p_AinG_.z());

    std::vector<size_t> ref_ids;


    // Anchor message
    uwb_msgs::UwbAnchor anchor;
    anchor.id = it.first;
    anchor.position.x = it.second.anchor_.p_AinG_.x();
    anchor.position.y = it.second.anchor_.p_AinG_.y();
    anchor.position.z = it.second.anchor_.p_AinG_.z();

    size_t idx_id = 0;
    size_t num_tags = it.second.betas_.size();
    for(auto const& e : it.second.gammas_) {
      anchor.ref_ids.push_back(e.first);
      anchor.gammas.push_back(e.second);
      double sigma_gamma = std::sqrt(it.second.cov_(3+idx_id, 3+idx_id));
      anchor.sigma_gammas.push_back(sigma_gamma);
      // check if beta exists:
      double beta = 1.0;

      if(it.second.betas_.find(e.first) != it.second.betas_.end()) {
        beta =it.second.betas_.at(e.first);

        double sigma_beta = std::sqrt(it.second.cov_(3+num_tags+idx_id, 3+num_tags+idx_id));
        anchor.sigma_betas.push_back(sigma_beta);
      }
      anchor.betas.push_back(beta);
    }

    // Store covarinace fully
    int index = 0;
    for (int i = 0; i < std::min(3, (int)it.second.cov_.rows()); i++)
    {
      for (int j = 0; j < std::min(3, (int)it.second.cov_.cols()); j++)
      {
        anchor.covariance_p.at(index++) = it.second.cov_(i, j);
      }
    }

    anchors_msg_.anchors.push_back(anchor);
    anchors_msg_.header.seq++;

    // Publish anchor tf if requested
    if (options_.publish_anchors_tf_)
    {
      publishAnchorTf(it.second.anchor_, "anchor_" + std::to_string(it.first));
    }
  }

  // Publish anchors
  uwb_anchors_pub_.publish(anchors_msg_);
}

void UwbInitRos::publishWaypoints(const uwb_init::Waypoints& wps)
{
  // Create message
  mission_sequencer::MissionWaypointArray waypoints_msg_;
  for (const auto& it : wps)
  {
    ROS_INFO("Waypoint at [%f, %f, %f]", it.x_, it.y_, it.z_);

    mission_sequencer::MissionWaypoint wp;
    wp.x = it.x_;
    wp.y = it.y_;
    wp.z = it.z_;
    wp.yaw = options_.wp_yaw_;
    wp.holdtime = options_.wp_holdtime_;
    waypoints_msg_.waypoints.push_back(wp);
  }

  waypoints_msg_.header.seq++;
  waypoints_msg_.header.stamp = ros::Time::now();
  waypoints_msg_.header.frame_id = options_.frame_id_waypoints_;
  waypoints_msg_.reference = options_.wp_nav_type_;
  waypoints_msg_.action = mission_sequencer::MissionWaypointArray::CLEAR;

  // INFO(scm): is_global is deprecated and will be removed soon, use reference above instead!
  // waypoints_msg_.is_global = false;  // Set to false to use local coordinates

  // Publish waypoints
  waypoints_pub_.publish(waypoints_msg_);
}

void UwbInitRos::saveAnchorsYaml(const uwb_init::NLSSolutions& sols) {

  if (options_.anchors_yaml_file_path_.empty()) {
    return;
  }
  // Create YAML emitter
  YAML::Emitter emitter;

  // Convert sol to vector of pairs
  std::vector<std::pair<uint, uwb_init::NLSSolution>> solsVector(sols.begin(), sols.end());

  // Sort the vector by the anchor id
  std::sort(solsVector.begin(), solsVector.end(),
            [](const std::pair<uint, uwb_init::NLSSolution>& a, const std::pair<uint, uwb_init::NLSSolution>& b) {
              return a.second.anchor_.id_ < b.second.anchor_.id_;
            });

  // Counter
  int i = 0;

  // Anchors map
  emitter << YAML::Precision(3) << YAML::BeginMap;

  for (const auto& it : solsVector) {
    emitter << YAML::Key << "anchor" + std::to_string(i);

    // Anchor map
    emitter << YAML::BeginMap;
    {
      emitter << YAML::Key << "id" << YAML::Value << (unsigned int) it.first;

      // Set the "fix" parameter to "true" for the first two anchors
      if (i < 2)
      {
        emitter << YAML::Key << "fix" << YAML::Value << "true";
      }
      else
      {
        emitter << YAML::Key << "fix" << YAML::Value << "false";
      }

      // Write p_AinG as a vector
      emitter << YAML::Key << "p_AinG";
      emitter << YAML::Value << YAML::Flow << YAML::BeginSeq
              << uwb_init::roundn(it.second.anchor_.p_AinG_.x(),3)
              << uwb_init::roundn(it.second.anchor_.p_AinG_.y(),3)
              << uwb_init::roundn(it.second.anchor_.p_AinG_.z(),3) << YAML::EndSeq;

      // Write bias parameters
      emitter << YAML::Key << "const_biases" << YAML::Value << YAML::Flow << YAML::BeginSeq;
      for(auto const& e : it.second.gammas_) {
        emitter<< uwb_init::roundn(e.second,3);
      }
      emitter << YAML::EndSeq;
      emitter << YAML::Key << "dist_biases" << YAML::Value << YAML::Flow << YAML::BeginSeq;
      for(auto const& e : it.second.betas_) {
        // todo(RJ): why beta - 1.0 ?
        emitter<< uwb_init::roundn(e.second,3);
      }
      emitter << YAML::EndSeq;
      emitter << YAML::Key << "Tag_IDs" << YAML::Value << YAML::Flow << YAML::BeginSeq;
      for(auto const& e : it.second.gammas_) {
        emitter<< e.first;
      }
      emitter << YAML::EndSeq;

              // Write prior p_AinG covariance as the mean of the first 3 elements of the diagonal of cov_
      double prior_p_AinG_cov =
        (std::sqrt(it.second.cov_(0, 0)) + std::sqrt(it.second.cov_(1, 1)) + std::sqrt(it.second.cov_(2, 2))) / 3;
      emitter << YAML::Key << "prior_p_AinG_cov" << YAML::Value << uwb_init::roundn(prior_p_AinG_cov,3);

              // Write prior bias covariance as the last two elements of the diagonal of cov_
      double prior_const_bias_cov = std::sqrt(it.second.cov_(3, 3));
      double prior_dist_bias_cov = std::sqrt(it.second.cov_(4, 4));
      emitter << YAML::Key << "prior_const_bias_cov" << YAML::Value << uwb_init::roundn(prior_const_bias_cov,3);
      emitter << YAML::Key << "prior_dist_bias_cov" << YAML::Value << uwb_init::roundn(prior_dist_bias_cov,3);

    }
    // End anchor map
    emitter << YAML::EndMap;

    // Add blank line between anchors in the YAML file
    emitter << YAML::Newline;
    // Increment counter
    i++;
  }

  // End anchors map
  emitter << YAML::EndMap;

  // Check if path to file exists
  std::filesystem::path filepath = std::filesystem::path(options_.anchors_yaml_file_path_).parent_path();
  if (!std::filesystem::is_directory(filepath))
  {
    // create directory
    ROS_WARN_STREAM("Path to file does not exist, creating " << filepath);
    if (!std::filesystem::create_directories(filepath))
    {
      ROS_ERROR_STREAM("Unable to create path " << filepath);
      return;
    }
  }

  // Generate YAML file into the options_.anchors_file_path_
  std::ofstream file(options_.anchors_yaml_file_path_);

  // If the file is not open, print an error message and return
  if (!file.is_open())
  {
    ROS_ERROR_STREAM("Unable to open file " << options_.anchors_yaml_file_path_);
    return;
  }

  // Write the %YAML:1.0 directive directly to the file stream (1.0 is required since 1.2 is not widely supported)
  file << "%YAML:1.0\n% \n";

  file << "% autogenerated by uwb_init_ros with ROS time " << std::fixed << ros::Time::now().toSec() << std::scientific
       << "\n";
  {
    // generate chrono time point for now
    const std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    file << "% generated on " << std::put_time(std::localtime(&now), "%F %T.\n") << "\n";
  }

  // Write the YAML emitter to the file stream
  file << emitter.c_str();

  // Close the file
  file.close();
}

void UwbInitRos::saveAnchorCSV(const uwb_init::NLSSolutions &sols) {
  if (options_.anchors_csv_file_path_.empty()) {
    return;
  }
  // Generate YAML file into the options_.anchors_file_path_
  std::ofstream file(options_.anchors_csv_file_path_);

          // If the file is not open, print an error message and return
  if (!file.is_open())
  {
    ROS_ERROR_STREAM("Unable to open file " << options_.anchors_csv_file_path_);
    return;
  }
  std::vector<std::pair<uint, uwb_init::NLSSolution>> solsVector(sols.begin(), sols.end());

 // Sort the vector by the anchor id
  std::sort(solsVector.begin(), solsVector.end(),
            [](const std::pair<uint, uwb_init::NLSSolution>& a, const std::pair<uint, uwb_init::NLSSolution>& b) {
              return a.second.anchor_.id_ < b.second.anchor_.id_;
            });

  file << "Anchor_ID, x, y, z, sigma_x, sigma_y, sigma_z" << std::endl;

  for (const auto& it : solsVector) {
    file << it.second.anchor_.id_ << ","
         << uwb_init::roundn(it.second.anchor_.p_AinG_(0),3) << ","
         << uwb_init::roundn(it.second.anchor_.p_AinG_(1),3) << ","
         << uwb_init::roundn(it.second.anchor_.p_AinG_(2),3) << ","
         << uwb_init::roundn(sqrt(it.second.cov_(0,0)),2) << ","
         << uwb_init::roundn(sqrt(it.second.cov_(1,1)),2) << ","
         << uwb_init::roundn(sqrt(it.second.cov_(2,2)),2)
         << std::endl;
  }
  // Close the file
  file.close();

}

void UwbInitRos::publishAnchorTf(const uwb_init::UwbAnchor& anchor, const std::string& anchor_name)
{
  // Create transform
  tf::StampedTransform anchor_tf;
  anchor_tf.frame_id_ = options_.frame_id_anchors_;
  anchor_tf.child_frame_id_ = anchor_name;
  anchor_tf.stamp_ = ros::Time::now();
  anchor_tf.setOrigin(tf::Vector3(anchor.p_AinG_.x(), anchor.p_AinG_.y(), anchor.p_AinG_.z()));
  anchor_tf.setRotation(tf::Quaternion(0, 0, 0, 1));

  // Publish transform
  tf_broadcaster_.sendTransform(anchor_tf);
}

bool UwbInitRos::initializeAnchors()
{
  ROS_INFO("Performing anchor inizalization...");

  // Initialize anchors
  try
  {
    if (!uwb_init_.init_anchors())
    {
      ROS_WARN("Initialization of anchor failed. Please collect additional data and repeat.");
      ROS_WARN_STREAM("Call " << options_.service_start_ << " to start collecting measurements.");
      return false;
    }
  }
  catch(const std::exception& e) {
      std::cout << e.what();
    throw;
  }

  // Stop collecting measurements
  ROS_INFO("Anchors initialization completed.");

  // If enabled, publish and save anchors
  if (options_.publish_first_solution_)
  {
    ROS_INFO("Publishing and saving solution...");
    publishAnchors(uwb_init_.get_nls_solutions());

    if (!options_.anchors_yaml_file_path_.empty()) {
      saveAnchorsYaml(uwb_init_.get_nls_solutions());
    }
    if (!options_.anchors_csv_file_path_.empty()) {
      saveAnchorCSV(uwb_init_.get_nls_solutions());
    }
  }

  return true;
}  // namespace uwb_init_ros

bool UwbInitRos::computeWaypoints()
{
  ROS_INFO("Computing optimal waypoints...");

  // Compute waypoints passing last registerd UWB tag position
  if (!uwb_init_.compute_waypoints(p_UinG_))
  {
    ROS_WARN("Optimal waipoints computation failed. Please make sure that the UWB anchors have been correctly "
             "initialized.");
    return false;
  }

  // Display and publish waypoints
  publishWaypoints(uwb_init_.get_waypoints());

  // Collect measurements for refinement
  collect_measurements_ = true;

  return true;
}

bool UwbInitRos::refineAnchors()
{
  ROS_INFO("Performing anchor refinement...");

  // Refine anchors
  if (!uwb_init_.refine_anchors())
  {
    ROS_WARN("Refinement of anchor failed. Please collect additional data and repeat.");
    ROS_WARN_STREAM("Call " << options_.service_start_ << " to start collecting measurements.");
    return false;
  }

  // Publish and save refined anchors
  ROS_INFO("Publishing and saving refined solution...");
  publishAnchors(uwb_init_.get_refined_solutions());
  if (!options_.anchors_yaml_file_path_.empty()) {
    saveAnchorsYaml(uwb_init_.get_refined_solutions());
  }
  if (!options_.anchors_csv_file_path_.empty()) {
    saveAnchorCSV(uwb_init_.get_refined_solutions());
  }

  return true;
}

bool UwbInitRos::uwb_id_on_black_list(const size_t id) {
  for (auto const& id_ : options_.uwb_id_black_list) {
    if (id_ == id) {
      return true;
    }
  }
  return false;
}

bool UwbInitRos::uwb_id_is_tag(const size_t id) {
  return (options_.dict_p_UinI_.find(id) != options_.dict_p_UinI_.end()) ||  (options_.dict_p_AinG_.find(id) != options_.dict_p_AinG_.end());
}

void UwbInitRos::feed_stationary_anchor_pos() {
  for (auto const& e : options_.dict_p_AinG_) {
    auto const Anchor_ID = e.first;
    auto const p_GtoA = e.second;

    // at least two elements are needed:
    uwb_init_.feed_position(0.0, p_GtoA, Anchor_ID);
    uwb_init_.feed_position(double(INT_MAX), p_GtoA, Anchor_ID);
  }
}

}  // namespace uwb_init_ros
