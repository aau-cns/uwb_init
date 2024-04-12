# Change Log
All notable changes to this project will be documented in this file.
 
The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).
 
## [Unreleased] - 2024-04-11
 
This feature includes support for:
- two-way ranging measurments, 
- calibration (position and pairwise biases) using multiple tags or known stationary anchors,
- and an outlier rejection method based on RANSAC.

 
### Added

- support for nav_msgs::Odometry as pose update
- Readme in uwb_init_lib
- optionally saving calibrated anchors in CSV file (id, x,y,z,sigma_x,sigma_y,sigma_z) 
- command line example tool to run a anchor calibration  
- (N)LsSolver: RANSAC bases outlier rejection method added
- CMakeList option to use system wide or local Eigen3 library
- supporting: both Tag-to-Anchor and Anchor-to-Anchor calibration  
- (N)LsSolver: supporting multiple tags for anchnor position and bias calibration
- UwbInitRos: thread safe ROS callbacks
- LS/NLSSolution: ref_id added to represent relation between tags and anchors 
- UWB ID blacklisting: ignore IDs from that list in two way ranging measurements.
- supporting: geometry_msgs::PoseStamped + geometry_msgs::TransformStamped
- supporting: uwb_msgs::TwoWayRangeStamped (from gitlab.aau.at:aau-cns/ros_pkgs/uwb_msgs.git) 

### Changed

- dependency to UwbInitOptions removed from Solvers
- UwbAnchor*.msg moved to uwb_msgs package (gitlab.aau.at:aau-cns/ros_pkgs/uwb_msgs.git): needed to remove dependency that package
- Upgrading to YAML-CPP v.0.8.0 
- saved anchors file supports multiple tags and multiple pairwise biases. Check for future use with BughWright2 packages. 

### Fixed

- distance bias in anchor_calib YAML file is directly written. Previously it was substracted by 1, like "yaml << dist_bias - 1.0;" 
- CMakeLists.txt: fix populated internal dependency + install binary
- semicolon after method removed and typos fixed
- TimedBuffer: return value fixed
- CMakeLists.txt: fix issues with debug build (#2)
