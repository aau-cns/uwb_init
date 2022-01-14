# UWB Init CPP ROS Wrapper

## License
This software is made available to the public to use (_source-available_), licensed under the terms of the BSD-2-Clause-
License with no commercial use allowed, the full terms of which are made available in the `LICENSE` file. No license in
patents is granted.

### Usage for academic purposes
If you use this software in an academic research setting, please cite the
corresponding paper and consult the `LICENSE` file for a detailed explanation.

```latex
@inproceedings{Blueml2021,
   author       = {Blueml, Julian and Fornasier, Alessandro and Weiss, Stephan},
   booktitle    = {2021 IEEE International Conference on Robotics and Automation (ICRA)},
   pages        = {5490--5496},
   title        = {Bias Compensated UWB Anchor Initialization using Information-Theoretic Supported
                   Triangulation Points},
   year         = {2021},
  organization  = {IEEE}
}
```

## Getting Started

### Requirements
These software components are needed on your platform to run this ROS node.

- [ROS](https://www.ros.org/): tested with [ROS noetic](http://wiki.ros.org/noetic/Installation)

### Prerequisites

1. Create a catkin workspace and install [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/installing.html)
    ```[bash]
    mkdir -p catkin_ws/src && cd catkin_ws
    sudo apt update && sudo apt install -y python3-catkin-tools
    catkin init
    catkin config --extend /opt/ros/$(rosversion -d) --cmake-args -DCMAKE_BUILD_TYPE=Release -j4 -l4
    ```
1. Clone the required ROS packeges to your workspace
    ```[bash]
    cd src
    git clone git@gitlab.aau.at:aau-cns/evb1000_driver.git
    git clone git@gitlab.aau.at:aau-cns/ros_pkgs/amaze_mission_sequencer.git
    git clone git@gitlab.aau.at:aau-cns/ros_pkgs/uwb_init_cpp.git
    ```

### Build

1. Compile using `catkin build`:
    ```[bash]
    cd catkin_ws/src && catkin build uwb_init_cpp
    ```

## Usage

This rosnode can be launched with the provided launchfile by running

```[bash]
roslaunch uwb_init_cpp uwb_init.launch
```

### Initialization Methods
This node is able to initialize the anchor positions including the measurement bias in various different formulations, using either a single or two measurements.

#### Method
The initialization method is set by the parameter `init_method`.

##### `SINGLE`
Uses only a single measurement in the formulation. This method performs slightly worse, as the problem for bias and anchor position differs in several order of magnitudes.

##### `DOUBLE`
Uses a pair of measurements at least `meas_baseline_idx` idices apart (in the buffer) and at least `meas_baseline_m` meters in measurement distance difference. This method performs better, as it keeps the problem in the same order of magnitude.

#### Variables
Various different variables can be initialized with this node, chosen by the `init_variables` parameter.

| Symbol                                            | Description                             |
| ------------------------------------------------- | --------------------------------------- |
| ![](https://latex.codecogs.com/svg.latex?z_m)     | anchor measurement                      |
| ![](https://latex.codecogs.com/svg.latex?z)       | actual distance between anchor and tag  |
| ![](https://latex.codecogs.com/svg.latex?\alpha)  | distance dependent bias                 |
| ![](https://latex.codecogs.com/svg.latex?\gamma)  | constant bias                           |

It should be noted that the actual distance between the tag and anchor uses the current tag position *p* to incorporate the unknown anchor position *q* in this formulation. I.e.

![](https://latex.codecogs.com/svg.latex?z^2=(p-q)^2)

##### `FULL_BIAS`
Initialize all unknown variables and biases with the measurement equation

![](https://latex.codecogs.com/svg.latex?z_m=(1+\alpha)z+\gamma)

##### `NO_BIAS`
Initialize the anchor position using no bias with the measurement equation

![](https://latex.codecogs.com/svg.latex?z_m=z)

##### `CONST BIAS`
Initialize the anchor position using only a constant bias with the measurement equation

![](https://latex.codecogs.com/svg.latex?z_m=z+\gamma)

*This performance is untested inflight.*

##### `CONST BIAS`
Initialize the anchor position using only a distance-dependent bias with the measurement equation

![](https://latex.codecogs.com/svg.latex?z_m=(1+\alpha)z)

*This performance is untested inflight.*

### Common  Issues with Initialization
This UWB initialization node's success highly depends on the path flown and quality of the measurements received.  The standard parameters are tuned for the TREK1000 UWB modules, for which the original version of this code is released for. Hence it might be relevant to adjust some parameters (see [launch parameters](#launch-parameters)) to run this node with your modules successfully.

#### Failure: LLS condition number
This failure is common if the condition number of the LLS problem does not get lower. Try to accumulate more distinctive data by flying more random trajectories and try to gather anchor measurements from far and close to the anchor, if possible. If this does not help, increase the `max_cond_number` parameter.

#### Falure: COV SVD threshold
We added a check on the covariance matrix condition number to check for the quality of the initialization. In general, the more data, the lower the covariance matrix condition number of the solution, the better the solution. Thus try to accumulate more data. This parameter (`cov_sv_threshold`) is fine tuned, increasing it by as much as `5e-4` might change the whole initialization process (in quality and duration).

### Launch Parameters
The provided launchfile and node allows the setting of the following parameters. Each of these parameters can either be set in the launchfile, set through other launchfiles by using the `<include>` and `<arg>` tags, or set through the command line with `roslaunch uwb_init_cpp uwb_init.launch <PARAMETER>:=<VALUE>`.

#### ROS Topics and Services
| ROS parameter | description | default value |
|---|---|---|
| topic_sub_pose | name of the pose topic used for anchor initialization | `/mus/ground_truth_pose_imu` |
| topic_uwb_in | name of the uwb topic used for anchor initialization  | `/mus/uwb` |
| topic_anchor_out | name of the topic used to publish the anchor initialization | `~anchors` |
| topic_wp_out | name of topic used to publish the waypoints for flying initialization | `~waypoints` |
| srv_get_start_pose | name of service client used to get the relative starting pose | `getStartPose` |
| srv_start_init | name service server used to allow initialization start | `/uwb_init/start` |

#### Initialization Parameters
| ROS parameter | description | default value |
|---|---|---|
| calib_p_ItoU | calibration between the Acnhor U and the IMU I | `[0.0, 0.0, 0.0]` |
| buffer_size_in_s | size of the buffer for measurements in seconds  | `50.0` |
| max_cond_number | maximum condition number of the LLS problem for successful initialization  | `300` |
| reg_lambda | lambda value used for regression (set to 0.0 if regression should be deactivated)  | `100` |
| cov_sv_threshold | threshold on the svd valu of the covariance matrix for successful initialization  | `3e-3` |
| regularize_z | also add the z value of the anchor position to the regularization (only use when anchors are on xy-plane of estimation)  | `False` |
| init_check_duration_in_s | period duration of the interval to calculate new calibration in seconds | `5.0` |
| t_pose_diff_s | calibtrated time offset between the pose and anchor measurement timestamps | `0.0` |
| init_method | method used for initialization | `DOUBLE` |
| init_variables | variables to initialize | `FULL_BIAS` |
| meas_baseline_idx | index offset between measurements used in `DOUBLE` initialization method | `50` |
| meas_baseline_m | minimum discance difference between measurements used in `DOUBLE` initialization method in meter | `0.3` |
| exit_when_done | exit the node when all anchors are initialized | `False` |
| continous_init | perform the initialization continously, mutal exclusive to *exit_when_done* | `False` |
| auto_init | start initialization automatically and do not wait for service call | `True` |

#### Waypoint Generation
| ROS parameter | description | default value |
|---|---|---|
| wp_max_dist_in_m | maximum distance between calculated waypoints in meters | `1.0` |
| wp_height_in_m | height of calculated waypoints | `1.0` |
| wp_random_offset_in_m | random addition to calculated anchor position waypoint in meters | `0.0` |
| wp_holdtime_in_s | hold/hover time at waypoints in seconds | `0.5` |

#### Debugging
| ROS parameter | description | default value |
|---|---|---|
| debug | Starts the node with GDB attached. the code has to be built with `--cmake-args -DCMAKE_BUILD_TYPE=Debug` for this to work. | `False` |

## Package Layout

<details><summary>Package Tree</summary>
<p>

```
.
|-- cfg
|   `-- UwbInit.cfg
|-- CHANGELOG.md
|-- CMakeLists.txt
|-- CMakeLists.txt.user
|-- CONTRIBUTORS.md
|-- docs
|   |-- CMakeLists.txt
|   |-- pkg-docs
|   |   |-- CMakeLists.txt
|   |   `-- doxyfile.in
|   `-- resources
|       |-- citations.bib
|       |-- cns_UNI_and_CNS_LOGO_uniblue.png
|       `-- cns_UNI_and_CNS_LOGO_uniblue.svg
|-- include
|   |-- options
|   |   `-- uwb_init_options.hpp
|   |-- types
|   |   |-- buffers
|   |   |-- buffers.hpp
|   |   |-- randomizer.hpp
|   |   |-- types.hpp
|   |   |-- uwb_anchor.hpp
|   |   `-- uwb_data.hpp
|   |-- uav_init
|   |   |-- uwb_init.cpp
|   |   `-- uwb_init.hpp
|   |-- utils
|   |   |-- colors.hpp
|   |   |-- logging.hpp
|   |   `-- parse_ros.hpp
|   `-- uwb_wrapper.hpp
|-- launch
|   |-- uwb_init_debug.launch
|   `-- uwb_init.launch
|-- LICENSE
|-- msg
|   |-- UwbAnchorArrayStamped.msg
|   `-- UwbAnchor.msg
|-- package.xml
|-- README.md
`-- src
    |-- main.cpp
    `-- uwb_wrapper.cpp
```

</p>
</details>

## Reporting Issues

In case of issues, feature requests, or other questions please open a [New Issue](https://gitlab.aau.at/aau-cns/ros_pkgs/uwb_init_cpp/issues/new?issue) or contact the authors via [email](mailto:martin.scheiber@ieee.org?cc=alessandro.fornasier@ieee.org&subject=[GitLab]%20uwb_init_cpp:%20).

## Authors

* Martin Scheiber ([email](mailto:martin.scheiber@ieee.org?subject=[UWB%20Init]))
* Alessandro Fornasier ([email](mailto:alessandro.fornasier@ieee.org?subject=[UWB%20Init]))
