# UWB initialization library ROS wrapper

A ROS1 workspace to use the UWB initialization library explained in the paper UVIO: An UWB-Aided Visual-Inertial 
Odometry Framework with Bias-Compensated Anchors Initialization (https://arxiv.org/abs/2308.00513).

Maintainers: [Giulio Delama](mailto:giulio.delama@aau.at) and [Alessandro Fornasier](mailto:alessandro.fornasier@aau.at)

## Credit

This code was written by the [Control of Networked System (CNS)](https://www.aau.at/en/smart-systems-technologies/control-of-networked-systems/), 
University of Klagenfurt, Klagenfurt, Austria.

## License

This software is made available to the public to use (_source-available_), licensed under the terms of the BSD-2-Clause-
License with no commercial use allowed, the full terms of which are made available in the `LICENSE` file. No license in
patents is granted.

### Usage for academic purposes

If you use this software in an academic research setting, please cite the
corresponding [academic paper] and consult the `LICENSE` file for a detailed explanation.

```latex
@inproceedings{Delama2023,
   author       = {Delama, Giulio and Shamsfakhr, Farhad and Weiss, Stephan and Fontanelli, Daniele and Fornasier, Alessandro},
   booktitle    = {2023 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
   title        = {UVIO: An UWB-Aided Visual-Inertial Odometry Framework with Bias-Compensated Anchors Initialization},
   year         = {2023},
  organization  = {IEEE}
}
```

## Getting Started

The main folder constains the C++ library and its ROS wrapper.

### Requirements

These software components are needed on your platform to run the ROS node.

- [ROS](https://www.ros.org/): tested with [ROS noetic](http://wiki.ros.org/noetic/Installation)

### Prerequisites

1. Create a catkin workspace and install [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/installing.html)
    ```[bash]
    mkdir -p catkin_ws/src && cd catkin_ws
    sudo apt update && sudo apt install -y python3-catkin-tools
    catkin init
    catkin config --extend /opt/ros/$(rosversion -d) --cmake-args -DCMAKE_BUILD_TYPE=Release -j4 -l4
    ```
2. Clone the required ROS packeges to your workspace
    ```[bash]
    cd src
    git clone git@gitlab.aau.at:aau-cns/ros_pkgs/mdek_uwb_driver.git
    git@github.com:aau-cns/mission_sequencer.git
    git clone git@gitlab.aau.at:aau-cns/ros_pkgs/uwb_init_cpp.git
    git clone git@gitlab.aau.at:aau-cns/ros_pkgs/uwb_msgs.git
    ```

### Build

1. Compile using `catkin build`:
    ```[bash]
    cd catkin_ws/src && catkin build uwb_init_ros
    ```
2. (not required) The C++ stand-alone library can also be builded and installed with:
    ```[bash]
    cd uwb_init_lib
    mkdir build && cd build
    cmake ../
    cmake --build .
    sudo cmake --install .


## Usage

The objective is to initialize a set of unknown UWB anchors. To understand the motivation and the detailed initialization procedure
please refer to the [academic paper].

The rosnode can be launched with the provided launchfile by running

```[bash]
roslaunch uwb_init_ros uwb_init.launch
```

The initialization can be performed by calling the following ros services:

1. Start collecting measurements (poses and UWB ranges)

```[bash]
rosservice call uwb_init_ros/start "{}"
```

2. Initialize UWB anchors (after first flight)

```[bash]
rosservice call uwb_init_ros/initialize "{}"
```

3. (optional) Compute optimal waypoints

```[bash]
rosservice call uwb_init_ros/compute_wps "{}"
```

4. (optional) Perform a flight through the waypoints (start collecting data with 1. before flying) and refine previous initialization

```[bash]
rosservice call uwb_init_ros/refine "{}"
```


### Launch Parameters
The provided launchfile and node allows the setting of the following parameters. Each of these parameters can either be set in the launchfile, set through other launchfiles by using the `<include>` and `<arg>` tags, or set through the command line with `roslaunch uwb_init_ros uwb_init.launch <PARAMETER>:=<VALUE>`.

#### ROS Topics and Services
| ROS parameter | description | default value |
|---|---|---|
| estimated_pose_topic | name of the pose topic used for anchor initialization | `/uvio/poseimu` |
| uwb_range_topic | name of the uwb topic used for anchor initialization  | `/uwb_driver_node/uwb` |
| uwb_anchors_topic | name of the topic used to publish the anchors after initialization | `~uwb_anchors` |
| waypoints_topic | name of topic used to publish the generated optimal waypoints | `~wps` |

## Reporting Issues

In case of issues, feature requests, or other questions please open a [New Issue](https://gitlab.aau.at/aau-cns/ros_pkgs/uwb_init_cpp/issues/new?issue) or contact the authors via email.

## Authors

* Giulio Delama ([email](mailto:giulio.delama@aau.at?subject=[UWB%20Init]))
* Alessandro Fornasier ([email](mailto:alessandro.fornasier@ieee.org?subject=[UWB%20Init]))
* Martin Scheiber ([email](mailto:martin.scheiber@ieee.org?subject=[UWB%20Init]))

<!-- LINKS: -->
[academic paper]: https://arxiv.org/abs/2308.00513
