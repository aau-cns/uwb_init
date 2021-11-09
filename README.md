# USB Init CPP ROS Wrapper

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
    git clone git@gitlab.aau.at:mascheiber/uwb_init_cpp.git
    git clone git@gitlab.aau.at:aau-cns/evb1000_driver.git
    ```

### Build

1. Compile using `catkin build`:
    ```[bash]
    cd catkin_ws/src && catkin build uav_init_uwb
    ```

## Usage

This rosnode can be launched with the provided launchfile by running

```[bash]
roslaunch uwb_init_cpp uwb_init.launch
```

### Launch Parameters
The provided launchfile and node allows the setting of the following parameters

> TODO
