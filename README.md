:warning: This fork of RotorS is thought to be used fro aerial manipulation purposes along with the [EagelMPC](https://github.com/PepMS/eagle-mpc) library and the corresponding [ROS packages](https://github.com/PepMS/eagle_mpc_ros). :warning:

You may want to use the branch *eagle_mpc_ros*. For any other regular uses, you should use the [original repo](https://github.com/ethz-asl/rotors_simulator).

:warning: **Modified readme below, with modified installation instructions** :warning:

RotorS
===============

RotorS is a MAV gazebo simulator.
It provides some multirotor models such as the [AscTec Hummingbird](http://www.asctec.de/en/uav-uas-drone-products/asctec-hummingbird/), the [AscTec Pelican](http://www.asctec.de/en/uav-uas-drone-products/asctec-pelican/), or the [AscTec Firefly](http://www.asctec.de/en/uav-uas-drone-products/asctec-firefly/), but the simulator is not limited for the use with these multicopters.

There are simulated sensors coming with the simulator such as an IMU, a generic odometry sensor, and the [VI-Sensor](http://wiki.ros.org/vi_sensor), which can be mounted on the multirotor.

This package also contains some example controllers, basic worlds, a joystick interface, and example launch files.

Below we provide the instructions necessary for getting started. See RotorS' wiki for more instructions and examples (https://github.com/ethz-asl/rotors_simulator/wiki).

If you are using this simulator within the research for your publication, please cite:
```bibtex
@Inbook{Furrer2016,
author="Furrer, Fadri
and Burri, Michael
and Achtelik, Markus
and Siegwart, Roland",
editor="Koubaa, Anis",
chapter="RotorS---A Modular Gazebo MAV Simulator Framework",
title="Robot Operating System (ROS): The Complete Reference (Volume 1)",
year="2016",
publisher="Springer International Publishing",
address="Cham",
pages="595--625",
isbn="978-3-319-26054-9",
doi="10.1007/978-3-319-26054-9_23",
url="http://dx.doi.org/10.1007/978-3-319-26054-9_23"
}
```
Installation Instructions
-------------------------
 1. ROS Noetic and Gazebo should already be installed. Install the remaining dependencies:

 ```console
 $ sudo apt-get update
 $ sudo apt-get install ros-noetic-joy ros-noetic-octomap-ros ros-noetic-mavlink python3-wstool python3-catkin-tools protobuf-compiler libgoogle-glog-dev ros-noetic-control-toolbox
 ```

 2. Clone *mav_msgs* to your ROS workspace and RotorS

 ```console
 $ cd <ros_ws>/src
 $ git clone https://github.com/ethz-asl/mav_comm.git
 $ git clone https://github.com/PepMS/rotors_simulator.git
 ```

 3. Build your workspace

   ```
   $ cd <ros_ws>
   $ catkin_make
   ```
