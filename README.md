# ROS2 - Robot Operating System 2 For Beginners

A beginner's guide to start working with projects based on ROS2 using C++

## Overview

This reposotory is based on Edouard Renard's Udemy courses below:

ROS2 For Beginners   - https://www.udemy.com/course/ros2-for-beginners/ \
ROS2 For Beginners 2 - https://www.udemy.com/course/ros2-tf-urdf-rviz-gazebo/ \
ROS2 For Begineers 3 - https://www.udemy.com/course/ros2-advanced-core-concepts/ 

### Preferred Environment Setup

To run this example without issues, the following environment setup are preferred.

|                  |                      |
|------------------|----------------------|
| Operating System | UBUNTU 22.04 (Jammy) |
| ROS2 Version     | ROS2 Humble          |

**Note: ** In my case, I was able to execute the ROS2 activities using VirtualBox.

## Colcon build

Navigate to the ros2 workspace directory and run the following command. 

Build all packages 

```bash
colcon build
```
Build specific package

```bash
colcon build --packages-select `name_of_package`
```

See the [Colcon Tutorial Link](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html) for full steps to configure and build ROS2 packages.