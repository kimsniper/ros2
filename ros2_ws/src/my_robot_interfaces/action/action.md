# ROS2 - ACTIONS

Basic utilities, commands and tools when working with ROS2 Actions 

Reference: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html

## Basic Commands

### Show action interface definition

Navigate to the ros2 workspace directory and run the following command.

Command: ros2 interface show `package name`/`path of action`/`action file`

```bash
ros2 run my_cpp_pkg count_until_server
ros2 interface show my_robot_interfaces/action/CountUntil
```
### List active action interfaces

While a single or multiple active action nodes were executed using `ros2 run` command, run the following command

```bash
ros2 action list
```
### Display active action information

While a single or multiple active action nodes were executed using `ros2 run` command, run the following command

Command: ros2 action info `action name` \
This command should display the number of clients and services as well as the name of the node where it is created

```bash
ros2 run my_cpp_pkg count_until_server
ros2 action info /count_until
```
### Display active action topics

Actions actually provide topics hidden from a user's perspective because we dont deal with these topics directly. \
But for some reason, we may use them such as subscribing to these topics. 

While a single or multiple active action nodes were executed using `ros2 run` command, run the following command


```bash
ros2 topic list --include-hidden-topics
```
### Display active action services

Actions actually provide topics hidden from a user's perspective because we dont deal with these services directly. \
But for some reason, we may use them such as using these services. 

While a single or multiple active action nodes were executed using `ros2 run` command, run the following command


```bash
ros2 service list --include-hidden-services
```

### List active action with interfaces

While a single or multiple active action nodes were executed using `ros2 run` command, run the following command


```bash
ros2 action list -t
```

### List active action with interfaces

While a single or multiple active action nodes were executed using `ros2 run` command, run the following command

Command: ros2 action send_goal `action name` `package name`/`path of action`/`action file` `parameter arguments` 

```bash
ros2 action send_goal /count_until my_robot_interfaces/action/CountUntil "{target_number: 4, period: 1.3}"
```

Add feedback
```bash
ros2 action send_goal /count_until my_robot_interfaces/action/CountUntil "{target_number: 4, period: 1.3}" --feedback
```