# The SOCSPIONEER Package

`socspioneer` is a package of configurations and helper nodes for use
with the School of Computer Science Intelligent Robotics module.

## Installation

**NOTE**: This part assumes basic understanding of Linux terminal and commandline usage. Basic understanding of ROS workflow and package organisation is also important (eg. [Building a catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)).

### Ideal Working Environment

- Ubuntu 18.04
- [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) (desktop-full)

**NOTE**: Installing ROS Melodic by default installs Python 2 packages instead of Python 3. For compatibility and lack of conflicts, stick to using Python2 when writing Python nodes unless absolutely necessary.

### Install dependencies

- `sudo apt install ros-$ROS_DISTRO-pr2-teleop ros-$ROS_DISTRO-joy`
- *Not needed when using simulation; Only required for real Pioneer 3DX robot.* `sudo apt install ros-$ROS_DISTRO-p2os-driver ros-$ROS_DISTRO-p2os-launch ros-$ROS_DISTRO-p2os-urdf ros-$ROS_DISTRO-p2os-teleop`.

### Build package

- Clone this repo to the `src` directory of your catkin workspace.
- Build the catkin workspace (`catkin_make` or `catkin build`).

## Testing Simulation and Installation

If everything installed correctly, the following steps should provide a very simplistic simulation of a robot in a provided world map.

1. In one terminal, run roscore.
2. In another, run `rosrun stage_ros stageros <catkin_ws>/src/socs_p2os/socspioneer/data/lgfloor.world`. This should start a simple simulated world with a robot and a map.
3. In a third terminal, run `roslaunch socspioneer keyboard_teleop.joy`.

This would allow you to move the robot using keyboard commands.

## Simulator Usage

**NOTE**: This part assumes basic understanding of ROS, ROS topics, messages, nodes, etc.

Running `rosrun stage_ros stageros <.world file>` will start the simulator with a robot and an obstacle the provided world. The robot and object can be interacted with using the mouse or using ROS topics, nodes, etc. The world view can also be changed using the mouse (pressing `R` on keyboard toggles between 2D and 3D views.)

The simulator publishes the following (important) topics. By subscribing to these topics, you can access different sensor information from the robot.

| ROS Topic | Data | ROS Message Type |
| ------ | ------ | ------ |
| `/odom` | The odometry information from the robot wheel encoders. | [`nav_msgs/Odometry`](http://docs.ros.org/kinetic/api/nav_msgs/html/msg/Odometry.html) |
| `/base_scan` | Laser scan data from the laser scanner at the front of the robot. | [`sensor_msgs/LaserScan`](http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/LaserScan.html) |

The simulator subscribes to the following topics. You can control the robot using this.

| ROS Topic | Data | ROS Message Type |
| ------ | ------ | ------ |
| `/cmd_vel` | Velocity commands to the robot's wheel motors. | [`geometry_msgs/Twist`](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html) |
