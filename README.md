# robile_gazebo

Gazebo simulations for KELO robile robots

## Installation
In addition to a few ROS dependencies, this package also depends on the following packages provided by KELO-robotics:
1. [kelo_tulip](https://github.com/kelo-robotics/kelo_tulip)
2. [robile_description](https://git.locomotec.com:444/kelo/platforms/robile_description)

Assuming you have a catkin workspace at `~/catkin_ws`, execute the below commands to install the simulator and its dependencies

~~~ sh
sudo apt install ros-$ROS_DISTRO-controller-manager ros-$ROS_DISTRO-effort-controllers ros-$ROS_DISTRO-velocity-controllers ros-$ROS_DISTRO-gazebo-ros ros-$ROS_DISTRO-gazebo-ros-control

cd ~/catkin_ws/src
git clone https://github.com/kelo-robotics/kelo_tulip.git
git clone https://chavan@git.locomotec.com:444/kelo/platforms/robile_description.git
git clone https://chavan@git.locomotec.com:444/kelo/simulation/robile_gazebo.git

catkin build kelo_tulip # you will need to enter your password for the kelo_tulip build to complete

catkin build robile_description robile_gazebo
source ~/catkin_ws/devel/setup.bash
~~~

## Usage

To start the gazebo simulator, use one of the launch files defined in the [launch/](launch/) directory as follows:

~~~ sh
roslaunch robile_gazebo 4_wheel_platform.launch
~~~

You can then publish command velocities on the ros topic `/cmd_vel` to move the robot around.

### Additional launch file arguments

Several arguments can be passed when launching the simulator. A complete list of supported roslaunch arguments can be found in the [launch/platform_independent/robile_gazebo.launch](launch/platform_independent/robile_gazebo.launch) file. We list a few most important ones here.

| Argument name        | Type    | Default | Description                                                                   |
|----------------------|---------|---------|-------------------------------------------------------------------------------|
| platform_max_lin_vel | float   | 1.0     | Maximum linear velocity the platform can achieve in the XY plane (in m/s)     |
| platform_max_ang_vel | float   | 1.0     | Maximum angular velocity the platform can achieve along the Z axis (in rad/s) |
| init_pos_x           | float   | 0.0     | Spawn position of robot along the world X-axis                                |
| init_pos_y           | float   | 0.0     | Spawn position of robot along the world Y-axis                                |
| init_yaw             | float   | 0.0     | Spawn orientation of robot along the world Z-axis                             |
| gazebo_gui           | boolean | true    | Start the Gazebo simulation GUI                                               |
| start_rviz           | boolean | true    | Start RViz with a predefined RViz configuration file                          |

For example to change the spawn position of the robot to `(2.0, 3.0)`, and disable the Gazebo GUI, use the below command:

~~~ sh
roslaunch robile_gazebo 4_wheel_platform.launch init_pos_x:=2.0 init_pos_y:=3.0 gazebo_gui:=false
~~~

## Adding a new platform
