Autonomous and Mobile Robotics - Guglielmo Morselli - 2024

# TASK 0
## Setting up the environment
. install/setup.bash
. /usr/share/gazebo/setup.sh
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo  turtlebot3_bighouse.launch.py


# TASK 1
## Explore the map
### Start Gazebo and Nav2
. install/setup.bash
. /usr/share/gazebo/setup.sh
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo  turtlebot3_bighouse.launch.py x_pose:=2.7 y_pose:=-1.1  #### launch big house with initial position

. install/setup.bash
. /usr/share/gazebo/setup.sh
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True slam:=True params_file:=/home/guglielmo/project_ws/nav2_params2.yaml

### Start the exploration using https://github.com/robo-friends/m-explore-ros2/tree/main
. install/setup.bash
. /usr/share/gazebo/setup.sh
export TURTLEBOT3_MODEL=burger
ros2 launch explore_lite explore.launch.py use_sim_time:=True

# TASK 2
## Localize in the map and navigate to user-defined locations
. install/setup.bash
ros2 launch goal_navigator localize_navigate.launch.py

# TASK 3
## Sanitize user-defined rooms
. install/setup.bash
. /usr/share/gazebo/setup.sh
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo  turtlebot3_bighouse.launch.py x_pose:=2.7 y_pose:=-1.1

. install/setup.bash
. /usr/share/gazebo/setup.sh
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=/home/guglielmo/project_ws/map/map.yaml params_file:=/home/guglielmo/project_ws/nav2_params2.yaml

. install/setup.bash
ros2 launch sanitizer sanitize_map.launch.py




