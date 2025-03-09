####################################### TASK 0

. install/setup.bash
. /usr/share/gazebo/setup.sh
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo  turtlebot3_bighouse.launch.py


####################################### TASK 1

. install/setup.bash
. /usr/share/gazebo/setup.sh
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo  turtlebot3_world.launch.py  #### launch turtebot world

. install/setup.bash
. /usr/share/gazebo/setup.sh
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo  turtlebot3_bighouse.launch.py x_pose:=2.7 y_pose:=-1.1  #### launch big house with initial position


. install/setup.bash
. /usr/share/gazebo/setup.sh
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True slam:=True params_file:=/home/guglielmo/project_ws/nav2_params2.yaml

. install/setup.bash
. /usr/share/gazebo/setup.sh
export TURTLEBOT3_MODEL=burger
ros2 launch explore_lite explore.launch.py use_sim_time:=True

############################################## TASK 2

. install/setup.bash
ros2 launch goal_navigator localize_navigate.launch.py

ros2 run roaming_loalizer bug_localizer
ros2 run goal_navigator goal_navigator

############################################## TASK 3

colcon build --packages-select sanitizer

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




