# Mark location of self so that robot_upstart knows where to find the setup file.
export ROBOT_SETUP=/etc/ros/setup.bash

# Setup robot upstart jobs to use the IP from the network bridge.
# export ROBOT_NETWORK=br0

# Load the robot's model type and serial number
source /etc/clearpath-serial.bash

# Insert extra platform-level environment variables here. The six hashes below are a marker
# for scripts to insert to this file.
######

# Pass through to the main ROS workspace of the system.
source /opt/ros/noetic/setup.bash

# If you have a catkin workspace, source it below. e.g.
source /home/administrator/catkin_ws/devel/setup.bash

# Any additional environment variables that depend on your workspace should be exported here
source /home/administrator/catkin_ws/src/que36_ridgeback/que36_bringup/config/setup.bash
