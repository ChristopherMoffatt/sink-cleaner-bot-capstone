#!/usr/bin/env bash

export RIDGEBACK_RISER_HEIGHT=0.07

#Front SICK LMS-111 Lidar
export RIDGEBACK_FRONT_SICK_LASER=1
export RIDGEBACK_FRONT_LASER_HOST=192.168.131.21

#Rear SICK LMS-111 Lidar
export RIDGEBACK_REAR_SICK_LASER=1
export RIDGEBACK_REAR_LASER_HOST=192.168.131.22

#Kuka Arm
#export RIDGEBACK_ARM_HEIGHT=0.445
#export RIDGEBACK_ARM_ANGLE=-1.5708

export RIDGEBACK_URDF_EXTRAS=$(catkin_find que36_description urdf/ridgeback_description.urdf.xacro --first-only)
