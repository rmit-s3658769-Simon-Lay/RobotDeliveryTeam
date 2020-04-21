#!/bin/bash

HOSTNAME=127.0.0.1
export ROS_IP=${HOSTNAME}
export ROS_MASTER_URI=http://${HOSTNAME}:11311

export MB_LASER_BIRDCAGE_R2000=1
export MB_LASER_BIRDCAGE_R2000_FREQ=50
export MB_LASER_BIRDCAGE_R2000_SAMPLES=3600

source ~/ws_baxter/devel/setup.bash

source ~/navigation_ws/devel/setup.bash
