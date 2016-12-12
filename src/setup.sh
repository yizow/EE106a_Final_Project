#!/bin/bash

startup.sh &
roslaunch boba_bot head_camera.launch &
rosrun boba_bot headsub.py &
rosrun boba_bot readscale.py &
