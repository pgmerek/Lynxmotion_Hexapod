#!/bin/bash


cd /home/pi/catkin_ws
source devel/setup.bash
echo "catkin_ws sourced"
cp ~/Lynxmotion_Hexapod/project_files/ros/* ~/catkin_ws/src/lynxmotion_package/scripts/
echo "copied files from repo to catkin package"

echo "starting roscore"
roscore > /dev/null &
echo "waiting...."
sleep 5
echo "waiting...."
sleep 5
echo "starting the orchestrator"
rosrun lynxmotion_package orchestrator.py &
echo "starting recorder"
rosrun lynxmotion_package recorder.py &> /dev/null &
echo "starting dialogflow"
rosrun lynxmotion_package dialogflow_node.py &> /dev/null &
echo "starting talk node"
rosrun lynxmotion_package talk_node.py &> /dev/null &
echo "starting motion control"
rosrun lynxmotion_package hexapod_motion_control.py > /dev/null &
echo "ready to go!"
