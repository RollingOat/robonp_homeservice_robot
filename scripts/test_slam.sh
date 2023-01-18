#!/bin/sh
xterm -e "source devel/setup.bash; roslaunch my_robot world.launch" &
sleep 5
xterm -e "source devel/setup.bash; roslaunch rtab_mapping mapping.launch" &
sleep 5
xterm -e "rosrun rviz rviz -d src/rvizConfig/test_slam_config.rviz" &
sleep 5
xterm -e "rosrun teleop_twist_keyboard teleop_twist_keyboard.py"
