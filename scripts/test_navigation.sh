#!/bin/sh
xterm -e "source devel/setup.bash; roslaunch my_robot world.launch" &
sleep 5
xterm -e "source devel/setup.bash; roslaunch mcl_localizer amcl.launch" &
sleep 5
xterm -e "rosrun rviz rviz -d src/rvizConfig/" &
sleep 5
xterm -e "rosrun teleop_twist_keyboard teleop_twist_keyboard.py"