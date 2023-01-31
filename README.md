# robonp_homeservice_robot

This repository contains the urdf model of a two-wheeled robot equipped with a 2D LiDAR and a RGB-D camera, a simulation environment built using Gazebo, and an autonomy stack 
for the robot.

The ros packages used to achieve autonomous navigation include: AMCL, move_base, gmapping. The differential drive plugin from gazebo provides to serve as the robot controller. Currently, the move_base needs to be tuned to have better performance. 

Packages like pick up and add_marker are implemented to simulate the delivery service. A marker will appear as the targrt position for the robot to reach. Once reached, the marker will disappear and the robot will move to 
a drop-off location. The marker will be shown on the drop-off location to indicate the success of delievery. 

## Visualization from RVIZ and GAZEBO

![](https://github.com/RollingOat/robonp_homeservice_robot/blob/master/gazebo.gif)
![](https://github.com/RollingOat/robonp_homeservice_robot/blob/master/rviz.gif)

