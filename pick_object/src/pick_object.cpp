#include <ros/ros.h>
#include <vector>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void setup_goal(move_base_msgs::MoveBaseGoal& goal, std::vector<double>& v, MoveBaseClient& ac){
    // set up the frame parameters
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    // Define a position and orientation for the robot to reach
    goal.target_pose.pose.position.x = v[0];
    goal.target_pose.pose.position.y = v[1];
    goal.target_pose.pose.orientation.w = v[2];

    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    // Wait an infinite time for the results
    ac.waitForResult();
}

int main(int argc, char** argv){
    // Initialize the simple_navigation_goals node
    ros::init(argc, argv, "pick_object");
    ros::NodeHandle nh;
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    // Wait 5 sec for move_base action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    
    std::vector<double> pick_up_goal(3);
    std::vector<double> drop_off_goal(3);
    nh.getParam("pick_up_x", pick_up_goal[0]);
    nh.getParam("pick_up_y", pick_up_goal[1]);
    nh.getParam("pick_up_w", pick_up_goal[2]);
    nh.getParam("drop_off_x", drop_off_goal[0]);
    nh.getParam("drop_off_y", drop_off_goal[1]);
    nh.getParam("drop_off_w", drop_off_goal[2]);
    ROS_INFO("pick up coordinate:%f, %f",pick_up_goal[0], pick_up_goal[1]);
    ROS_INFO("drop off coordinate:%f, %f",drop_off_goal[0], drop_off_goal[1]);

    move_base_msgs::MoveBaseGoal goal;

    // pick up
    setup_goal(goal, pick_up_goal, ac);
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("Hooray, the robot reached the pick up location");
        // wait for 5 second
        ros::Duration(5.0).sleep();
        // drop-off
        setup_goal(goal, drop_off_goal, ac);
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            ROS_INFO("Hooray, the robot reached the drop off location");
        }
    }
    else
        ROS_INFO("The base failed to reach the pick up location");
    return 0;
}