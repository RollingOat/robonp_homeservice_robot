// Initially show the marker at the pickup zone
// Hide the marker once your robot reaches the pickup zone
// Wait 5 seconds to simulate a pickup
// Show the marker at the drop off zone once your robot reaches it

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <math.h>

ros::Publisher marker_pub;
// Set our initial shape type to be a cube
uint32_t shape = visualization_msgs::Marker::CUBE;

void publish_marker(double x, double y, bool act){

    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    if(act)
        marker.action = visualization_msgs::Marker::ADD;
    else
        marker.action = visualization_msgs::Marker::DELETE;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    marker_pub.publish(marker);
}

void reachCallback(const geometry_msgs::PoseWithCovarianceStamped& robot_pos){
    double thres = 0.5;
    double robot_x = robot_pos.pose.pose.position.x;
    double robot_y = robot_pos.pose.pose.position.y;
    double pick_x;
    double pick_y;
    double pick_w;
    double drop_x;
    double drop_y;
    double drop_w;
    ros::param::get("pick_up_x",pick_x);
    ros::param::get("pick_up_y",pick_y);
    ros::param::get("pick_up_w",pick_w);
    ros::param::get("drop_off_x",drop_x);
    ros::param::get("drop_off_y",drop_y);
    ros::param::get("drop_off_w",drop_w);
    double dist = sqrt(pow(pick_x - robot_x, 2) + pow(pick_y - robot_y, 2));
    if(dist<=thres){
        publish_marker(pick_x, pick_y, true);
    }
    else{
        publish_marker(pick_x, pick_y, false);
    }
    
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  marker_pub = n.advertise<visualization_msgs::Marker>("target_object", 1);
  ros::Subscriber odom_sub = n.subscribe("/amcl_pose ", 10, reachCallback);
  ros::spin();
}