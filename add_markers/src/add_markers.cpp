#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
// #include <nav_msgs/Odometry.h>
#include <std_msgs/UInt8.h>

// bool marker_reach_state = false;
uint8_t goal_reach_state = 0;

/* robot goal proximity callback function */
void goalReachCallback(const std_msgs::UInt8::ConstPtr& msg)
{
   goal_reach_state = msg->data;
   return;
}

/* main function */
int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");

  ros::NodeHandle nh;
  ros::Rate r(5);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber odom_sub = nh.subscribe("/goal_reached", 1, goalReachCallback);
  bool done = false;

  // Use SPHERE as the initial shape 
  uint32_t shape = visualization_msgs::Marker::SPHERE;

  ROS_INFO("Subscribed to desired goal-position");

  while (ros::ok()) {
    //Do this every cycle to ensure the subscriber receives the message
    ros::spinOnce();
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    // Set the marker id and namespace. 
   
    marker.ns = "basic_shapes";
    marker.id = 0;
    // Set the marker shapes.  
    marker.type = shape;
    //Marker Scales
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
     

    // ser the markers color
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    switch (goal_reach_state)
    {
      case 0: 
        {
           
          marker.action = visualization_msgs::Marker::ADD;
           //Marker drop off pose and orientation in within the enviroment 
          nh.getParam("/object_pick_location/tx", marker.pose.position.x);
          nh.getParam("/object_pick_location/ty", marker.pose.position.y);
          nh.getParam("/object_pick_location/tz", marker.pose.position.z);
          nh.getParam("/object_pick_location/qx", marker.pose.orientation.x);
          nh.getParam("/object_pick_location/qy", marker.pose.orientation.y);
          nh.getParam("/object_pick_location/qz", marker.pose.orientation.z);
          nh.getParam("/object_pick_location/qw", marker.pose.orientation.w);
	 
          break;
        } // case 0

        case 1:   // robot reach pickup site, delete pick-up marker
          {
            sleep(2);
            //ROS_INFO("hiding PICK-UP marker");
            marker.action = visualization_msgs::Marker::DELETE;
            break;
          } // case 1

        case 2: // wait for robot to reach drop-off site
          {
            marker.action = visualization_msgs::Marker::DELETE;
            break;
          }

        case 3:   // publish drop-off marker
          {
            sleep(5);
            //ROS_INFO("adding drop-off marker");
           
            marker.action = visualization_msgs::Marker::ADD;
            //Marker drop off pose and orientation in within the enviroment
            nh.getParam("/object_drop_location/tx", marker.pose.position.x);
            nh.getParam("/object_drop_location/ty", marker.pose.position.y);
            nh.getParam("/object_drop_location/tz", marker.pose.position.z);
            nh.getParam("/object_drop_location/qx", marker.pose.orientation.x);
            nh.getParam("/object_drop_location/qy", marker.pose.orientation.y);
            nh.getParam("/object_drop_location/qz", marker.pose.orientation.z);
            nh.getParam("/object_drop_location/qw", marker.pose.orientation.w);
		
            done = true;
            break;
          }

    } // switch

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }

    //publish the marker
    marker_pub.publish(marker);

    // if last marker published and noted as done exit
    if (done) {
      ROS_INFO("=== DESTINATION Reached ===");
      sleep(7);
      return 0;
      }

    r.sleep();
  } // while ros-ok

  return 0;
} // main
