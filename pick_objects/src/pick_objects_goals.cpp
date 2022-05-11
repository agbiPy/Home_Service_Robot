#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/UInt8.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/* main function */
int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle nh;
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  //set up publisher to broadcast if robot is at goal marker
  ros::Publisher goal_reach_pub = nh.advertise<std_msgs::UInt8>("/goal_reached", 1);
  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal; // pick-up & drop goal
  std_msgs::UInt8 status_msg;  // goal reach status

  // pick-goal setup
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Send the robot position and orientation of the pick-up location
  ROS_INFO("publishing pick-up goal1");
   // Pose and the orientation of the robot pickup distance within its enviroment
  nh.getParam("/object_pick_location/tx", goal.target_pose.pose.position.x);
  nh.getParam("/object_pick_location/ty", goal.target_pose.pose.position.y);
  nh.getParam("/object_pick_location/tz", goal.target_pose.pose.position.z);
  nh.getParam("/object_pick_location/qx", goal.target_pose.pose.orientation.x);
  nh.getParam("/object_pick_location/qy", goal.target_pose.pose.orientation.y);
  nh.getParam("/object_pick_location/qz", goal.target_pose.pose.orientation.z);
  nh.getParam("/object_pick_location/qw", goal.target_pose.pose.orientation.w);

  ROS_INFO("publishing pick-up goal2");
   // Pose and the orientation of the robot pickup distance within its enviroment
  nh.getParam("/object_pick_location2/tx", goal.target_pose.pose.position.x);
  nh.getParam("/object_pick_location2/ty", goal.target_pose.pose.position.y);
  nh.getParam("/object_pick_location2/tz", goal.target_pose.pose.position.z);
  nh.getParam("/object_pick_location2/qx", goal.target_pose.pose.orientation.x);
  nh.getParam("/object_pick_location2/qy", goal.target_pose.pose.orientation.y);
  nh.getParam("/object_pick_location2/qz", goal.target_pose.pose.orientation.z);
  nh.getParam("/object_pick_location2/qw", goal.target_pose.pose.orientation.w);
  ac.sendGoal(goal);

  ac.sendGoal(goal);



  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Robot has arrived at PICK-UP location");

    ROS_INFO("picking up package");
    // publish goal-reach status
    status_msg.data = 1;
 
    goal_reach_pub.publish(status_msg);
  }
  else {
    ROS_INFO("The robot failed to reach pick-up location");
    return 0;
  }


  // robot reached pick-up location, send drop-off location
  ROS_INFO("Sending drop-off goal");
  // wait a bit before next message
  ros::Duration(3.0).sleep();

  // Pose and the orientation of the robot drop off distance within its enviroment
  nh.getParam("/object_drop_location/tx", goal.target_pose.pose.position.x);
  nh.getParam("/object_drop_location/ty", goal.target_pose.pose.position.y);
  nh.getParam("/object_drop_location/tz", goal.target_pose.pose.position.z);
  nh.getParam("/object_drop_location/qx", goal.target_pose.pose.orientation.x);
  nh.getParam("/object_drop_location/qy", goal.target_pose.pose.orientation.y);
  nh.getParam("/object_drop_location/qz", goal.target_pose.pose.orientation.z);
  nh.getParam("/object_drop_location/qw", goal.target_pose.pose.orientation.w);
  ac.sendGoal(goal);

  ROS_INFO("Robot moving to Drop-off location");
  // Wait for results
  ac.waitForResult();

  // Check if the robot reached its drop goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    sleep(2);
    ROS_INFO("Dropping Package");
    // publish goal-reach status
    status_msg.data = 3;
    goal_reach_pub.publish(status_msg);
  }
  else {
    ROS_INFO("The robot failed to reach drop-off location");
  }

  // wait
  ros::Duration(5.0).sleep();

  return 0;
}
