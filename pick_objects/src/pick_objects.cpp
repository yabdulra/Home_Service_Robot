#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef move_base_msgs::MoveBaseGoal Goal;

Goal set_goal(double x, double y, double w){
  Goal goal;

  //we'll send a goal to the robot to move the robot to the goal location.
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.w = w;

  return goal;
}

void delay(double duration){
      ros::Time end = ros::Time::now() + ros::Duration(duration);
      while(ros::Time::now() < end){
        ;
      }
    }

int main(int argc, char** argv){
  ros::init(argc, argv, "pick_objects");

  ros::NodeHandle nh;

  double x1, y1, x2, y2;
  // get node name
  std::string node_name = ros::this_node::getName();  
  //get goals parameters
  nh.getParam(node_name + "/pickup_x", x1);
  nh.getParam(node_name + "/pickup_y", y1);
  nh.getParam(node_name + "/dropoff_x", x2);
  nh.getParam(node_name + "/dropoff_y", y2);

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  Goal goal;

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  goal = set_goal(x1, y1, 1.0);

  ROS_INFO("received pickup location");
  ROS_INFO("moving to pickup location");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("arrived pickup location");
  else
    ROS_INFO("The base failed to move to the pickup location for some reason");

  ROS_INFO("picking up object");
  delay(5.0);
  ROS_INFO("object picked");

  goal = set_goal(x2, y2, 1.0);
 
  ROS_INFO("received drop off location");
  ROS_INFO("moving to drop off location");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("arrived drop off location");
  else
    ROS_INFO("The base failed to move to the drop off location for some reason");
  
  ROS_INFO("dropping off object");
  delay(5.0);
  ROS_INFO("object delivered");
  delay(5.0);

  return 0;
}
