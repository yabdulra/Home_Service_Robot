#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

#include <iostream>
#include <string>
#include <vector>
#include <cmath>

class AddMarkers{
  private:
    ros::NodeHandle n;
    ros::Publisher marker_pub;
    visualization_msgs::Marker marker;

  public:
    AddMarkers(){
      marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    };

    void define_marker(int shape){
      // Set the frame ID and timestamp.  See the TF tutorials for information on these.
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();

      // Set the namespace and id for this marker.  This serves to create a unique ID
      // Any marker sent with the same namespace and id will overwrite the old one
      marker.ns = "basic_shapes";
      marker.id = 0;

      // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
      marker.type = shape;

    }

    void add_marker(double x, double y){
      // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
      marker.action = visualization_msgs::Marker::ADD;

      // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
      marker.pose.position.x = x;
      marker.pose.position.y = y;
      marker.pose.position.z = 0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker.scale.x = 0.2;
      marker.scale.y = 0.2;
      marker.scale.z = 0.2;

      // Set the color -- be sure to set alpha to something non-zero!
      marker.color.r = 0.5f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;

      marker.lifetime = ros::Duration();

      marker_pub.publish(marker);
    }

    void delete_marker(){
      marker.action = visualization_msgs::Marker::DELETE;
      marker_pub.publish(marker);
    }

    void delay(double duration){
      ros::Time end = ros::Time::now() + ros::Duration(duration);
      while(ros::Time::now() < end){
        ;
      }
    }
};

class Robot{
  private:
    ros::NodeHandle nh;
    ros::Subscriber odom;
    double x_pos = 0.0, y_pos = 0.0;
  
  public:
    Robot(){
      odom = nh.subscribe("odom", 10, &Robot::odom_callback, this);
    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg){
      x_pos = msg->pose.pose.position.x;
      y_pos = msg->pose.pose.position.y;
    }

    std::vector<double> get_robot_pose(){
      return {x_pos, y_pos};
    }
};

double euclidean_distance(double x1, double y1, double x2, double y2){
  return sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2));
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  Robot robot;
  AddMarkers markers;

  ros::Rate loop_rate(10);

  // Set our shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;
  std::vector<double> robot_pose, pickup{-7.0, 2.0}, dropoff{3.0, -6.5};

  double distance, x, y;
  bool picked = false, dropped = false;
    
  x = pickup[0];
  y = pickup[1];
  while (ros::ok())
  { 
    robot_pose = robot.get_robot_pose();
    distance = euclidean_distance(robot_pose[0], robot_pose[1], x, y);

    if(picked == false && distance > 0.2){
      markers.define_marker(shape);
      markers.add_marker(pickup[0], pickup[1]);
    }
    else if (picked == false && distance <= 0.2){
      markers.delay(8);
      markers.delete_marker();
      picked = true;
    }
    else{
      x = dropoff[0];
      y = dropoff[1];
      distance = euclidean_distance(robot_pose[0], robot_pose[1], x, y);
  
      if (dropped == false && distance <= 0.2){
        markers.delay(4);
        markers.define_marker(shape);
        markers.add_marker(dropoff[0], dropoff[1]);
        dropped = true;
      }
    }
    
    ros::spinOnce();
    loop_rate.sleep();
  }
}
