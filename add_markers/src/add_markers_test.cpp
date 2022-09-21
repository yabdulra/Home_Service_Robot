#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>


class Markers{
  private:
    ros::NodeHandle n;
    ros::Publisher marker_pub;
    visualization_msgs::Marker marker;

  public:
    Markers(){
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


int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers_test");

  Markers markers;

  ros::NodeHandle nh;

  ros::Rate loop_rate(10);

  double x1, y1, x2, y2;
  // get node name
  std::string node_name = ros::this_node::getName();  
  //get goals parameters
  nh.getParam(node_name + "/pickup_x", x1);
  nh.getParam(node_name + "/pickup_y", y1);
  nh.getParam(node_name + "/dropoff_x", x2);
  nh.getParam(node_name + "/dropoff_y", y2);

  // Set our shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;
 
  markers.delay(1);
  markers.define_marker(shape);
  markers.add_marker(x1, y1);
  ROS_INFO("virtual object at pickup location");
  markers.delay(5.0);

  markers.delete_marker();
  ROS_INFO("object picked up");
  markers.delay(5.0);

  markers.define_marker(shape);
  markers.add_marker(x2, y2);
  ROS_INFO("virtual object at drop off location");
  markers.delay(5.0);

  return 0;
}
