#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

nav_msgs::Odometry odom;

double x,y,z;


void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  x = (double)msg->pose.pose.position.x;
  y = (double)msg->pose.pose.position.y;
  z = (double)msg->pose.pose.orientation.z;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/robot_odom", 1, odomCallback);

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;



  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, z , 1), 
	tf::Vector3(x,y,0.2)),
        ros::Time::now(),"base_link", "laser"));
  ros::spinOnce();
    r.sleep();
  }
ros::spin();
}
