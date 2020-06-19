#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "shallow_depth_insertion/Aruco_array.h"
#include <iostream>  
#include <sstream> 

void poseCallback(shallow_depth_insertion::Aruco_array msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  
  for (int i=0; i<msg.aruco_array.size(); i++){
    transform.setOrigin(tf::Vector3(msg.aruco_array[i].pose.position.x, msg.aruco_array[i].pose.position.y, msg.aruco_array[i].pose.position.z));
    tf::Quaternion q;
    int tag_id;
    tag_id = msg.aruco_array[i].tag_id;
    q[0] = msg.aruco_array[i].pose.orientation.x;
    q[1] = msg.aruco_array[i].pose.orientation.y;
    q[2] = msg.aruco_array[i].pose.orientation.z;
    q[3] = msg.aruco_array[i].pose.orientation.w;
    transform.setRotation(q);
    std::ostringstream tag;
    tag << tag_id;
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera", "tag_"+tag.str()));
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "aruco_tf_broadcaster");
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/ArucoPose_array", 10, &poseCallback);
  ros::spin();
  return 0;
};