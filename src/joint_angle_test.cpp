#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
sensor_msgs::JointState msg;

void chatterCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  // ROS_INFO("I heard: [%s]", msg.name[0].c_str());
  ROS_INFO("pos:[%f]  vel:[%f]  effort:[%f]", msg->position[0], msg->velocity[0], msg->effort[0]);
  // ROS_INFO("GET");
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_angle_sensor");
  ros::NodeHandle joint_sub_h;
  ros::Subscriber joint_sub = joint_sub_h.subscribe("/carpole/joint_states", 1000, chatterCallback);
  ros::spin();
  return 0;
}
