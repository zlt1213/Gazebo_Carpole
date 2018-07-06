#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"



ros::Subscriber joint_sub;
sensor_msgs::JointState msg;


std_msgs::Float64 msg_pub;
ros::Publisher joint_pub;

void chatterCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  ros::Time c_time = ros::Time::now();
  double secs = c_time.toSec();

  ROS_INFO("pos:[%f]  vel:[%f]  effort:[%f]", msg->position[0], msg->velocity[0], secs );

  msg_pub.data = sin(10 * secs) * 0.3;
  joint_pub.publish(msg_pub);


  // if(msg->position[0] > 0.2 || msg->position[0] <-0.3){
  //   msg_pub.data = 0.0;
  //   joint_pub.publish(msg_pub);
  //
  // }else{
  //   msg_pub.data  = msg->position[0] + 0.01;
  //   joint_pub.publish(msg_pub);
  // }




}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_angle_sensor");

  ros::NodeHandle joint_sub_h;

  joint_sub = joint_sub_h.subscribe("/carpole/joint_states", 1000, chatterCallback);
  joint_pub = joint_sub_h.advertise<std_msgs::Float64>("/carpole/joint1_position_controller/command", 1000);
  ros::spin();
  return 0;
}
