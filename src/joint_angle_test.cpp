#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"


ros::Subscriber joint_sub;
sensor_msgs::JointState msg;


std_msgs::Float64 msg_pub;
// ros::Publisher joint_pub;

ros::Publisher joint_eff_pub;
sensor_msgs::JointState msg_eff_pub;

// ==== define the paramters for the cart-pole problem
float m_car = 1.0;
float m_pole = 2.0;
float l = 0.61;
float g = 9.8;
float e_target = m_pole * g * l;
float k_e = 5.0;
float k_p = 5;
float k_d = 5;

float ang_tolerance = 3.14 / 10.0;
float ang_balance = 3.1415926 * 1.5;
float ang_low_lim = ang_balance - ang_tolerance;
float ang_high_lim = ang_balance + ang_tolerance;

void chatterCallback(const sensor_msgs::JointState::ConstPtr& msg)
{

  // calculate the current energy of the pendulum
  float pole_theta_d = -msg->velocity[0];
  float pole_theta = 3.141 / 2.0 -msg->position[0];
  float car_x = msg->position[1];
  float car_x_dot = msg->velocity[1];


  // if energy shapping
  float true_angle = msg->position[0];
  while(true_angle > 2*3.14159){
    true_angle -= 2*3.14159;
  }
  while(true_angle < 0){
    true_angle += 2.0 * 3.14159;
  }

  if( true_angle > ang_low_lim && true_angle < ang_high_lim){
    // range of LQR
    ROS_INFO("LQR");
  }else{
    // range of energy shaping
    ROS_INFO("E-SHAPPING");
    float e_diff = 1.0 / 2.0 * m_pole * l * l * pole_theta_d * pole_theta_d - m_pole * g * l * cos(pole_theta) - e_target - 1.0;
    msg_pub.data = k_e * pole_theta_d * cos(pole_theta) * e_diff - k_p * car_x - k_d * car_x_dot;
    joint_eff_pub.publish(msg_pub);
  }



}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_angle_sensor");

  ros::NodeHandle joint_sub_h;

  joint_sub = joint_sub_h.subscribe("/carpole/joint_states", 1000, chatterCallback);
  // joint_pub = joint_sub_h.advertise<std_msgs::Float64>("/carpole/joint1_position_controller/command", 1000);
  joint_eff_pub = joint_sub_h.advertise<std_msgs::Float64>("/carpole/joint1_effort_controller/command", 100);
  ros::spin();
  return 0;
}
