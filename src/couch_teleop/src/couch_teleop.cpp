// %Tag(FULL)%
// %Tag(INCLUDE)%
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "couch_control/MotorCommand.h"
// %EndTag(INCLUDE)%
// %Tag(CLASSDEF)%
class CouchTeleop
{
public:
  CouchTeleop();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_;
  ros::Publisher cmd_pub_;
  ros::Subscriber joy_sub_;
  
};
// %EndTag(CLASSDEF)%
// %Tag(PARAMS)%
CouchTeleop::CouchTeleop():
  linear_(1),
  angular_(2)
{
  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);
// %EndTag(PARAMS)%
// %Tag(PUB)%
  cmd_pub_ = nh_.advertise<couch_control::MotorCommand>("/test", 1);
// %EndTag(PUB)%
// %Tag(SUB)%
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &CouchTeleop::joyCallback, this);
// %EndTag(SUB)%
}
// %Tag(CALLBACK)%
void CouchTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  couch_control::MotorCommand cmd;
  cmd.left = 300 * a_scale_*joy->axes[angular_];
  cmd.right = 300 * l_scale_*joy->axes[linear_];
  cmd_pub_.publish(cmd);
}
// %EndTag(CALLBACK)%
// %Tag(MAIN)%
int main(int argc, char** argv)
{
  ros::init(argc, argv, "couch_teleop");
  CouchTeleop couch_teleop;

  ros::spin();
}
// %EndTag(MAIN)%
// %EndTag(FULL)%
