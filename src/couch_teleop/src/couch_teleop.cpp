// %Tag(FULL)%
// %Tag(INCLUDE)%
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "couch_control/MotorCommand.h"
#include <algorithm>
// %EndTag(INCLUDE)%
// %Tag(CLASSDEF)%
class CouchTeleop
{
public:
  CouchTeleop();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void timerCallback(const ros::TimerEvent& timer);
  
  ros::NodeHandle nh_;

  double x_scale_, y_scale_;
  ros::Publisher cmd_pub_;
  ros::Subscriber joy_sub_;
  
  ros::Timer joy_timer_;

  double xaxis = 0;
  double yaxis = 0;

  double alpha = 0.1;
  double left = 0;
  double right = 0;
};
// %EndTag(CLASSDEF)%
// %Tag(PARAMS)%
CouchTeleop::CouchTeleop()
{
  nh_.param("x_scale", x_scale_, 1.0);
  nh_.param("y_scale", y_scale_, 1.0);
// %EndTag(PARAMS)%
// %Tag(PUB)%
  cmd_pub_ = nh_.advertise<couch_control::MotorCommand>("/couchMotors", 1);
// %EndTag(PUB)%
// %Tag(SUB)%
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &CouchTeleop::joyCallback, this);
// %EndTag(SUB)%
  joy_timer_ = nh_.createTimer(ros::Duration(0.02), &CouchTeleop::timerCallback, this);
}
// %Tag(CALLBACK)%
void CouchTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  //ROS_ERROR("joy!");
  xaxis = joy->axes[0] * x_scale_;
  yaxis = joy->axes[1] * y_scale_;

}
// %Tag(CALLBACK)%
void CouchTeleop::timerCallback(const ros::TimerEvent& timer)
{  
  //ROS_ERROR("timer!");
  double newLeft = (yaxis - xaxis) * 300.0;
  double newRight = (yaxis + xaxis) * 300.0;


  left = alpha*newLeft + (1-alpha)*left;
  right = alpha*newRight + (1-alpha)*right;

  couch_control::MotorCommand cmd;
  cmd.left = left;
  cmd.right = right;
  cmd.left = std::max<double>(std::min<double>(cmd.left,300.0),-300.0);
  cmd.right = std::max<double>(std::min<double>(cmd.right,300.0),-300.0);
  //cmd.left = 300.0 * a_scale_*joy->axes[angular_];
  //cmd.right = 300.0 * l_scale_*joy->axes[linear_];
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
