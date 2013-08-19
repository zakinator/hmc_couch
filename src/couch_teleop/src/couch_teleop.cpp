// %Tag(FULL)%
// %Tag(INCLUDE)%
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
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
  ros::Publisher vel_pub_;
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
  vel_pub_ = nh_.advertise<>("", 1);
// %EndTag(PUB)%
// %Tag(SUB)%
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopTurtle::joyCallback, this);
// %EndTag(SUB)%
}
// %Tag(CALLBACK)%
void TeleopTurtle::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  turtlesim::Velocity vel;
  vel.angular = a_scale_*joy->axes[angular_];
  vel.linear = l_scale_*joy->axes[linear_];
  vel_pub_.publish(vel);
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