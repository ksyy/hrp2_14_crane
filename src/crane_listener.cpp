#include <ros/ros.h>
#include <std_msgs/String.h>
#include "crane_message_handler.hh"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "crane_control");
  ros::NodeHandle n;
 
  CraneMessageHandler craneMsgHandler; 
  ros::Subscriber sub = n.subscribe("joy", 10, &CraneMessageHandler::joystickCallback, &craneMsgHandler); 

  ros::Rate loop_rate(50);
  while (ros::ok())
  {
    ros::spinOnce();
    craneMsgHandler.applyControlStrategy();
    loop_rate.sleep();
  }

  return 0;
}
