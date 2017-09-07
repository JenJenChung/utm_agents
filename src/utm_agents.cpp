#include <ros/ros.h>
#include "Agent.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "utm_agents") ;

  ros::NodeHandle nHandle ;
  
  Agent utmAgent(nHandle) ;
  
  ros::spin();
  return 0;
}
