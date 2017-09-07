#include <ros/ros.h>
#include "UtmManager.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "utm_manager") ;

  ros::NodeHandle nHandle ;
  
  UtmManager utmManager(nHandle) ;
  
  ros::spin();
  return 0;
}
