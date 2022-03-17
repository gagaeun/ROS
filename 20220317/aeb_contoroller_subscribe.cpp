#include "sensor_msgs/Range.h"
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h> 
#include <sensor_msgs/Range.h>


void UltraSonarCallback(const std_msgs::Bool::ConstPtr& msg){
	if(msg->data)
		ROS_INFO("ACTIVE!!");
	else
		ROS_INFO("FAIL!!");
}

int main(int argc, char **argv){
  
 ros :: init(argc, argv, "aeb_controller2");
	
	ros ::NodeHandle n;
	
	ros ::Rate loop_rate(1);
 
  ros::Subscriber sub = n.subscribe("range", 1000, UltraSonarCallback);

  ros::spin();
  
  while(ros::ok()){   
		loop_rate.sleep();
		ros::spinOnce();
		
	}

  return 0;
}