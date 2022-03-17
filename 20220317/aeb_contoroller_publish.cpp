#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h> 
#include <sensor_msgs/Range.h>   

std_msgs::Bool flag_AEB;

void UltraSonarCallback(const sensor_msgs::Range::ConstPtr& msg){

  ROS_INFO("Sonar Range: [%f]", msg->range);
  
  
	if(msg->range <=1.0){
		ROS_INFO("ACTIVE!!");
		flag_AEB.data=true;
	}
	else{
		flag_AEB.data =false;
	}
}

int main(int argc, char **argv){
	int count =0;
	
	ros :: init(argc, argv, "aeb_controller");
	
	ros ::NodeHandle n;
	
	ros ::Rate loop_rate(1);
	
	ros ::Subscriber sub = n.subscribe("range", 1000, UltraSonarCallback);
	
	ros::Publisher pub1 = n.advertise<std_msgs::Bool>("range2", 1000);
	
	while(ros::ok()){   
		sensor_msgs::Range msg;
		
		
		pub1.publish(flag_AEB);
		
		loop_rate.sleep();
		ros::spinOnce();
		++count;
	}
	return 0;
}