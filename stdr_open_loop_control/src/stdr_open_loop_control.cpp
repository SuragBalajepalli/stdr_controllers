#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
//node to send Twist commands to the Simple 2-Dimensional Robot Simulator via cmd_vel
int main(int argc, char **argv) {
    ros::init(argc, argv, "stdr_commander"); 
    ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS
    ros::Publisher twist_commander = n.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1);
    //some "magic numbers"
    double sample_dt = 0.01; //specify a sample period of 10ms  
    double speed = 1.0; // 1m/s speed command
    double yaw_rate = 1; //1rad/sec yaw rate command
    double time = 0; // simple method to implement distance/angle moved
	char dir;
	int moves=0;    
      
    geometry_msgs::Twist twist_cmd; //this is the message type required to send twist commands to STDR 
    // start with all zeros in the command message; should be the case by default, but just to be safe..
    twist_cmd.linear.x=0.0;
    twist_cmd.linear.y=0.0;    
    twist_cmd.linear.z=0.0;
    twist_cmd.angular.x=0.0;
    twist_cmd.angular.y=0.0;
    twist_cmd.angular.z=0.0;   
	

    ros::Rate loop_timer(1/sample_dt); //create a ros object from the ros “Rate” class; set 100Hz rate     
    double timer=0.0;
    //start sending some zero-velocity commands, just to warm up communications with STDR
    for (int i=0;i<10;i++) {
      twist_commander.publish(twist_cmd);
      loop_timer.sleep();
    }
	
	while(1){	
	std::cout<<"left/right/front?"; //Take direction input from the user
	std::cin>>dir;
	std::cout<<"how much?"; // Take magnitude input from the user
	std::cin>>time;
    
	switch (dir) {
		case 'l':
		twist_cmd.angular.z=yaw_rate; //and start spinning in place    
		break;
		case 'r':
		twist_cmd.angular.z=-yaw_rate; //and start spinning in place
		break;	
		case 'f':
		twist_cmd.linear.x=speed; // move forward
		break;	
		}
	while(timer<time) {
          twist_commander.publish(twist_cmd);
          timer+=sample_dt;
          loop_timer.sleep();
          }
    twist_cmd.linear.x=0.0; //stop moving forward
    twist_cmd.angular.z=0; //and start spinning in place	
	for (int i=0;i<10;i++) {
    twist_commander.publish(twist_cmd);
    loop_timer.sleep();
    }    
	timer=0.0; //reset the timer
   	moves++;

	}	
    //halt the motion
    twist_cmd.angular.z=0.0; 
    twist_cmd.linear.x=0.0; 
    for (int i=0;i<10;i++) {
      twist_commander.publish(twist_cmd);
      loop_timer.sleep();
    }               
    //done commanding the robot; node runs to completion
}

