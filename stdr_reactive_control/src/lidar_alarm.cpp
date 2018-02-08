#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message 


const double MIN_SAFE_DISTANCE = 1.0; // set alarm if anything is within 1m of the front of robot

// these values to be set within the laser callback
int mid_index= -1; // NOT real; callback will have to find this
double angle_min_=0.0;
double angle_max_=0.0;
double angle_increment_=0.0;
double range_min_ = 0.0;
double range_max_ = 0.0;
bool left_laser_alarm=false;
bool right_laser_alarm=false;

ros::Publisher left_lidar_alarm_publisher;
ros::Publisher right_lidar_alarm_publisher;


void laserCallback(const sensor_msgs::LaserScan& laser_scan) {
    if (mid_index<0)  {
        //for first message received, set up the desired index of LIDAR range to eval
        angle_min_ = laser_scan.angle_min;
        angle_max_ = laser_scan.angle_max;
        angle_increment_ = laser_scan.angle_increment;
        range_min_ = laser_scan.range_min;
        range_max_ = laser_scan.range_max;
        mid_index = (int) ((0.0 -angle_min_)/angle_increment_);
               
    }
	left_laser_alarm=false;
	right_laser_alarm=false;	
    
   for (int i=mid_index-100; i<mid_index+100;i++) { //corridor of scanning
    
      if (laser_scan.ranges[i]<MIN_SAFE_DISTANCE) {
         if(i>mid_index) {
         left_laser_alarm =true;      
        }
        else  {
         right_laser_alarm=true;
	
        }
      }

    }
   
   std_msgs::Bool left_lidar_alarm_msg, right_lidar_alarm_msg;
   left_lidar_alarm_msg.data = left_laser_alarm;
   right_lidar_alarm_msg.data = right_laser_alarm;
   left_lidar_alarm_publisher.publish(left_lidar_alarm_msg);
   right_lidar_alarm_publisher.publish(right_lidar_alarm_msg);
      
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_alarm"); //name this node
    ros::NodeHandle nh; 
    //create a Subscriber object and have it subscribe to the lidar topic
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("left_lidar_alarm", 1);
    left_lidar_alarm_publisher = pub; // let's make this global, so callback can use it
    ros::Publisher pub2 = nh.advertise<std_msgs::Bool>("right_lidar_alarm", 1);  
    right_lidar_alarm_publisher = pub2;
    ros::Subscriber lidar_subscriber = nh.subscribe("robot0/laser_0", 1, laserCallback);
    ros::spin(); //this is essentially a "while(1)" statement, except it
    // forces refreshing wakeups upon new data arrival
    // main program essentially hangs here, but it must stay alive to keep the callback function alive
    return 0; // should never get here, unless roscore dies
}
