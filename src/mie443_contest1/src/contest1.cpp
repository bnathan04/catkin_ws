#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <eStop.h>

#include <stdio.h>
#include <cmath>

#include <iostream>
#include <chrono>

using namespace std;

double angular = 0.0, linear = 0.0;
double posX, posY, yaw, angleAtHit, currentYaw;
double pi = 3.1416;
bool bumperLeft = false, bumperCenter = false, bumperRight = false;

double laserLeft = 10, laserCentre = 10, laserRight = 10, laserRange=10;
int laserSize=0, laserOffset=0, desiredAngle=5;


void bumperCallback(const kobuki_msgs::BumperEvent msg){
	if (msg.bumper == 0){
		bumperLeft = true;
		angleAtHit = yaw;
	}
	else if (msg.bumper == 1){
		bumperCenter = true;
		angleAtHit = yaw;
	} 
	else if (msg.bumper == 2){
		bumperRight = true;
		angleAtHit = yaw;
	}
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	int iStart,iEnd;
	laserSize = (msg->angle_max - msg->angle_min)/msg->angle_increment;
	laserOffset = desiredAngle*pi/(180*msg->angle_increment);
	
	if(desiredAngle*pi/180 < msg->angle_max && -desiredAngle*pi/180 > msg->angle_min){ 
		iStart = laserSize/2 - laserOffset;
		iEnd = laserSize/2 + laserOffset;
	} else {
		iStart = 0;
		iEnd = laserSize;
	}

	laserRange = 999.9;
	laserCentre = 999.9;
	laserRight = 999.9;
	laserLeft = 999.9;
	for(int i =0; i < laserSize; i++){
		if (i < iStart && msg->ranges[i] < laserLeft){ //left sensors
			laserLeft = msg->ranges[i];
		} else if (i >= iStart && i < iEnd &&  msg->ranges[i] < laserCentre){ //center sensors
			laserCentre = msg->ranges[i];
		} else if (i >= iEnd && msg->ranges[i] < laserRight){ // right sensors
			laserRight = msg->ranges[i];
		}

		if(laserRange > msg->ranges[i])
			laserRange = msg->ranges[i];
	}
	
	if(laserRange == 11)
		laserRange = 0;

	ROS_INFO("Laser Left: %f\n", laserLeft);
	ROS_INFO("Laser Centre: %f\n", laserCentre);
	ROS_INFO("Laser Centre: %f\n", laserRight);
}

void infoRotate(){
	while (fabs(currentYaw) > (yaw -pi/6) ){
		angular = -pi/6;
		linear = 0.0;
	} 
	while (fabs(yaw) <  (currentYaw + pi/6) ){
		angular = pi/6;
		linear = 0.0;
	} 
	while (fabs(yaw) >=  (currentYaw) ){
		angular = -pi/6;
		linear = 0.0;
	} 
}

void bumperHit(){
	if(fabs(yaw-angleAtHit) < pi/2){
		angular = pi/6;
		linear = 0;
	} else{
		bumperCenter=0;
		bumperLeft=0;
		bumperRight=0;
	}
}


void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	posX = msg->pose.pose.position.x;
	posY = msg->pose.pose.position.y;
	yaw = tf::getYaw(msg->pose.pose.orientation);

	//ROS_INFO("Position: (%f,%f) Orientation: %f rad or %f degrees. ",posX, posY, yaw, yaw*180/pi);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	teleController eStop;

	ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
	ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
	ros::Subscriber odom= nh.subscribe("odom", 1, odomCallback); 

	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

	// double angular = 0.0;
	// double linear = 0.0;
	geometry_msgs::Twist vel;

	std::chrono::time_point<std::chrono::system_clock> start;
	start = std::chrono::system_clock::now(); /* start timer */
    uint64_t secondsElapsed = 0; // the timer just started, so we know it is less than 480, no need to check.

	while(ros::ok() && secondsElapsed <= 480)
    {
		ros::spinOnce();
		//.....**E-STOP DO NOT TOUCH**.......
		eStop.block();
		//...................................
		//ROS_INFO("Position: (%f, %f) Orientation: %f degrees Range: %f", posX, posY, yaw*180/pi, laserRange);

		// ROS_INFO(">>>>>>>>>>>>LASERCENTRE MAIN: %f\n", laserCentre);
		currentYaw = yaw;
		if (bumperCenter || bumperLeft || bumperRight){
			bumperHit();
		} else if (laserLeft < 0.5) {
			// keep going
			linear = 0;
			angular = pi/6;
		} else if (laserRight < 0.5) {
			// keep going
			linear = 0;
			angular = -pi/6;
		} else if (laserCentre > 0.8) {
			// keep going
			linear = 0.2;
			angular = 0.0;

		}else if (laserCentre < 0.8 && laserRight<laserCentre && laserLeft<laserCentre) {
			// keep going
			bumperCenter = true;
			angleAtHit = yaw;
			infoRotate();
			
		} else if (laserLeft < laserRight) {
			linear = 0.1;
			angular = pi/3;
		} else if (laserRight <= laserLeft) {
			linear = 0.1;
			angular = -pi/3;
		} 
		else 
		{
			angular = 0.0;
			linear = 0.0;
		} 

  		vel.angular.z = angular;
  		vel.linear.x = linear;

  		vel_pub.publish(vel);

        // The last thing to do is to update the timer.
		secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
	}

	return 0;
}
