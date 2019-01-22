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

double angular, linear;
double posX, posY, yaw;
double angleAtHit;
double pi = 3.1416;
bool bumperLeft = 0, bumperCenter = 0, bumperRight = 0;


void bumperCallback(const kobuki_msgs::BumperEvent msg){
	if (msg.bumper == 0){
		bumperLeft = 1;
		angleAtHit = yaw;
	}
	else if (msg.bumper == 1){
		bumperCenter = 1;
		angleAtHit = yaw;
	} 
	else if (msg.bumper == 2){
		bumperRight = 1;
		angleAtHit = yaw;
	}
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	//fill with your code
}

void odomCallback(const navmsgs::Odometry::ConstPtr& msg){
	posX = msg->pose.pose.position.x;
	posY = msg->pose.pose.position.y;
	yaw = tf::getYaw(msg->pose.pose.orientation);

	ROS_INFO("Position: (%f,%f) Orientation: %f rad or %f degrees. ",posX, posY, yaw, yaw*180/pi);
}

void bumperHit(){
	if (abs(yaw - angleAtHit) < pi/2){
		angular = pi/6;
		linear = 0.0;
	} else {
		bumperCenter = 0;
		bumperLeft = 0;
		bumperRight = 0;
	}
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

	double angular = 0.0;
	double linear = 0.0;
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

		if (!bumperRight && !bumperCenter && !bumperLeft){
			angular = 0.0;
			linear = 0.1;
		} else if (bumperCenter == 1 || bumperLeft == 1 || bumperRight == 1){
			bumperHit();
		} else {
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
