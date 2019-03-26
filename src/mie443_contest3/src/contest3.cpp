#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <kobuki_msgs/WheelDropEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <kobuki_msgs/BumperEvent.h>

using namespace std;

enum Emotion { NEUTRAL, SAD, ANGER};

geometry_msgs::Twist follow_cmd;
int world_state = 0;
double laserLeft = 10, laserCentre = 10, laserRight = 10, laserRange=10;
int laserSize=0, laserOffset=0, desiredAngle=5;
double pi = 3.1416;
Emotion emotion_state = NEUTRAL;



string filePath;


// void parseEmotion(sound_play::SoundClient sc, Emotion emotion) {

// 	switch (emotion)
// 	{
// 		case SAD:
// 			sc.playWave(filePath+"sound.wav");
// 			break;

// 		case ANGER:
// 			sc.playWave(filePath+"sound.wav");
// 			break;

// 		case NEUTRAL:
// 			world_state = 0;
// 			break;

// 		default:
// 			break;
// 	}

// 	emotion_state = NEUTRAL;
// 	ros::Duration(5).sleep();

// }

void followerCB(const geometry_msgs::Twist msg){
    follow_cmd = msg;
}

void bumperCB(const kobuki_msgs::BumperEvent msg){

	if (msg.state) {
		
		std::cout << "IM CHEESED" << std::endl;

		world_state = 1;
		emotion_state = ANGER;

	} else if (!msg.state) {

		world_state = 0;

	}
}

void wheelDropCB(const kobuki_msgs::WheelDropEvent msg){
    if (msg.state) {
		
		world_state = 1;
		emotion_state = SAD;

	} else if (!msg.state) {

		world_state = 0;

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

	if(laserCentre > 1)
		world_state = 2;
	else
		world_state = 0;

	std::cout<<laserCentre<<std::endl;
}

//-------------------------------------------------------------

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	sound_play::SoundClient sc;
	string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
	filePath = path_to_sounds;
	teleController eStop;

	//publishers
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",1);

	//subscribers
	ros::Subscriber follower = nh.subscribe("follower_velocity_smoother/smooth_cmd_vel", 10, &followerCB);
	ros::Subscriber bumper = nh.subscribe("mobile_base/events/bumper", 10, &bumperCB);
	ros::Subscriber wheelDrop = nh.subscribe("mobile_base/events/wheel_drop", 10, &wheelDropCB);
	ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);

	imageTransporter rgbTransport("camera/image/", sensor_msgs::image_encodings::BGR8); //--for Webcam
	//imageTransporter rgbTransport("camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8); //--for turtlebot Camera
	imageTransporter depthTransport("camera/depth_registered/image_raw", sensor_msgs::image_encodings::TYPE_32FC1);

	double angular = 0.2;
	double linear = 0.0;

	geometry_msgs::Twist vel;
	vel.angular.z = angular;
	vel.linear.x = linear;

	sc.playWave(path_to_sounds + "sound.wav");
	ros::Duration(0.5).sleep();

	while(ros::ok()){
		ros::spinOnce();
		//.....**E-STOP DO NOT TOUCH**.......
		//eStop.block();
		//...................................

		if(world_state == 0){
			//fill with your code
			//vel_pub.publish(vel);
			vel_pub.publish(follow_cmd);

		}else if(world_state == 1){ // sensor stimuli 

			switch (emotion_state)
			{
				case SAD:
					sc.playWave(path_to_sounds+"sound.wav");
					ros::Duration(5).sleep();
					break;

				case ANGER:
					sc.playWave(path_to_sounds+"sound.wav");
					ros::Duration(5).sleep();
					break;

				case NEUTRAL:
					world_state = 0;
					break;

				default:
					break;
			}

			emotion_state = NEUTRAL;
			

		} else if(world_state == 2){// I lost my follower
			
			sc.playWave(path_to_sounds+"sound.wav");
			ros::Duration(5).sleep();
			
		}

	}

	return 0;
}
