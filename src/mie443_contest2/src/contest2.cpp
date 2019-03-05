#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>

/* Our includes */
#include <cmath.h>

/* Params */
float distanceFromBox = 2.50;


std::vector<vector<float>> computeTarget(Boxes) {

    std::vector<vector<float>> result;

    for(int i = 0; i < boxes.coords.size(); ++i) {
        float x = boxes.coords[i][0];
        float y = boxes.coords[i][1];
        float phi = boxes.coords[i][2] * 180.0 / M_PI;

        float xOffset = 0.0;
        float yOffset = 0.0;

        if (phi <= 90.0 && phi > 0.0) {
            xOffset = distanceFromBox * cos(phi);
            yOffset = distanceFromBox * sin(phi);
        } 

        else if (phi <= 180.0 && phi > 90.0) {
            xOffset = distanceFromBox * sin(phi-90.0) * -1.0;
            yOffset = distanceFromBox * cos(phi-90.0);
        }

        else if (phi <=270.0 && phi >  180.0) {
            xOffset = -1.0 * distanceFromBox * cos(phi-180.0);
            yOffset = -1.0 * distanceFromBox * sin(phi-180.0);
        }

        else {
            xOffset = distanceFromBox * sin(phi-270.0);
            yOffset = -1.0 * distanceFromBox * cos(phi-270.0);
        }  
           
        float xTarget = x + xOffset;
        float yTarget = y + yOffset;
        float phiTarget = ((phi + 180)%360) * M_PI / 180.0 ;

        vector<float> newCoords;
        newCoords.push_back(xTarget);
        newCoords.push_back(yTarget);
        newCoords.push_back(phiTarget);

        result.push_back(newCoords)
    }

    return result;

}


int main(int argc, char** argv) {
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;
    // Robot pose object + subscriber.
    RobotPose robotPose(0,0,0);
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);
    // Initialize box coordinates and templates
    Boxes boxes; 
    if(!boxes.load_coords() || !boxes.load_templates()) {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }
    for(int i = 0; i < boxes.coords.size(); ++i) {
        std::cout << "Box coordinates: " << std::endl;
        std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " z: " 
                  << boxes.coords[i][2] << std::endl;
    }
    // Initialize image objectand subscriber.
    ImagePipeline imagePipeline(n);
    // Execute strategy.

    std::vector<vector<float>> targetPoints = computeTarget(Boxes);


    while(ros::ok()) {
        ros::spinOnce();
        /***YOUR CODE HERE***/
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi




        imagePipeline.getTemplateID(boxes);
        ros::Duration(0.01).sleep();
    }
    return 0;
}
