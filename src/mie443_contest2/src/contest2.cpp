#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <limits>
#include <cmath>
#include <algorithm> 

double findDistanceOfPath (std::vector<float> origin, std::vector<std::vector<float>> points){
    double distance = 0;
    distance += sqrt(pow(origin[0] - points[0][0], 2) + pow(origin[1] - points[0][1], 2));
    for (int i = 0; i < points.size() - 1; i++){
        distance += sqrt(pow(points[i][0] - points[i+1][0], 2) + pow(points[i][1] - points[i+1][1], 2));
    }
    distance += sqrt(pow(origin[0] - points[points.size() - 1][0], 2) + pow(origin[1] - points[points.size() - 1][1], 2));
    return distance;
}

std::vector<std::vector<float>> findOptimalPath (std::vector<float> origin, std::vector<std::vector<float>> pointsToVisit){
    std::cout << pointsToVisit[0][0] << " " << pointsToVisit[1][0] << " " << pointsToVisit[2][0] << " " << 
                pointsToVisit[3][0] << " " << pointsToVisit[4][0] << std::endl;
    double minimumDistance = DBL_MAX;
    std::vector<std::vector<float>> optimalPath;
    std::sort (pointsToVisit.begin(), pointsToVisit.end());
    int index =  0;
    do {
        double curDistance = findDistanceOfPath(origin, pointsToVisit);
        //std::cout << "Distance of path "<<index++ << ": "<< curDistance <<std::endl;
        if(curDistance < minimumDistance){
            minimumDistance = curDistance;
            optimalPath = pointsToVisit;
        }
    } while ( std::next_permutation(pointsToVisit.begin(),pointsToVisit.end()));

    std::cout << optimalPath[0][0] << " " << optimalPath[1][0] << " " << optimalPath[2][0] << " " << 
                 optimalPath[3][0] << " " << optimalPath[4][0] << std::endl;

    return optimalPath;
}

/* Params */
float distanceFromBox = 0.25;


std::vector<std::vector<float>> computeTarget(Boxes boxes) {

    std::vector<std::vector<float>> result;

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
        float phiTarget = fmod((phi + 180),360) * M_PI / 180.0 ;

        std::vector<float> newCoords;
        newCoords.push_back(xTarget);
        newCoords.push_back(yTarget);
        newCoords.push_back(phiTarget);

        result.push_back(newCoords);
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

    std::vector<std::vector<float>> targetPoints = computeTarget(boxes);
    std::vector<std::vector<float>> path = findOptimalPath({robotPose.x, robotPose.y, robotPose.phi}, targetPoints);
    path.push_back({robotPose.x, robotPose.y, robotPose.phi});
    int index = 0;
    while(ros::ok()) {
        ros::spinOnce();
        std::cout << " x: " << robotPose.x << " y: " << robotPose.y << " z: " 
            << robotPose.phi << std::endl;
        /***YOUR CODE HERE***/
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi
        std::cout << path[index][0]<< " " <<path[index][1]<< " " <<path[index][2]<<std::endl;
        Navigation::moveToGoal(path[index][0], path[index][1], path[index][2]);
        imagePipeline.getTemplateID(boxes);
        index++;
        ros::Duration(0.01).sleep();
    }
    return 0;
}


