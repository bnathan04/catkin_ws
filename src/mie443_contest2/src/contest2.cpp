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
float distanceFromBox = 0.8;


std::vector<std::vector<float>> computeTarget(std::vector<std::vector<float>> boxes) {

    std::vector<std::vector<float>> result;

    for(int i = 0; i < boxes.size(); ++i) {
        for (int j = -1; j <= 1; j++){
            float x = boxes[i][0];
            float y = boxes[i][1];
            float phi = boxes[i][2] + (M_PI/6)*j;

            float xOffset = 0.0;
            float yOffset = 0.0;

            xOffset = distanceFromBox * cos(phi);
            yOffset = distanceFromBox * sin(phi);
            
            float xTarget = x + xOffset;
            float yTarget = y + yOffset;
            float phiTarget;
            if (phi >= 0) phiTarget =(phi - M_PI);
            if (phi < 0) phiTarget = (phi + M_PI);

            std::vector<float> newCoords;
            newCoords.push_back(xTarget);
            newCoords.push_back(yTarget);
            newCoords.push_back(phiTarget);

            result.push_back(newCoords);
        }
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

    while (robotPose.x == 0 && robotPose.y == 0 && robotPose.phi == 0){
        ros::spinOnce();
        std::cout <<"Nah"<<std::endl;
    }
    std::vector<std::vector<float>> orderBoxes = findOptimalPath({robotPose.x, robotPose.y, robotPose.phi}, boxes.coords);
    std::vector<std::vector<float>> path = computeTarget(orderBoxes);
    path.push_back({robotPose.x, robotPose.y, robotPose.phi});
    int index = 0;
    while(ros::ok() && index < path.size()) {
        ros::spinOnce();
        std::cout<<"Robot Position: " << " x: " << robotPose.x << " y: " << robotPose.y << " z: " 
            << robotPose.phi << std::endl;
        /***YOUR CODE HERE***/
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi
        std::cout << "Curent Goal:"<<path[index][0]<< " " <<path[index][1]<< " " <<path[index][2]<<std::endl;
        Navigation::moveToGoal(path[index][0], path[index][1], path[index][2]);
        ros::spinOnce();
        imagePipeline.getTemplateID(boxes);
        index++;
        ros::Duration(0.01).sleep();
    }
    return 0;
}


