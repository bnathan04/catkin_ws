#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <limits>
#include <cmath>
#include <algorithm> 
#include <iostream>
#include <fstream>

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

std::vector<std::vector<float>> computeTarget(std::vector<std::vector<float>> boxes, float distanceFromBox) {

    std::vector<std::vector<float>> result;

    for(int i = 0; i < boxes.size(); ++i) {
        for (int j = 0; j <= 2; j++){
            float x = boxes[i][0];
            float y = boxes[i][1];
            float phi = boxes[i][2] + (M_PI/9)*j;
            if (j == 2){ //To Ensure we follow the order of straight on, 20 degrees to the right, 20 degrees to the left
                phi = boxes[i][2] - (M_PI/9)*j;
            }

            float xOffset = 0.0;
            float yOffset = 0.0;

            xOffset = (distanceFromBox) * cos(phi);
            yOffset = (distanceFromBox) * sin(phi);
            
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

    /* Params */
    float boxDistance = 0.75;

    std::vector<float> origin{robotPose.x, robotPose.y, robotPose.phi};
    std::vector<std::vector<float>> orderBoxes = findOptimalPath({robotPose.x, robotPose.y, robotPose.phi}, boxes.coords);
    std::vector<std::vector<float>> path = computeTarget(orderBoxes, boxDistance);
    int index = 0;
    int found[3] = {0};
    bool goalFound = false;
    std::ofstream f;
    f.open ("/home/hmnikola/ouputC2.txt");

    while(ros::ok() && index < path.size()) {
        ros::spinOnce();
        std::cout<<"Robot Position: " << " x: " << robotPose.x << " y: " << robotPose.y << " z: " 
            << robotPose.phi << std::endl;

        std::cout << "Curent Goal:"<<path[index][0]<< " " <<path[index][1]<< " " <<path[index][2]<<std::endl;
        goalFound = Navigation::moveToGoal(path[index][0], path[index][1], path[index][2]);
        ros::spinOnce();
        int match = imagePipeline.getTemplateID(boxes);
        std::string boxType = "none";

        switch(match) {
        case 0 :
            boxType = "Raisin Bran";
            break;
        case 1 :
            boxType = "Cinnamon Toast Crunch";
            break;
        case 2 :
            boxType = "Rice Krispies";
            break;
        default :
            boxType = "None";
        }

        if (goalFound){
            if (match >=0 && match <=2){ //found a good match, record it and skip to next box
                if (found[match] == 1){//duplicate match found
                    f << "Found duplicate of "<<boxType<<", (image " << match<<") at location ("<<orderBoxes[index/3][0]<<", "
                    <<orderBoxes[index/3][1]<<", "<<orderBoxes[index/3][2]<<")"<<std::endl;
                }
                else { //first match of box type found
                    f << "Found "<<boxType<<", (image " << match<<") at location("<<orderBoxes[index/3][0]<<", "
                    <<orderBoxes[index/3][1]<<", "<<orderBoxes[index/3][2]<<")"<<std::endl;
                }
                found[match] = 1;
                index += (3 - index%3);
            } else if (match == -2 || index%3 == 2){ //A definite blank box or all 3 angles couldn't find a match
                    f << "Found blank box at position ("<<orderBoxes[index/3][0]<<", "
                    <<orderBoxes[index/3][1]<<", "<<orderBoxes[index/3][2]<<")"<<std::endl;
                    index += (3 - index%3);
            }
            else { //not a great match but not a definite blank either, try a different angle
                index++;
            }
        } else {
            std::vector<float> failedBox = {orderBoxes[index/3][0], orderBoxes[index/3][1], orderBoxes[index/3][2]};
            std::vector<std::vector<float>> failedBoxes;
            failedBoxes.push_back(failedBox);
            std::vector<std::vector<float>> newTargets = computeTarget(failedBoxes, boxDistance - 0.10);
            for (int i = 0; i < newTargets.size(); i++){
                path.push_back(newTargets[i]);
            }
            index += (3 - index%3);
        }
        ros::Duration(0.01).sleep();
    }
    Navigation::moveToGoal(origin[0], origin[1], origin[2]);

    return 0;
}


