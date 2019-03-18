
#include <stdio.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/imgproc.hpp"

#include <imagePipeline.h>

using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;
#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "camera/rgb/image_raw"
//camera/rgb/image_raw" // kinect:"camera/rgb/image_raw" webcam:"camera/image"

float computeArea(std::vector<Point2f> scene_corners, cv::Mat img_object) {
    // Might need additional checks
    if (scene_corners.size() < 4) return 0.0;
    auto pointA = scene_corners[0] + Point2f( img_object.cols, 0);
    auto pointB =  scene_corners[1] + Point2f( img_object.cols, 0);
    auto pointC = scene_corners[2] + Point2f( img_object.cols, 0);
    auto pointD = scene_corners[3] + Point2f( img_object.cols, 0);

    auto xMin = fmin(fmin(pointA.x, pointB.x), fmin(pointC.x, pointD.x));
    auto yMin = fmin(fmin(pointA.y, pointB.y), fmin(pointC.y, pointD.y));
    auto xMax = fmax(fmax(pointA.x, pointB.x), fmax(pointC.x, pointD.x));
    auto yMax = fmax(fmax(pointA.y, pointB.y), fmax(pointC.y, pointD.y));


    float s1 = xMax - xMin;
    float s2 = yMax - yMin;

    float area = s1 * s2; 

    return area;
}

int compareImages(cv::Mat img_scene, cv::Mat img_object, float& area) {
    //-- Step 1 & 2: Detect the keypoints and calculate descriptors using SURF Detector
    int minHessian = 400;
    Ptr<SURF> detector = SURF::create(minHessian);
    vector<KeyPoint> keypoints_object, keypoints_scene;
    Mat descriptors_object, descriptors_scene;
    detector->detectAndCompute(img_object, Mat(), keypoints_object,
    descriptors_object);
    detector->detectAndCompute(img_scene, Mat(), keypoints_scene,
    descriptors_scene);
    cout << "Step 1 and 2 Done" << std::endl;
    //-- Step 3: Matching descriptor vectors using FLANN matcher
    BFMatcher matcher;
    std::vector< DMatch > matches;
    matcher.match( descriptors_object, descriptors_scene, matches );

    double max_dist = 0; double min_dist = 100;
    cout << "Step 3 Done" << std::endl;
    //-- Quick calculation of max and min distances between keypoints
    for( int i = 0; i < descriptors_object.rows; i++ )
    { double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }

    printf("-- Max dist : %f \n", max_dist );
    printf("-- Min dist : %f \n", min_dist );
    cout << "Step 4 Done" << std::endl;
    //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
    std::vector< DMatch > good_matches;

    for( int i = 0; i < descriptors_object.rows; i++ )
    { if( matches[i].distance < 3*min_dist )
        { good_matches.push_back( matches[i]); }
    }

    Mat img_matches;
    drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
                good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    //-- Localize the object
    std::vector<Point2f> obj;
    std::vector<Point2f> scene;
    cout << "Step 5 Done" << std::endl;
    for( int i = 0; i < good_matches.size(); i++ )
    {
        //-- Get the keypoints from the good matches
        obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
        scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
    }


    if (obj.size() < 4  || scene.size() < 4) {
       cout << "WE HERE" << endl;
        return 0;
    }
    Mat H = findHomography( obj, scene, RANSAC );

    //-- Get the corners from the image_1 ( the object to be "detected" )
    std::vector<Point2f> obj_corners(4);
    obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( img_object.cols, 0 );
    obj_corners[2] = cvPoint( img_object.cols, img_object.rows ); obj_corners[3] = cvPoint( 0, img_object.rows );
    std::vector<Point2f> scene_corners(4);

    cout << "OBJ: " << obj.size() << endl;
    cout << "SCENE: " << scene.size() << endl;

    if (H.empty()) return 0;

    perspectiveTransform( obj_corners, scene_corners, H);

    //-- Draw lines between the corners (the mapped object in the scene - image_2 )
    line( img_matches, scene_corners[0] + Point2f( img_object.cols, 0), scene_corners[1] + Point2f( img_object.cols, 0), Scalar(0, 255, 0), 4 );
    line( img_matches, scene_corners[1] + Point2f( img_object.cols, 0), scene_corners[2] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
    line( img_matches, scene_corners[2] + Point2f( img_object.cols, 0), scene_corners[3] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
    line( img_matches, scene_corners[3] + Point2f( img_object.cols, 0), scene_corners[0] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );

    area = computeArea(scene_corners, img_object);

    //-- Show detected matches
    imshow( "Good Matches & Object detection", img_matches );

    waitKey(0);
    return good_matches.size();
}

//   /** @function readme */
//   void readme()
//   { std::cout << " Usage: ./SURF_descriptor <img1> <img2>" << std::endl; }

ImagePipeline::ImagePipeline(ros::NodeHandle& n) {
    image_transport::ImageTransport it(n);
    sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::imageCallback, this);
    isValid = false;
}

void ImagePipeline::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        if(isValid) {
            img.release();
        }
        img = (cv_bridge::toCvShare(msg, IMAGE_TYPE)->image).clone();
        isValid = true;
    } catch (cv_bridge::Exception& e) {
        std::cout << "ERROR: Could not convert from " << msg->encoding.c_str()
                  << " to " << IMAGE_TYPE.c_str() << "!" << std::endl;
        isValid = false;
    }    
}

int ImagePipeline::getTemplateID(Boxes& boxes) {

    cv::Mat image_array_1 = imread("/home/hmnikola/catkin_ws/src/mie443_contest2/boxes_database/template1.jpg",IMREAD_GRAYSCALE);
    cv::Mat image_array_2 = imread("/home/hmnikola/catkin_ws/src/mie443_contest2/boxes_database/template2.jpg",IMREAD_GRAYSCALE);
    cv::Mat image_array_3 = imread("/home/hmnikola/catkin_ws/src/mie443_contest2/boxes_database/template3.jpg",IMREAD_GRAYSCALE);

    int template_id = -1;
    if(!isValid) {
        std::cout << "ERROR: INVALID IMAGE!" << std::endl;
    } else if(img.empty() || img.rows <= 0 || img.cols <= 0) {
        std::cout << "ERROR: VALID IMAGE, BUT STILL A PROBLEM EXISTS!" << std::endl;
        std::cout << "img.empty():" << img.empty() << std::endl;
        std::cout << "img.rows:" << img.rows << std::endl;
        std::cout << "img.cols:" << img.cols << std::endl;
    } else {
        /***YOUR CODE HERE***/
        float rectArea = 0.0;
        int checkImage = compareImages(img, image_array_1, rectArea); //need to fix this
        cout << "Matches: " << checkImage << " Area: " << rectArea << endl;
        if(rectArea > 5000.0 && rectArea < 20000.0){
            cout << "It's image 1 tho" << std::endl;
        }
        checkImage = compareImages(img, image_array_2, rectArea); //need to fix this
        cout << "Matches: " << checkImage << " Area: " << rectArea << endl;
        if(rectArea > 5000.0 && rectArea < 20000.0){
            cout << "It's image 2 tho" << std::endl;
        }
        checkImage = compareImages(img, image_array_3, rectArea); //need to fix this
        cout << "Matches: " << checkImage << " Area: " << rectArea << endl;
        if(rectArea > 5000.0 && rectArea < 20000.0){
            cout << "It's image 3 tho" << std::endl;
        }


        // Use: boxes.templates 
        cv::imshow("view", img);
        cv::waitKey(10);
    }  
    return template_id;
}
