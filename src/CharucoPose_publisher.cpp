#include "opencv2/opencv.hpp"
#include <opencv2/aruco.hpp>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include "shallow_depth_insertion_v2/getCharucoPose.h"
#include "ros/ros.h"
#include <opencv2/aruco/charuco.hpp>
#include <vector>
#include <iostream>

using namespace std;
using namespace cv;

VideoCapture cap(1); // open the default camera

namespace {
const char* about = "Pose estimation using a ChArUco board";
const char* keys  =
        "{w        |       | Number of squares in X direction }"
        "{h        |       | Number of squares in Y direction }"
        "{sl       |       | Square side length (in meters) }"
        "{ml       |       | Marker side length (in meters) }"
        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
        "{c        |       | Output file with calibrated camera parameters }"
        "{v        |       | Input from video file, if ommited, input comes from camera }"
        "{ci       | 0     | Camera id if input doesnt come from video (-v) }"
        "{dp       |       | File of marker detector parameters }"
        "{rs       |       | Apply refind strategy }"
        "{r        |       | show rejected candidates too }";
}

// Read camera parameters for aruco
static bool readCameraParameters(std::string filename, cv::Mat &camMatrix, cv::Mat &distCoeffs) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}

// Read detector parameters for aruco
static bool readDetectorParameters(std::string filename, cv::Ptr<cv::aruco::DetectorParameters> &params) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if(!fs.isOpened()){
        std::cout << "NOT OPENED" << std::endl;
        return false;
    }
    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params->errorCorrectionRate;
    return true;
}

// Read Charuco parameters 
static bool readCharucoParameters(std::string filename, int &squaresX, int &squaresY, float &squareLength, float &markerLength) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["squaresX"] >> squaresX;
    fs["squaresY"] >> squaresY;
    fs["squareLength"] >> squareLength;
    fs["markerLength"] >> markerLength;
    return true;
}

// Service callback function: detect charuco board and return pose  
bool getCharucoPose()
{
  int squaresX, squaresY;
  float squareLength, markerLength;
  readCharucoParameters("src/shallow_depth_insertion_v2/config/charuco_param.yaml", squaresX, squaresY, squareLength, markerLength);
    
  int dictionaryId = 0;
  bool showRejected = 0;
  bool refindStrategy = 1;
  int margins = squareLength - markerLength;
  int borderBits = 1;
  bool showImage = 1;

  // Specify aruco tag dictionary id
  cv::Ptr<cv::aruco::Dictionary> dictionary =
    cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

  // Read detector parameters from config/yaml
  cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
  bool readOk = readDetectorParameters("src/shallow_depth_insertion_v2/config/detector_params.yaml", detectorParams);
  if(!readOk) {
    std::cerr << "Invalid detector parameters file" << std::endl;
    return 0;
  }
  
  // Read camera parameters from config/yaml
  cv::Mat camMatrix, distCoeffs;
  readOk = readCameraParameters("src/shallow_depth_insertion_v2/config/camera_param.yaml", camMatrix, distCoeffs);
  if(!readOk) {
    std::cerr << "Invalid camera file" << std::endl;
    return 0;
  }

  float axisLength = 0.5f * ((float)min(squaresX, squaresY) * (squareLength));    
  // create charuco board object
  Ptr<aruco::CharucoBoard> charucoboard =
    aruco::CharucoBoard::create(squaresX, squaresY, squareLength, markerLength, dictionary);
  Ptr<aruco::Board> board = charucoboard.staticCast<aruco::Board>();


  while (ros::ok())
  {
  
    // Get latest image from camera 
    cv::Mat image, imageCopy;
    cap >> image; // get a new frame from camera

    vector< int > markerIds, charucoIds;
    vector< vector< Point2f > > markerCorners, rejectedMarkers;
    vector< Point2f > charucoCorners;
    Vec3d rvec, tvec;
    // detect markers
    aruco::detectMarkers(image, dictionary, markerCorners, markerIds, detectorParams, rejectedMarkers);

    // refind strategy to detect more markers
    if(refindStrategy)
      aruco::refineDetectedMarkers(image, board, markerCorners, markerIds, rejectedMarkers,
                                           camMatrix, distCoeffs);

    // interpolate charuco corners
    int interpolatedCorners = 0;
    if(markerIds.size() > 0)
      interpolatedCorners = aruco::interpolateCornersCharuco(markerCorners, markerIds, image, charucoboard,
                                                   charucoCorners, charucoIds, camMatrix, distCoeffs);

    // estimate charuco board pose
    bool validPose = false;
    if(camMatrix.total() != 0)
      validPose = aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, charucoboard,
                                                          camMatrix, distCoeffs, rvec, tvec);

    cv::Mat rot;
    cv::Rodrigues(rvec, rot); 
    Eigen::Matrix<double, 3, 3> eigMat;
    cv::cv2eigen(rot, eigMat);
    Eigen::Quaterniond q(eigMat);
  //if(validPose){
  //  res.x = tvec[0];
  //  res.y = tvec[1];
  //  res.z = tvec[2];
  //  res.q0 = q.w();
  //  res.qx = q.x();
  //  res.qy = q.y();
  //  res.qz = q.z();
  //  ROS_INFO("sending back response:\n x = %f \n y = %f \n z = %f \n q0 = %f \n qx = %f \n qy = %f \n qz = %f \n", 
  //          (float)res.x, (float)res.y, (float)res.z, 
  //          (float)res.q0, (float)res.qx, (float)res.qy, (float)res.qz);
  //}
  //else{
  //  ROS_INFO("sending back response: charuco not found.");
  //} 

    // draw results
    image.copyTo(imageCopy);
    if(markerIds.size() > 0) {
      aruco::drawDetectedMarkers(imageCopy, markerCorners);
    }

    if(showRejected && rejectedMarkers.size() > 0)
      aruco::drawDetectedMarkers(imageCopy, rejectedMarkers, noArray(), Scalar(100, 0, 255));

    if(interpolatedCorners > 0) {
      Scalar color;
      color = Scalar(255, 0, 0);
      aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds, color);
    }

    if(validPose)
      aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvec, tvec, axisLength);

    imshow("out", imageCopy);
    //char key = (char)waitKey(3);
    if(waitKey(30) >= 0) break;
  }
  return true;
}

int main(int argc, char **argv)
{
  if(!cap.isOpened())  // check if we succeeded
    return -1;

  ros::init(argc, argv, "getCharucoPose_server");
  ros::NodeHandle n;
  //ros::ServiceServer service = n.advertiseService("getCharucoPose", getCharucoPose);
  getCharucoPose();
  ROS_INFO("Ready to get charuco pose.");
  ros::spin();

  return 0;
}