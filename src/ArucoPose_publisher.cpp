#include "opencv2/opencv.hpp"
#include <opencv2/aruco.hpp>
#include "ros/ros.h"
#include "shallow_depth_insertion/getArucoPose.h"
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <vector>
#include <iostream>
#include "shallow_depth_insertion/Aruco_array.h"
#include "shallow_depth_insertion/Aruco.h"

using namespace std;
using namespace cv;

VideoCapture cap(1); // open the default camera

namespace {
const char* about = "Basic marker detection";
const char* keys  =
        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16,"
        "DICT_APRILTAG_16h5=17, DICT_APRILTAG_25h9=18, DICT_APRILTAG_36h10=19, DICT_APRILTAG_36h11=20}"
        "{v        |       | Input from video file, if ommited, input comes from camera }"
        "{ci       | 0     | Camera id if input doesnt come from video (-v) }"
        "{c        |       | Camera intrinsic parameters. Needed for camera pose }"
        "{l        | 0.1   | Marker side lenght (in meters). Needed for correct scale in camera pose }"
        "{dp       |       | File of marker detector parameters }"
        "{r        |       | show rejected candidates too }"
        "{refine   |       | Corner refinement: CORNER_REFINE_NONE=0, CORNER_REFINE_SUBPIX=1,"
        "CORNER_REFINE_CONTOUR=2, CORNER_REFINE_APRILTAG=3}";
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

// Read Aruco parameters 
static bool readCharucoParameters(std::string filename, int &squaresX, int &squaresY, float &squareLength, float &markerLength) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["markerLength"] >> markerLength;
    return true;
}

// Service callback function: detect id tag and return tag pose 
bool getArucoPose(ros::Publisher pub)
{
  int squaresX, squaresY;
  float squareLength, markerLength;
  readCharucoParameters("/home/john/catkin_ws/src/shallow_depth_insertion/config/charuco_param.yaml", squaresX, squaresY, squareLength, markerLength);
  
  int dictionaryId = 0;
  bool showRejected = 0; 
  bool estimatePose = 1;

  // Read detector parameters from config/yaml
  cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
  bool readOk = readDetectorParameters("/home/john/catkin_ws/src/shallow_depth_insertion/config/detector_params.yaml", detectorParams);
  if(!readOk) {
    std::cerr << "Invalid detector parameters file" << std::endl;
    return 0;
  }
  
  // Specify aruco tag dictionary id
  cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

  // Read camera parameters from config/yaml
  cv::Mat camMatrix, distCoeffs;
  if(estimatePose) {
    bool readOk = readCameraParameters("/home/john/catkin_ws/src/shallow_depth_insertion/config/camera_param.yaml", camMatrix, distCoeffs);
    if(!readOk) {
      std::cerr << "Invalid camera file" << std::endl;
      return 0;
    }
  }
  
  while(ros::ok())
  {
    // Get latest image from camera 
    cv::Mat image, imageCopy;
    cap >> image; // get a new frame from camera

    vector< int > ids;
    vector< vector< Point2f > > corners, rejected;
    vector< Vec3d > rvecs, tvecs;

    // detect markers and estimate pose
    cv::aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);
    if(estimatePose && ids.size() > 0){
      cv::aruco::estimatePoseSingleMarkers(corners, markerLength, camMatrix, distCoeffs, rvecs, tvecs);
     
      ROS_INFO("Dectected Tags :");
      for (int i=0; i<ids.size(); i++) 
        std::cout << " " << ids[i]; 
      std::cout << "\n"; 
    }

    // Find requested tag if it exists 
    std::vector<int>::iterator it;
    
    // Populate msg array and publish
    shallow_depth_insertion::Aruco_array msg;
    for (int i=0; i<ids.size(); i++){ 
      shallow_depth_insertion::Aruco tag;
      cv::Mat rot;
      cv::Rodrigues(rvecs[i], rot); // Probably need to index rvecs
      Eigen::Matrix<double, 3, 3> eigMat;
      cv::cv2eigen(rot, eigMat);
      Eigen::Quaterniond q(eigMat);
      tag.tag_id = ids[i];
      tag.pose.position.x = tvecs[i][0];
      tag.pose.position.y = tvecs[i][1];
      tag.pose.position.z = tvecs[i][2];
      tag.pose.orientation.x = q.x();
      tag.pose.orientation.y = q.y();
      tag.pose.orientation.z = q.z();
      tag.pose.orientation.w = q.w();
      msg.aruco_array.push_back(tag);
    }
    pub.publish(msg);

    // draw results
    image.copyTo(imageCopy);
    if(ids.size() > 0) {
      cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
      if(estimatePose) {
        for(unsigned int i = 0; i < ids.size(); i++){
          cv::aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i],
                                      markerLength * 0.5f);
        }
      }
    }

    if(showRejected && rejected.size() > 0)
      cv::aruco::drawDetectedMarkers(imageCopy, rejected, cv::noArray(), cv::Scalar(100, 0, 255));
 
    cv::imshow("out", imageCopy);
    if(waitKey(30) >= 0) break;
  }
  return true;
}

int main(int argc, char **argv)
{
  if(!cap.isOpened())  // check if we succeeded
    return -1;
   
  ros::init(argc, argv, "ArucoPose_publisher");
  ros::NodeHandle n;
  ros::Publisher ArucoPose_pub = n.advertise<shallow_depth_insertion::Aruco_array>("ArucoPose_array", 1);
  getArucoPose(ArucoPose_pub);
  ROS_INFO("Aruco_Pose publisher ended.");
  ros::spin();

  return 0;
}
