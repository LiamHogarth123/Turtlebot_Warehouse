#ifndef MARKERS_H
#define MARKERS_H

/** Include 'ImageConverter' for webcam stream*/
#include <imageconverter.h>

/** Include other relevant headers*/
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/PoseArray.h"
#include "std_msgs/UInt16MultiArray.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include "marker_msgs/marker.h"

class Markers
{
public:
    /**
     * @brief Constructor
     * @param[in] nh nodehandle
     */
    Markers(ros::NodeHandle nh);

    /**
     * @brief Constructor
     */
    Markers();

    /**
     * @brief Destructor
     */
    ~Markers();

    /**
     * @brief markers thread.
     *
     *  The main processing thread that will run continously and utilise the data
     *  When data needs to be combined then running a thread seperate to callback will guarantee data is processed
     */
    void markersThread();

    /**
     * @brief Calibrate AR tag detector
     * @param[in] @todo
     * @return Calibration parameters (global)
     */
    double calibrate(cv::Mat calibImage);

    /**
     * @brief Draw AR tag marker
     * @param[in] value Value of marker to be generated
     * @return void
     */
    void drawMarker(double value);

    /**
     * @brief Detect AR tag marker
     * @param[in] image Image from webcam
     * @return Number of tag observed OR return -1 if no tag | Corners of tag OR return -1 if no tag
     */
    // std::pair<std::vector<int>,std::vector<std::vector<cv::Point2f>>> detectMarker(cv::Mat image);
    std::vector<int> detectMarker(cv::Mat image);

    /**
     * @brief Detect pose of AR tag marker
     * @param[in] publish Boolean if publishing info to ROS topics is required
     * @param[in] markerCorners_ Coordinates of marker corners
     * @param[in] markerLength_ Size of marker in metres (m)
     * @param[in] cameraMatrix_ Camera calibration matrix
     * @param[in] distCoeffs_ Distortion coefficients based on calibration
     * @param[out] rvecs_ Rotation vectors
     * @param[out] tvecs_ Translation vectors
     * @param[out] objPoints
     * @return Pose of marker
     */
    void markerPose(bool publish);

    /** Calibration information*/
    cv::Mat cameraMatrix_; // Calibration matrix
    cv::Mat distCoeffs_; // Distortion coefficients

    /** Detected marker information*/
    std::vector<int> markerIds_; // IDs of detected markers
    std::vector<std::vector<cv::Point2f>> markerCorners_; // Coordinates of corners of detected markers
    std::vector<std::vector<cv::Point2f>> rejectedCandidates_; // Rejected potential markers
    
    /** Vectors of each marker detected*/
    std::vector<cv::Mat> rvecs_; // Rotation vectors
    std::vector<cv::Mat> tvecs_; // Translation vectors

    /** Horizontal (x) position errors*/
    std::vector<float> xErrors_;

    /** Define the aruco dictionary to be used*/
    cv::Ptr<cv::aruco::Dictionary> dictionary_;

    /** Define the parameters to be used*/
    cv::Ptr<cv::aruco::DetectorParameters> parameters_;

protected:
    /** Calibration input parameters*/
    int markersX_ = 5; // Number of markers on x-axis
    int markersY_ = 7; // Number of markers on y-axis
    float markerLength_ = 0.025; // Length on board in metres
    float markerSeparation_ = 0.01; // Distance between markers on board in metres
    float squareLength_ = 0.034; // Length of sides of square
    float aspectRatio_ = 1; // Aspect ratio between fx and fy

    /** Nodehandle for this node. Note, only 1 nodehandle is required (there is only 1 node).*/
    ros::NodeHandle nh_;

    /**
     * Publisher of marker IDs only
     * @typedef std_msgs/Int16MultiArray
     * @topic /markers/ids
    */ 
    ros::Publisher pubIds_;

    /**
     * Publisher of marker IDs AND horizontal error (in metres)
     * @typedef marker_msgs/marker
     * @topic /markers/info
    */   
    ros::Publisher pubMarker_;
};

#endif