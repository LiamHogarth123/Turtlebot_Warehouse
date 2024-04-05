#ifndef MARKERS_H
#define MARKERS_H

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco/dictionary.hpp>
// #include <opencv2/objdetect/
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>

// USEFUL LINK: https://learnopencv.com/augmented-reality-using-aruco-markers-in-opencv-c-python/

class Markers
{
public:
    /**
     * @brief Constructor
     * @param[in] nh nodehandle
     */
    // Markers(ros::NodeHandle nh);

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
     * @param[in] value Value of marker to be assessed
     * @param[in] markerCorners Corners of marker coordinates
     * @param[in] markerLength Size of marker in pixels
     * @param[in] cameraMatrix Camera calibration matrix
     * @param[in] distCoeffs Distortion coefficients based on calibration
     * @param[out] rvecs Rotation vectors
     * @param[out] tvecs Translation vectors
     * @param[out] objPoints
     * @return Pose of marker
     */
    void markerPose(double value, std::vector<std::vector<cv::Point2f>> markerCorners);

    /** Calibration outputs*/
    std::vector<cv::Mat> rvecs_; // Rotation vectors
    std::vector<cv::Mat> tvecs_; // Translation vectors
    cv::Mat cameraMatrix_; // Calibration matrix
    cv::Mat distCoeffs_; // Distortion coefficients

    /** Define the aruco dictionary to be used*/
    cv::Ptr<cv::aruco::Dictionary> dictionary_;

    /** Define the parameters to be used*/
    cv::Ptr<cv::aruco::DetectorParameters> parameters_;

private:
    /**
     * @brief RGBD Callback
     * @param sensor_msgs::Image::ConstPtr - The scan message
     * @note This function and the declaration are ROS specific
     * @return void
     */
    void RGBDCallback(const sensor_msgs::Image::ConstPtr &msg);

protected:
    /** Calibration input parameters*/
    int markersX_ = 5; // Number of markers on x-axis
    int markersY_ = 7; // Number of markers on y-axis
    float markerLength_ = 0.011; // Length on board in metres
    float markerSeparation_ = 0.01; // Distance between markers on board in metres
    float squareLength_ = 0.015; // Length of sides of square
    float aspectRatio_ = 1; // Aspect ratio between fx and fy

    /** Nodehandle for this node. Note, only 1 nodehandle is required (there is only 1 node).*/
    // ros::NodeHandle nh_;

    /**
     * Subscriber to image topic to get image from RGB-D sensor
     * @typedef sensor_msgs/Image
     * @topic /camera/color/image_raw
     */
    // ros::Subscriber subRGBD_;
};

#endif