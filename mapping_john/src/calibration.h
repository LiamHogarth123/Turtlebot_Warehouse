#ifndef CALIBRATION_H
#define CALIBRATION_H
/** Include other relevant headers*/
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/UInt16MultiArray.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>

class Calibration
{
public:
    void calculateCameraParameters(const std::vector<cv::Mat> &calibrationImages, const cv::Size &boardSize, float squareLength, float markerLength, cv::Mat &cameraMatrix, cv::Mat &distCoeffs);
};

#endif