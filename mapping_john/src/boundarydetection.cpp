#include "boundarydetection.h"
#include <math.h>
#include <vector>
#include <iostream>

// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/highgui.hpp>

// BoundaryDetection::BoundaryDetection(ros::NodeHandle nh)
// {
//     subCam_ = nh_.subscribe("/usb_cam/image_raw", 1000, &BoundaryDetection::webcamCallback, this);
// }

BoundaryDetection::BoundaryDetection()
{
}

BoundaryDetection::~BoundaryDetection()
{
}

void BoundaryDetection::webcamCallback(const sensor_msgs::Image::ConstPtr &msg)
{
}

double BoundaryDetection::detectColour(cv::Mat image)
{
    /** Values to convert image to Hue Saturation Value (HSV) from Blue Green Red (BGR)*/
    unsigned int hlr = 0; // Lower hue value (red)
    unsigned int hur = 15; // Upper hue value (red)
    unsigned int hlb = 110; // Lower hue value (blue)
    unsigned int hub = 130; // Upper hue value (blue)
    unsigned int sl = 80; // Lower saturation value
    unsigned int su = 255; // Upper saturation value
    unsigned int vl = 0; // Lower value
    unsigned int vu = 255; // Upper value

    /** Matrix to store processed image*/
    cv::Mat image_hsv;

    /** Convert image to HSV*/
    cv::cvtColor(image,image_hsv,cv::COLOR_BGR2HSV);

    /** Check for blue*/
    cv::Mat output_blue;
    cv::inRange(image_hsv, cv::Scalar(hlb,sl,vl), cv::Scalar(hub,su,vu),output_blue);

    /** Check for red*/
    cv::Mat output_red;
    cv::inRange(image_hsv, cv::Scalar(hlr,sl,vl), cv::Scalar(hur,su,vu),output_red);
    
    /** Define variables*/
    double boundaryFlag = 0; // Default to no boundary
    cv::Size imageSize = image_hsv.size();
    double y = imageSize.height; // Image 'y' coordinate
    double x = imageSize.width; // Image 'x' coordinate

    /** Define vectors for threshold exceedance*/
    std::vector<unsigned int> bluePix = {};
    std::vector<unsigned int> redPix = {};

    /** Iterate through each pixel, top left to bottom right*/
    for (int i = 1; i < y; i++)
    {
        for (int j = 1; j < x; j++)
        {
            /** Check if blue intensity is above threshold*/
            if (output_blue.at<int>(i,j) > -1 - 1e-3 && output_blue.at<int>(i,j) < -1 + 1e-3)
            {
                bluePix.push_back(1);
                if (bluePix.size() >= colour_id_threshold_)
                {
                    std::cout << "blue over threshold at y = " << i << "x = " << j << std::endl; // DEBUG ONLY
                    return boundaryFlag = 1;
                }
            }

            /** Check if red intensity is above threshold*/
            if (output_red.at<int>(i,j) > -1 - 1e-3 && output_red.at<int>(i,j) < -1 + 1e-3)
            {
                redPix.push_back(1);
                if (redPix.size() >= colour_id_threshold_)
                {
                    std::cout << "red over threshold at y = " << i << "x = " << j << std::endl; // DEBUG ONLY
                    return boundaryFlag = 2;
                }
            }
        }
    }
    
    return boundaryFlag;
}

double BoundaryDetection::runBoundaryDetection(bool running)
{
    double boundaryFlag = 0;
    // while (running == true)
    // {
    //     double flag = BoundaryDetection::detectColour()
    // }
    return boundaryFlag;
}