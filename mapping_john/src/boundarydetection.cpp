#include "boundarydetection.h"
#include <math.h>
#include <vector>
#include <iostream>

// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/highgui.hpp>

BoundaryDetection::BoundaryDetection(ros::NodeHandle nh)
{
    subCam_ = nh_.subscribe("/usb_cam/image_raw", 1000, &BoundaryDetection::webcamCallback, this);
}

BoundaryDetection::BoundaryDetection()
{
}

BoundaryDetection::~BoundaryDetection()
{
}

void BoundaryDetection::webcamCallback(const sensor_msgs::Image::ConstPtr &msg)
{
}

bool BoundaryDetection::openCVtest()
{
    cv::Mat input = cv::imread("/home/john/Desktop/image.jpg");
    cv::imshow("Window",input);
    cv::waitKey(0);

    return 0;
}

double BoundaryDetection::detectColour(cv::Mat image)
{
    // Define variables
    double boundaryFlag = 0; // Default to no boundary
    cv::Size imageSize = image.size();
    double y = imageSize.height; // Image 'y' coordinate
    double x = imageSize.width; // Image 'x' coordinate
    
    // cv::Point3_<uchar>* pixelRGB = image.ptr<cv::Point3_<uchar>>(y,x);

    /* Where
    pixelRGB.x = blue
    pixelRGB.y = green
    pixelRGB.z = red
    */

    std::cout << "image height = " << imageSize.height << "\n";
    std::cout << "image width = " << imageSize.width << "\n";

    std::cout << "y = " << y << "\n";
    std::cout << "x = " << x << "\n";

    // std::cout << "y = " << y << "\n";
    // std::cout << "x = " << x << "\n";

    // Iterate through each pixel, top left to bottom right
    for (int i = 1; i < y; i++)
    {
        // std::cout << "yee\n";
        for (int j = 1; j < x; j++)
        {
            // std::cout << "haw\n";
            // cv::Point3_<uchar>* pixelRGB = image.ptr<cv::Point3_<uchar>>(i,j);
            cv::Vec3b pixelRGB = image.at<cv::Vec3b>(i,j);
            // Check if blue intensity is above threshold
            if (pixelRGB[0] > blue_threshold_)
            {
                std::cout << "blue over threshold at y = " << i << "x = " << j << "\n";
                return boundaryFlag = 1;
            }

            /** NOTE: Currently assumed that red boundary is acceptable*/
            // // Check if red intensity is above threshold
            // if (pixelRGB[2] > red_threshold_)
            // {
            //     std::cout << "red over threshold at y = " << i << "x = " << j << "\n";
            //     return boundaryFlag = 2;
            // }
        }
    }
    
    return boundaryFlag;
}