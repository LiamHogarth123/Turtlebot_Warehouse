#include "boundarydetection.h"
#include <math.h>
#include <vector>
#include <iostream>

// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/highgui.hpp>

BoundaryDetection::BoundaryDetection()
{

}

BoundaryDetection::~BoundaryDetection()
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
    double y; // Image 'y' coordinate
    double x; // Image 'x' coordinate
    cv::Size imageSize = image.size();
    cv::Point3_<uchar>* pixelRGB = image.ptr<cv::Point3_<uchar>>(y,x);

    /* Where
    pixelRGB.x = blue
    pixelRGB.y = green
    pixelRGB.z = red
    */

    // Iterate through each pixel, top left to bottom right
    for (int i = 0; i < imageSize.height; i++)
    {
        for (int j = 0; i < imageSize.width; i++)
        {
            // Check if blue intensity is above threshold
            if (pixelRGB->x > blue_threshold_)
            {
                return boundaryFlag = 1;
            }

            // Check if red intensity is above threshold
            if (pixelRGB->z > red_threshold_)
            {
                return boundaryFlag = 2;
            }
        }
    }
    
    return boundaryFlag;
}