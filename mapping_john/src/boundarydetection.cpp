#include "boundarydetection.h"

BoundaryDetection::BoundaryDetection(ros::NodeHandle nh)
{
    pubDetection_ = nh_.advertise<std_msgs::Int16>("/boundary/detection",100);
}

BoundaryDetection::BoundaryDetection()
{
}

BoundaryDetection::~BoundaryDetection()
{
}

int BoundaryDetection::detectColour(cv::Mat image)
{
    /** Matrix to store processed image*/
    cv::Mat image_hsv;

    /** Convert image to HSV*/
    cv::cvtColor(image,image_hsv,cv::COLOR_BGR2HSV);

    /** Masks for inRange*/
    cv::Mat blueMask;
    cv::Mat redMask1, redMask2;

    /** Ranges for hue, saturation and value*/
    cv::Scalar lowerBlue = cv::Scalar(90, 100, 100);
    cv::Scalar upperBlue = cv::Scalar(130, 255, 255);
    cv::Scalar lowerRed1 = cv::Scalar(0, 100, 100);
    cv::Scalar upperRed1 = cv::Scalar(10, 255, 255);
    cv::Scalar lowerRed2 = cv::Scalar(160, 100, 100);
    cv::Scalar upperRed2 = cv::Scalar(179, 255, 255);
    
    /** Evaluate image within nominated ranges*/
    /** Red*/
    cv::inRange(image_hsv, lowerRed1, upperRed1, redMask1);
    cv::inRange(image_hsv, lowerRed2, upperRed2, redMask2);
    /** Blue*/
    cv::inRange(image_hsv, lowerBlue, upperBlue, blueMask);
    
    /** Select red mask*/
    cv::Mat redMask = redMask1 | redMask2;

    /** Define variables*/
    int boundaryFlag = 0; // Default to no boundary
    /** Count number of coloured pixels (identified by non-zero values)*/
    int bluePixelCount = cv::countNonZero(blueMask); // Count of blue pixels
    int redPixelCount = cv::countNonZero(redMask); // Count of red pixels
    
    /** Query blue boundary*/
    if (bluePixelCount >= colour_threshold_)
    {
        return boundaryFlag = 1;
    }
    
    /** Query red boundary*/
    if (redPixelCount >= colour_threshold_)
    {
        return boundaryFlag = 2;
    }

    /** Return the flag*/
    return boundaryFlag;
}

void BoundaryDetection::runBoundaryDetection(cv::Mat input)
{
    unsigned int counter = 0;
    counter++;
    if (counter == 1)
    {
        int flag = BoundaryDetection::detectColour(input);
        std_msgs::Int16 msg;
        msg.data = flag;
        pubDetection_.publish(msg);
        counter = 0;
    }
}