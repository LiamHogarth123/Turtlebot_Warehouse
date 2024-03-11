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

bool BoundaryDetection::detectColour(long image)
{
    cv::Mat input = cv::imread("image.jpg",1);
    // cv::namedWindow("image",cv::setWindowProperty::WND_PROP_AUTOSIZE);
    cv::imshow("image",input);
}