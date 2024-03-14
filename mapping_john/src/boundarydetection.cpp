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
    cv::Mat input = cv::imread("/home/john/Desktop/image.jpg");
    cv::imshow("Window",input);
    return 0;
}