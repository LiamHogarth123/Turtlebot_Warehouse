#include "boundarydetection.h"
#include <math.h>
#include <vector>
#include <iostream>
#include <chrono>
#include <thread>

// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/highgui.hpp>

BoundaryDetection::BoundaryDetection()
{

}

BoundaryDetection::~BoundaryDetection()
{

}

// bool BoundaryDetection::detectColour(long image)
bool BoundaryDetection::detectColour()
{
    cv::Mat input = cv::imread("/home/john/Desktop/image.jpg");
    cv::imshow("Window",input);
    cv::waitKey(0);

    // std::this_thread::sleep_for(std::chrono::nanoseconds(1000000));

    return 0;
}