#ifndef BOUNDARYDETECTION_H
#define BOUNDARYDETECTION_H
// #include "ros/ros.h"
// #include "opencv2/opencv.hpp"
// #include "opencv2/core/core.hpp"
// #include <opencv2/highgui.hpp>
// #include "opencv2/highgui.hpp"
// #include <opencv2/highgui.hpp>
// #include <../highgui.hpp>
// #include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

class BoundaryDetection
{
public:
    /**
     * @brief Constructor
     */
    BoundaryDetection();

    /**
     * @brief Destructor
     */
    ~BoundaryDetection();

    /**
     * @brief Detect boundary colour
     * @param[in] image Image from webcam
     * @return Double boundary detected and colour
     */
    double detectColour(cv::Mat image);

    bool openCVtest();

private:
    double red_threshold_ = 205;
    double blue_threshold_ = 210;
};

#endif