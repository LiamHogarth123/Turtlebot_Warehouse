#ifndef BOUNDARYDETECTION_H
#define BOUNDARYDETECTION_H

/** Include 'ImageConverter' for webcam stream*/
#include <imageconverter.h>

/** Include other relevant headers*/
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <iostream>
#include <math.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

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

    /**
     * @brief Run boundary detection
     * @param[in] running Boolean to run
     * @return If boundary is detected
     */
    double runBoundaryDetection(bool running, cv::Mat input);

private:
    double colour_id_threshold_ = 3000; /** Number of -1 pixels to identify a colour*/
};

#endif