#ifndef BOUNDARYDETECTION_H
#define BOUNDARYDETECTION_H

/** Include 'ImageConverter' for webcam stream*/
#include <imageconverter.h>

/** Include other relevant headers*/
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Int16.h"
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
     * @param[in] nh ROS NodeHandle
     * @param[in] tb TurtleBot name
     */
    BoundaryDetection(ros::NodeHandle nh, const std::string tb);

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
    int detectColour(cv::Mat image);

    /**
     * @brief Run boundary detection
     * @param[in] input Image for processing
     * @return If boundary is detected
     */
    void runBoundaryDetection(cv::Mat input);

    /** Nodehandle for this node. Note, only 1 nodehandle is required (there is only 1 node).*/
    ros::NodeHandle nh_;

    /**
     * Publisher of boundary detection
     * @typedef std_msgs/Int16
     * @topic /boundary/detection
    */ 
    ros::Publisher pubDetection_;

private:
    double colour_threshold_ = 3000; /** Number of non-zero pixels to identify a colour*/
};

#endif