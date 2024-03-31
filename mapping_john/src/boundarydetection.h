#ifndef BOUNDARYDETECTION_H
#define BOUNDARYDETECTION_H
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
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
    BoundaryDetection(ros::NodeHandle nh);
    
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

    /**
     * @brief webcam Callback
     * @param sensor_msgs::Image::ConstPtr - The scan message
     * @note This function and the declaration are ROS specific
     * @return void
     */
    void webcamCallback(const sensor_msgs::Image::ConstPtr &msg);

protected:
    /** Nodehandle for this node. Note, only 1 nodehandle is required (there is only 1 node).*/
    ros::NodeHandle nh_;

    /**
     * Subscriber to image topic to get image from webcam
     * @typedef sensor_msgs/Image
     * @topic /usb_cam/image_raw
     */
    ros::Subscriber subCam_;
};

#endif