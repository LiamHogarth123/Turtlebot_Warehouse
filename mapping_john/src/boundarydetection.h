#ifndef BOUNDARYDETECTION_H
#define BOUNDARYDETECTION_H
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
    BoundaryDetection(ros::NodeHandle nh);

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
    double runBoundaryDetection(bool running);

private:
    double colour_id_threshold_ = 1000; /** Number of -1 pixels to identify a colour*/

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
     * @topic /cam/output
     */
    ros::Subscriber subCam_;

    /** Image reading*/
    cv::Mat cam_;
};

#endif