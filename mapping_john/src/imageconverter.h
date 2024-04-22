#ifndef IMAGECONVERTER_H
#define IMAGECONVERTER_H
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <iostream>
#include <math.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

class ImageConverter
{
public:
    /**
     * @brief Constructor
     */
    ImageConverter(ros::NodeHandle nh);

    /**
     * @brief Constructor
     */
    ImageConverter();

    /**
     * @brief Destructor
     */
    ~ImageConverter();

    /**
     * @brief Function to return the webcam image as a cv::Mat
     * @return Latest image, stored in cam_image_
    */
    cv::Mat getCam();

    /**
     * @brief Function to return the RGB-D image as a cv::Mat
     * @return Latest image, stored in rgbd_image_
    */
    cv::Mat getRGBD();

    /**
     * @brief webcam conversion callback
     * @param sensor_msgs::Image::ConstPtr - The scan message
     * @note This function and the declaration are ROS specific
     * @return void
     */
    void camCallback(const sensor_msgs::ImageConstPtr& msg);

    /**
     * @brief RGBD conversion callback
     * @param sensor_msgs::Image::ConstPtr - The scan message
     * @note This function and the declaration are ROS specific
     * @return void
     */
    void rgbdCallback(const sensor_msgs::ImageConstPtr& msg);

// protected:
    /** Nodehandle for this node. Note, only 1 nodehandle is required (there is only 1 node).*/
    ros::NodeHandle nh_;

    /** USB camera (webcam) subscriber*/
    ros::Subscriber subCam_;

    /** Store webcam image*/
    cv::Mat cam_image_;

    /** RGB-D camera subscriber*/
    ros::Subscriber subRGBD_;

    /** Store RGB-D image*/
    cv::Mat rgbd_image_;
};

#endif