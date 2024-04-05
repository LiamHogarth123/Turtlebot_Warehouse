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
     * @brief Destructor
     */
    ~ImageConverter();

private:
    /**
     * @brief webcam conversion callback
     * @param sensor_msgs::Image::ConstPtr - The scan message
     * @note This function and the declaration are ROS specific
     * @return void
     */
    void webcamConvert(const sensor_msgs::Image::ConstPtr &msg);

    /**
     * @brief RGBD conversion callback
     * @param sensor_msgs::Image::ConstPtr - The scan message
     * @note This function and the declaration are ROS specific
     * @return void
     */
    void RGBDConvert(const sensor_msgs::Image::ConstPtr &msg);

protected:
    /** Nodehandle for this node. Note, only 1 nodehandle is required (there is only 1 node).*/
    ros::NodeHandle nh_;

    /** Image transport initialisers*/
    image_transport::ImageTransport it_;
    image_transport::Subscriber subCam_;
    image_transport::Publisher pubCam_;
};

#endif