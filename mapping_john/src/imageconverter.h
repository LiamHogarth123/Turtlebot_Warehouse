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

class CvImage
{
public:
std_msgs::Header header;
std::string encoding;
cv::Mat image;
};

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

// private:
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
    void rgbdConvert(const sensor_msgs::Image::ConstPtr &msg);

// protected:
    /** Nodehandle for this node. Note, only 1 nodehandle is required (there is only 1 node).*/
    ros::NodeHandle nh_;

    /** Image transport initialiser*/
    image_transport::ImageTransport it_;

    /** USB camera subscriber and output*/
    image_transport::Subscriber subCam_;
    typedef boost::shared_ptr<CvImage> CvImagePtr;
    typedef boost::shared_ptr<CvImage const> CvImageConstPtr;
    cv_bridge::CvImagePtr cam_ptr_;

    /** RGB-D camera subscriber and output*/
    image_transport::Subscriber subRGBD_;
    cv_bridge::CvImagePtr rgbd_ptr_;
};

#endif