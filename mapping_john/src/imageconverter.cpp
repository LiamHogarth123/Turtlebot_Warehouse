#include "imageconverter.h"

ImageConverter::ImageConverter(ros::NodeHandle nh)
    : nh_(nh)
{
    subCam_ = nh_.subscribe("/usb_cam/image_raw",1,&ImageConverter::camCallback,this);
    subRGBD_ = nh_.subscribe("/camera/color/image_raw",1,&ImageConverter::rgbdCallback,this);
}

ImageConverter::ImageConverter()
{
}

ImageConverter::~ImageConverter()
{
}

void ImageConverter::camCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if (cv_ptr->image.empty()) {
        ROS_WARN("Received empty image");
        return;
    }

    cam_image_ = cv_ptr->image;
}

void ImageConverter::rgbdCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if (cv_ptr->image.empty()) {
        ROS_WARN("Received empty image");
        return;
    }

    rgbd_image_ = cv_ptr->image;
}


cv::Mat ImageConverter::getCam()
{
    return cam_image_;
}

cv::Mat ImageConverter::getRGBD()
{
    return rgbd_image_;
}