#include "imageconverter.h"

ImageConverter::ImageConverter(ros::NodeHandle nh) : it_(nh_)
{
    subCam_ = it_.subscribe("/usb_cam/image_raw",1000,&ImageConverter::webcamConvert,this);
    subRGBD_ = it_.subscribe("/camera/color/image_raw",1000,&ImageConverter::rgbdConvert,this);
}

ImageConverter::ImageConverter() : it_(nh_)
{
}

ImageConverter::~ImageConverter()
{
}

void ImageConverter::webcamConvert(const sensor_msgs::Image::ConstPtr &msg)
{
    cv_bridge::CvImagePtr cam_ptr;
    try
    {
        cam_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e)
    {
        return;
    }

    cam_ptr_ = cam_ptr; 
}

void ImageConverter::rgbdConvert(const sensor_msgs::Image::ConstPtr &msg)
{
    cv_bridge::CvImagePtr rgbd_ptr;
    try
    {
        rgbd_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e)
    {
        return;
    }

    rgbd_ptr_ = rgbd_ptr;
}