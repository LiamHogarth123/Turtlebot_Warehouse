#include "imageconverter.h"

ImageConverter::ImageConverter(ros::NodeHandle nh) : it_(nh_)
{
    subCam_ = it_.subscribe("/usb_cam/image_raw",1000,&ImageConverter::webcamConvert,this);
    pubCam_ = it_.advertise("/cam/output",1000);

    subRGBD_ = it_.subscribe("/camera/color/image_raw",1000,&ImageConverter::rgbdConvert,this);
    pubRGBD_ = it_.advertise("/rgbd/output",1000);
}

ImageConverter::~ImageConverter()
{
}

void ImageConverter::webcamConvert(const sensor_msgs::Image::ConstPtr &msg)
{
    cv_bridge::CvImagePtr cam_ptr;
    pubCam_.publish(cam_ptr->toImageMsg());
}

void ImageConverter::rgbdConvert(const sensor_msgs::Image::ConstPtr &msg)
{
    cv_bridge::CvImagePtr rgbd_ptr;
    pubRGBD_.publish(rgbd_ptr->toImageMsg());
}