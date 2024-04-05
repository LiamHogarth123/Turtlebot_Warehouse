#include "imageconverter.h"

ImageConverter::ImageConverter(ros::NodeHandle nh) : it_(nh_)
{
    subCam_ = it_.subscribe("/usb_cam/image_raw",1000,&ImageConverter::webcamConvert,this);
    pubCam_ = it_.advertise("/cam/output",1000);
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
    
}
