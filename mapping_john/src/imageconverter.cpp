#include "imageconverter.h"

ImageConverter::ImageConverter(ros::NodeHandle nh)
    : nh_(nh)
{
    image_sub_ = nh_.subscribe("/usb_cam/image_raw", 1, &ImageConverter::imageCallback, this);
}

ImageConverter::ImageConverter() {}

ImageConverter::~ImageConverter() {}

void ImageConverter::imageCallback(const sensor_msgs::ImageConstPtr& msg)
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

    current_image_ = cv_ptr->image;
}

cv::Mat ImageConverter::getImage()
{
    return current_image_;
}


// ImageConverter::ImageConverter(ros::NodeHandle nh) : it_(nh_)
// {
//     subCam_ = it_.subscribe("/usb_cam/image_raw",1000,&ImageConverter::webcamConvert,this);
//     subRGBD_ = it_.subscribe("/camera/color/image_raw",1000,&ImageConverter::rgbdConvert,this);
// }

// ImageConverter::ImageConverter() : it_(nh_)
// {
// }

// ImageConverter::~ImageConverter()
// {
// }

// void ImageConverter::webcamConvert(const sensor_msgs::ImageConstPtr &msg)
// {
//     std::cout << "1" << std::endl;
//     cv_bridge::CvImagePtr cam_ptr(new cv_bridge::CvImage);
//     try
//     {
//         cam_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
//         std::cout << "3" << std::endl;
//     }
//     catch(cv_bridge::Exception& e)
//     {
//         std::cout << "4" << std::endl;
//         return;
//     }

//     std::cout << "1" << std::endl;
//     cam_ptr_ = cam_ptr;

//     std::cout << "2" << std::endl;

//     cv::imshow("Window", cam_ptr_->image);
//     cv::waitKey(0);
// }

// void ImageConverter::rgbdConvert(const sensor_msgs::Image::ConstPtr &msg)
// {
//     cv_bridge::CvImagePtr rgbd_ptr(new cv_bridge::CvImage);
//     // cv_bridge::CvImagePtr rgbd_ptr;
//     try
//     {
//         rgbd_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
//     }
//     catch(cv_bridge::Exception& e)
//     {
//         return;
//     }

//     rgbd_ptr_ = rgbd_ptr;
// }