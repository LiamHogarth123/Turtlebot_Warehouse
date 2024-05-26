#include "imageconverter.h"

ImageConverter::ImageConverter(ros::NodeHandle nh, const std::string tb)
    : nh_(nh)
{
    /** Identify topic names*/
    std::string camTopic;
    std::string rgbdTopic;

    /** If a name is provided*/
    if (!tb.empty())
    {
        camTopic = "/" + tb + "/usb_cam/image_raw";
        rgbdTopic = "/" + tb + "/camera/color/image_raw";
    }
    /** Proceed with no name if none is provided*/
    else
    {
        camTopic = "/usb_cam/image_raw";
        rgbdTopic = "/camera/color/image_raw";
    }

    /** Initialise webcam subscriber*/
    subCam_ = nh_.subscribe(camTopic, 1, &ImageConverter::camCallback, this);

    /** Initialise RGB-D subscriber*/
    subRGBD_ = nh_.subscribe(rgbdTopic, 1, &ImageConverter::rgbdCallback, this);
}

ImageConverter::ImageConverter()
{
}

ImageConverter::~ImageConverter()
{
}

void ImageConverter::camCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if (cv_ptr->image.empty())
    {
        ROS_WARN("Received empty image");
        return;
    }

    cam_image_ = cv_ptr->image;
}

void ImageConverter::rgbdCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if (cv_ptr->image.empty())
    {
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