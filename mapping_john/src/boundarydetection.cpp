#include "boundarydetection.h"
#include <math.h>
#include <vector>
#include <iostream>

// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/highgui.hpp>

// BoundaryDetection::BoundaryDetection(ros::NodeHandle nh)
// {
//     subCam_ = nh_.subscribe("/usb_cam/image_raw", 1000, &BoundaryDetection::webcamCallback, this);
// }

BoundaryDetection::BoundaryDetection()
{
}

BoundaryDetection::~BoundaryDetection()
{
}

void BoundaryDetection::webcamCallback(const sensor_msgs::Image::ConstPtr &msg)
{
}

bool BoundaryDetection::openCVtest()
{
    cv::Mat input = cv::imread("/home/john/Desktop/image.jpg");
    cv::imshow("Window",input);
    cv::waitKey(0);

    return 0;
}

double BoundaryDetection::detectColour(cv::Mat image)
{
    /** Process image to Hue Saturation Value (HSV)*/
    // unsigned int hl = 0; // Lower hue value (red)
    // unsigned int hu = 15; // Upper hue value (red)
    unsigned int hl = 110; // Lower hue value (blue)
    unsigned int hu = 130; // Upper hue value (blue)
    unsigned int sl = 80; // Lower saturation value
    unsigned int su = 255; // Upper saturation value
    unsigned int vl = 0; // Lower value
    unsigned int vu = 255; // Upper value

    /** Matrix to store processed image*/
    cv::Mat image_hsv;

    /** Convert image to HSV*/
    cv::cvtColor(image,image_hsv,cv::COLOR_BGR2HSV);

    /** Extract hue channel for processing*/
    std::vector<cv::Mat> hsv_channels;
    cv::split(image_hsv,hsv_channels);

    cv::Mat hue = hsv_channels[0];

    double minHue;
    double maxHue;
    cv::Point minLoc;
    cv::Point maxLoc;

    cv::Mat output;
    cv::inRange(image_hsv, cv::Scalar(hl,sl,vl), cv::Scalar(hu,su,vu),output);

    std::cout << output.at<int>(234,477);

    // // cv::minMaxLoc(hsv_channels[0],&minHue,&maxHue,&minLoc,&maxLoc);
    cv::minMaxLoc(output,&minHue,&maxHue,&minLoc,&maxLoc);

    std::cout << "min hue = " << minHue << std::endl;
    std::cout << "max hue = " << maxHue << std::endl;

    std::cout << "min hue loc = " << minLoc << std::endl;
    std::cout << "max hue loc = " << maxLoc << std::endl;

    cv::imshow("Window",output);
    cv::waitKey(0);

    


    // std::cout << "hue at middle = " << hue.at<float>(100,300);
    
    /** Define variables*/
    double boundaryFlag = 0; // Default to no boundary
    cv::Size imageSize = image.size();
    double y = imageSize.height; // Image 'y' coordinate
    double x = imageSize.width; // Image 'x' coordinate

    /** Define vectors for threshold exceedance*/
    std::vector<unsigned int> bluePix = {};
    std::vector<unsigned int> redPix = {};

    /* Where
    pixelRGB.x = blue
    pixelRGB.y = green
    pixelRGB.z = red
    */

    // std::cout << "image height = " << imageSize.height << "\n"; // DEBUG ONLY
    // std::cout << "image width = " << imageSize.width << "\n"; // DEBUG ONLY

    // std::cout << "y = " << y << "\n"; // DEBUG ONLY
    // std::cout << "x = " << x << "\n"; // DEBUG ONLY

    /** Iterate through each pixel, top left to bottom right*/
    for (int i = 1; i < y; i++)
    {
        for (int j = 1; j < x; j++)
        {
            // int hue_out = hsv_channels[0].at<int>(i,j);
            // int hue_out = hue.at<int>(i,j);
            // int hue_out = hue.data.at(i);
            // std::cout << "hue = " << hue_out << std::endl;

            // if (hue_out > 120 - colour_tolerance_ && hue_out < 120 + colour_tolerance_)
            // {
            //     std::cout << "blue!" << hue_out << i << j <<std::endl;
            // }

            // std::cout << "haw\n";
            // cv::Point3_<uchar>* pixelRGB = image.ptr<cv::Point3_<uchar>>(i,j);
            cv::Vec3b pixelRGB = image.at<cv::Vec3b>(i,j);
            /** Check if blue intensity is above threshold*/
            if (pixelRGB[0] > blue_threshold_)
            {
                bluePix.push_back(1);
                // std::cout << "bluePix size = " << bluePix.size() << "\n";
                if (bluePix.size() >= colour_id_threshold_)
                {
                    // std::cout << "blue over threshold at y = " << i << "x = " << j << "\n";
                    return boundaryFlag = 1;
                }
            }

            /** Check if red intensity is above threshold*/
            if (pixelRGB[2] > red_threshold_)
            {
                redPix.push_back(1);
                // std::cout << "redPix size = " << redPix.size() << "\n";
                if (redPix.size() >= colour_id_threshold_)
                {
                    // std::cout << "red over threshold at y = " << i << "x = " << j << "\n";
                    return boundaryFlag = 2;
                }
            }
        }
    }
    
    return boundaryFlag;
}