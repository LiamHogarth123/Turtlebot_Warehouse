#include "boundarydetection.h"

BoundaryDetection::BoundaryDetection(ros::NodeHandle nh)
{
    pubDetection_ = nh_.advertise<std_msgs::Int16>("/boundary/detection",100);
}

BoundaryDetection::BoundaryDetection()
{
}

BoundaryDetection::~BoundaryDetection()
{
}

int BoundaryDetection::detectColour(cv::Mat image)
{
    /** Matrix to store processed image*/
    cv::Mat image_hsv;

    /** Convert image to HSV*/
    cv::cvtColor(image,image_hsv,cv::COLOR_BGR2HSV);

    /** Values to convert image to Hue Saturation Value (HSV) from Blue Green Red (BGR)*/
    unsigned int hlr = 0; // Lower hue value (red)
    unsigned int hur = 15; // Upper hue value (red)
    unsigned int hlb = 110; // Lower hue value (blue)
    unsigned int hub = 130; // Upper hue value (blue)
    unsigned int sl = 80; // Lower saturation value
    unsigned int su = 255; // Upper saturation value
    unsigned int vl = 0; // Lower value
    unsigned int vu = 255; // Upper value

    cv::Mat blueMask;
    cv::Scalar lowerBlue = cv::Scalar(90, 100, 100);
    cv::Scalar upperBlue = cv::Scalar(130, 255, 255);

    /** Using Scalars*/
    cv::Mat mask1, mask2;
    cv::Scalar lowerRed1 = cv::Scalar(0, 100, 100);
    cv::Scalar upperRed1 = cv::Scalar(10, 255, 255);
    cv::Scalar lowerRed2 = cv::Scalar(160, 100, 100);
    cv::Scalar upperRed2 = cv::Scalar(179, 255, 255);
    
    cv::inRange(image_hsv, lowerRed1, upperRed1, mask1);
    cv::inRange(image_hsv, lowerRed2, upperRed2, mask2);

    cv::inRange(image_hsv, lowerBlue, upperBlue, blueMask);
    
    cv::Mat redMask = mask1 | mask2;

    /** Define variables*/
    int boundaryFlag = 0; // Default to no boundary
    
    int bluePixelCount = cv::countNonZero(blueMask);
    
    if (bluePixelCount >= 1000)
    {
        return boundaryFlag = 1;
    }
    
    int redPixelCount = cv::countNonZero(redMask);
    
    if (redPixelCount >= 1000)
    {
        return boundaryFlag = 2;
    }

    // /** Check for blue*/
    // cv::Mat output_blue;
    // cv::inRange(image_hsv, cv::Scalar(hlb,sl,vl), cv::Scalar(hub,su,vu),output_blue);

    // /** Check for red*/
    // cv::Mat output_red;
    // cv::inRange(image_hsv, cv::Scalar(hlr,sl,vl), cv::Scalar(hur,su,vu),output_red);
    
    // /** Define variables*/
    // // int boundaryFlag = 0; // Default to no boundary
    // cv::Size imageSize = image_hsv.size();
    // double y = imageSize.height; // Image 'y' coordinate
    // double x = imageSize.width; // Image 'x' coordinate

    // /** Define vectors for threshold exceedance*/
    // std::vector<unsigned int> bluePix = {};
    // std::vector<unsigned int> redPix = {};

    // Return max values
    // double minVal; 
    // double maxVal; 
    // cv::Point minLoc; 
    // cv::Point maxLoc;

    // cv::minMaxLoc(output_blue, &minVal, &maxVal, &minLoc, &maxLoc );

    // std::cout << "blue min val: " << minVal << std::endl;
    // std::cout << "blue max val: " << maxVal << std::endl;
    // std::cout << "blue output size = " << output_blue.size() << std::endl;

    // cv::minMaxLoc(output_red, &minVal, &maxVal, &minLoc, &maxLoc );

    // std::cout << "red min val: " << minVal << std::endl;
    // std::cout << "red max val: " << maxVal << std::endl;
    // std::cout << "red output size = " << output_red.size() << std::endl;

    /** Iterate through each pixel, top left to bottom right*/
    // for (int i = 1; i < y; i++)
    // {
    //     for (int j = 1; j < x; j++)
    //     {
    //         // /** Check if blue intensity is above threshold*/
    //         // cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);
    //         // cv::imshow("Image",output_blue);
    //         // cv::waitKey(0);
    //         // if (output_blue.at<int>(i,j) > -1 - 1e-3 && output_blue.at<int>(i,j) < -1 + 1e-3)
    //         // {
    //         //     bluePix.push_back(1);
    //         //     if (bluePix.size() >= blue_id_threshold_)
    //         //     {
    //         //         std::cout << "blue over threshold at y = " << i << "x = " << j << std::endl; // DEBUG ONLY
    //         //         return boundaryFlag = 1;
    //         //     }
    //         // }
    //         /** Check if blue intensity is above threshold*/
    //         // if (output_blue.at<int>(i,j) > 1)
    //         // {
    //         //     std::cout << "condition 1" << std::endl;
    //         // }

    //         // if (output_blue.at<int>(i,j) < -1 + 100e-3)
    //         // {
    //         //     std::cout << "condition 2" << std::endl;
    //         // }

    //         // if (output_blue.at<double>(y/2,x/2) != 0 && !isnan(output_blue.at<double>(y/2,x/2)))
    //         // {
    //         //     std::cout << output_blue.at<double>(y/2,x/2) << std::endl;
    //         // }
            

    //         // if (output_blue.at<int>(i,j) > -1 - 1e-3 && output_blue.at<int>(i,j) < -1 + 1e-3)
    //         if (output_blue.at<int>(i,j) > 255 - 1e-3 && output_blue.at<int>(i,j) < 255 + 1e-3)
    //         // std::cout << output_blue << std::endl;
    //         // cv::waitKey(0);
    //         // if (!isnan(output_blue.at<int>(i,j)))
    //         {
    //             bluePix.push_back(1);
    //             if (bluePix.size() >= blue_id_threshold_)
    //             {
    //                 std::cout << "blue over threshold at y = " << i << "x = " << j << std::endl; // DEBUG ONLY
    //                 return boundaryFlag = 1;
    //             }
    //         }

    //         else{
    //             boundaryFlag = 0;
    //             break;
    //         }

    //         /** Check if red intensity is above threshold*/
    //         // if (output_red.at<int>(i,j) > -1 - 1e-3 && output_red.at<int>(i,j) < -1 + 1e-3)
    //         if (output_red.at<int>(i,j) > 255 - 1e-3 && output_red.at<int>(i,j) < 255 + 1e-3)
    //         {
    //             redPix.push_back(1);
    //             if (redPix.size() >= red_id_threshold_)
    //             {
    //                 std::cout << "red over threshold at y = " << i << "x = " << j << std::endl; // DEBUG ONLY
    //                 return boundaryFlag = 2;
    //             }
    //         }

    //         else{
    //             boundaryFlag = 0;
    //             break;
    //         }
    //     }
    // }
    
    return boundaryFlag;
}

void BoundaryDetection::runBoundaryDetection(cv::Mat input)
{
    unsigned int counter = 0;
    counter++;
    if (counter == 1)
    {
        int flag = BoundaryDetection::detectColour(input);
        std_msgs::Int16 msg;
        msg.data = flag;
        pubDetection_.publish(msg);
        counter = 0;
    }
}