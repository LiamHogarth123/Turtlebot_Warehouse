#include "markers.h"
#include <math.h>
#include <vector>
#include <iostream>

Markers::Markers(ros::NodeHandle nh)
{
    subRGBD_ = nh_.subscribe("/camera/color/image_raw", 1000, &Markers::RGBDCallback, this);
}

Markers::Markers()
{
}

Markers::~Markers()
{
}

void Markers::RGBDCallback(const sensor_msgs::Image::ConstPtr &msg)
{
}

double Markers::calibrate()
{
    // Code
    double placeholder = 0.0;
    return placeholder;
}

void Markers::drawMarker(double value)
{
    cv::Mat marker;

    const cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    cv::aruco::drawMarker(dictionary, value, markerLength_, marker, 1);

    cv::imshow("Window", marker);
    cv::waitKey(0);
}

std::vector<int> Markers::detectMarker(cv::Mat image)
{
    // std::vector<int, std::allocator<int>> markerIds;
    std::vector<int> markerIds;
    // std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;

    // cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

    // Load the relevant dictionary
    // cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    // const cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;

    // cv::aruco::ArucoDetector detector(dictionary,detectorParams);

    // cv::aruco::detectMarkers(image,dictionary,markerCorners,markerIds);
    cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

    if (markerIds.size() >= 1)
    {
        for (unsigned int i = 0; i < markerIds.size(); i++)
        {
            std::cout << "ID = " << markerIds.at(i) << "\n";
        }
    }
    else
    {
        markerIds.push_back(-1);
        std::cout << "ID = " << markerIds.at(0) << "\n";
    }

    return markerIds;
}

void Markers::markerPose(double value, std::vector<std::vector<cv::Point2f>> markerCorners)
{
    // Code
}