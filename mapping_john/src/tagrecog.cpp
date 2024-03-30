#include "tagrecog.h"
#include <math.h>
#include <vector>
#include <iostream>

TagRecog::TagRecog()
{

}

TagRecog::~TagRecog()
{

}

void TagRecog::drawMarker(double value)
{
    cv::Mat marker;

    const cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    cv::aruco::drawMarker(dictionary, value, 200, marker, 1);

    cv::imshow("Window",marker);
    cv::waitKey(0);
}

std::vector<int> TagRecog::detectMarker(cv::Mat image)
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
    cv::aruco::detectMarkers(image,dictionary,markerCorners,markerIds,parameters,rejectedCandidates);

    for (unsigned int i = 0; i < markerIds.size(); i++)
    {
        std::cout << "ID = " << markerIds.at(i) << "\n";
    }

    return markerIds;
}