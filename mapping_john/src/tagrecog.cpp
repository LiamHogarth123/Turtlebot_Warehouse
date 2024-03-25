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

double TagRecog::detectMarker(cv::Mat image)
{
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;

    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();

    // Load the relevant dictionary
    // cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    const cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    // cv::aruco::ArucoDetector detector(dictionary,detectorParams);
    
    cv::aruco::detectMarkers(image,dictionary,markerCorners,markerIds);

    return 0;
}