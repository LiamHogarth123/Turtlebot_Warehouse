#include "markers.h"
#include <math.h>
#include <vector>
#include <iostream>

Markers::Markers(ros::NodeHandle nh)
{
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    parameters_ = cv::aruco::DetectorParameters::create();
    cameraMatrix_ = (cv::Mat_<double>(3,3) << 912.5086,0.0,651.25220,0.0,912.21368,348.58951,0.0,0.0,1.0);
    distCoeffs_ = (cv::Mat_<double>(1,5) << 0.0,0.0,0.0,0.0,0.0);
    pubIds_ = nh_.advertise<std_msgs::Int16MultiArray>("/markers/ids",100);
    pubMarker_ = nh_.advertise<marker_msgs::marker>("/markers/info",100);
}

Markers::Markers()
{
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    parameters_ = cv::aruco::DetectorParameters::create();
}

Markers::~Markers()
{
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
    /** Perform marker detection using OpenCV*/
    cv::aruco::detectMarkers(image, dictionary_, markerCorners_, markerIds_, parameters_, rejectedCandidates_);
    
    /** DEBUG ONLY*/
    // if (markerIds_.size() >= 1)
    // {
    //     for (unsigned int i = 0; i < markerIds_.size(); i++)
    //     {
    //         std::cout << "ID = " << markerIds_.at(i) << std::endl;
    //     }
    // }
    // else
    // {
    //     markerIds_.push_back(-1);
    //     std::cout << "ID = " << markerIds_.at(0) << std::endl;
    // }

    return markerIds_;
}

void Markers::markerPose(bool publish)
{
    cv::Mat objPoints(4, 1, CV_32FC3);
    objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength_/2.f, markerLength_/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength_/2.f, markerLength_/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength_/2.f, -markerLength_/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength_/2.f, -markerLength_/2.f, 0);

    /** Empty the vector to re-populate with new markers*/
    if (!xErrors_.empty())
    {
        xErrors_.clear();
    }

    /** Empty the vector to re-populate with new markers*/
    if (!yaws_.empty())
    {
        yaws_.clear();
    }

    /** Check if there are marker IDs from detected tags*/
    if (!markerIds_.empty())
    {
        /** Allocate size of rotation and translation vectors*/
        cv::Mat emptyMat = (cv::Mat_<double>(1,3) << 0.0,0.0,0.0);
        for (unsigned int i = 0; i < markerIds_.size(); i++)
        {
            if (rvecs_.size() != markerIds_.size())
            {
                rvecs_.push_back(emptyMat);
                tvecs_.push_back(emptyMat);
            }
            cv::solvePnP(objPoints,markerCorners_.at(i),cameraMatrix_,distCoeffs_,rvecs_.at(i),tvecs_.at(i));
            xErrors_.push_back(tvecs_.at(i).at<double>(0));
            yaws_.push_back(rvecs_.at(i).at<double>(2));
        }
    }
    else
    {
        /** If there are no tags detected, return -1*/
        cv::Mat negativeMat = (cv::Mat_<double>(1,3) << -1,-1,-1);
        rvecs_.push_back(negativeMat);
        tvecs_.push_back(negativeMat);
    }

    /** If nominated, publish data to ROS topic*/
    if (publish)
    {
        for (unsigned int i = 0; i < markerIds_.size(); i++)
        {
            /** Populate marker IDs*/
            std_msgs::Int16MultiArray marker_ids;
            marker_ids.layout.dim.clear();
            marker_ids.layout.dim.resize(1);
            marker_ids.layout.dim[0].size = markerIds_.size();
            marker_ids.layout.dim[0].stride = 1;
            marker_ids.layout.dim[0].label = "ids";
            marker_ids.data.clear();
            marker_ids.data.insert(marker_ids.data.end(),markerIds_.begin(),markerIds_.end());

            /** Publish the message*/
            pubIds_.publish(marker_ids);

            /** Populate marker IDs*/
            marker_msgs::marker marker_info;
            marker_info.ids.layout.dim.clear();
            marker_info.ids.layout.dim.resize(1);
            marker_info.ids.layout.dim[0].size = markerIds_.size();
            marker_info.ids.layout.dim[0].stride = 1;
            marker_info.ids.layout.dim[0].label = "ids";
            marker_info.ids.data.clear();
            marker_info.ids.data.insert(marker_info.ids.data.end(),markerIds_.begin(),markerIds_.end());

            /** Populate horizontal error for each marker*/
            marker_info.xErrors.layout.dim.clear();
            marker_info.xErrors.layout.dim.resize(1);
            marker_info.xErrors.layout.dim[0].size = xErrors_.size();
            marker_info.xErrors.layout.dim[0].stride = 1;
            marker_info.xErrors.layout.dim[0].label = "horizontal errors";
            marker_info.xErrors.data = xErrors_;

            /** Populate yaw error for each marker*/
            marker_info.yaws.layout.dim.clear();
            marker_info.yaws.layout.dim.resize(1);
            marker_info.yaws.layout.dim[0].size = xErrors_.size();
            marker_info.yaws.layout.dim[0].stride = 1;
            marker_info.yaws.layout.dim[0].label = "yaws";
            marker_info.yaws.data = yaws_;

            /** Publish the message*/
            pubMarker_.publish(marker_info);
        }
    }
}

double Markers::calibrate(cv::Mat calibImage)
{
    /** Create ArUco grid board*/
    cv::Ptr<cv::aruco::GridBoard> gridBoard = cv::aruco::GridBoard::create(markersX_, markersY_, markerLength_, markerSeparation_, dictionary_);

    /** Create ChArUco grid board*/
    cv::Ptr<cv::aruco::CharucoBoard> charucoBoard = cv::aruco::CharucoBoard::create(markersX_, markersY_, squareLength_, markerLength_, dictionary_);

    /** Initialise ArUco board*/
    cv::Ptr<cv::aruco::Board> arBoard = gridBoard.staticCast<cv::aruco::Board>();

    /** Initialise ChArUco board*/
    cv::Ptr<cv::aruco::Board> chBoard = charucoBoard.staticCast<cv::aruco::Board>();

    /** Populate camera matrix*/
    cameraMatrix_ = cv::Mat::eye(3, 3, CV_64F);
    cameraMatrix_.at<double>(0, 0) = aspectRatio_;

    /** Define marker detection requirements*/
    std::vector<std::vector<std::vector<cv::Point2f>>> allCorners;
    std::vector<std::vector<int>> allIds;
    cv::Size imgSize = calibImage.size();
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;

    /** Detect the markers on the board*/
    cv::aruco::detectMarkers(calibImage, dictionary_, markerCorners, markerIds, parameters_, rejectedCandidates);

    /** Define calibration variables*/
    std::vector<std::vector<cv::Point2f>> allCornersConcatenated;
    std::vector<int> allIdsConcatenated;
    std::vector<int> markerCounterPerFrame;
    markerCounterPerFrame.reserve(allCorners.size());

    /** Populate calibration variables*/
    for (unsigned int i = 0; i < allCorners.size(); i++)
    {
        markerCounterPerFrame.push_back((int)allCorners[i].size());
        for (unsigned int j = 0; j < allCorners[i].size(); j++)
        {
            allCornersConcatenated.push_back(allCorners[i][j]);
            allIdsConcatenated.push_back(allIds[i][j]);
        }
    }

    // DEBUG ONLY
    std::cout << "Corners size = " << markerCorners.size() << "\n";
    std::cout << "Ids size = " << markerIds.size() << "\n";
    // std::cout << "Corners size = " << allCorners.size() << "\n";
    // std::cout << "Ids size = " << allIds.size() << "\n";

    /** Define calibration output*/
    double repError;

    /** Calibrate the camera*/
    // repError = cv::aruco::calibrateCameraAruco(allCornersConcatenated, allIdsConcatenated, markerCounterPerFrame, chBoard, imgSize, cameraMatrix_,
    //                                            distCoeffs_, rvecs_, tvecs_);
    // repError = cv::aruco::calibrateCameraAruco(markerCorners, markerIds, markerCounterPerFrame, chBoard, imgSize, cameraMatrix_,
    //                                            distCoeffs_, rvecs_, tvecs_);

    // double repError = calibrateCamera(allObjectPoints, allImagePoints, imageSize, cameraMatrix, distCoeffs,noArray(), noArray(), noArray(), noArray(), noArray(), calibrationFlags);

    // cv::solvePnP();
    // https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html
    return repError;
}