#include "markers.h"
#include <math.h>
#include <vector>
#include <iostream>

Markers::Markers(ros::NodeHandle nh)
{
    // subRGBD_ = nh_.subscribe("/camera/color/image_raw", 1000, &Markers::RGBDCallback, this);
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    parameters_ = cv::aruco::DetectorParameters::create();
}

Markers::Markers()
{
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    parameters_ = cv::aruco::DetectorParameters::create();
}

Markers::~Markers()
{
}

void Markers::RGBDCallback(const sensor_msgs::Image::ConstPtr &msg)
{
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
    std::cout << "Corners size = " << allCorners.size() << "\n";
    std::cout << "Ids size = " << allIds.size() << "\n";

    /** Define calibration output*/
    double repError;

    /** Calibrate the camera*/
    // repError = cv::aruco::calibrateCameraAruco(allCornersConcatenated, allIdsConcatenated, markerCounterPerFrame, chBoard, imgSize, cameraMatrix_,
    //                                            distCoeffs_, rvecs_, tvecs_);
    repError = cv::aruco::calibrateCameraAruco(markerCorners, markerIds, markerCounterPerFrame, chBoard, imgSize, cameraMatrix_,
                                               distCoeffs_, rvecs_, tvecs_);

    return repError;
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
// std::pair<std::vector<int>,std::vector<std::vector<cv::Point2f>>> Markers::detectMarker(cv::Mat image)
{
    // std::vector<int, std::allocator<int>> markerIds;
    std::vector<int> markerIds;

    // Load the relevant dictionary
    // cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    // const cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    // cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;

    // cv::aruco::ArucoDetector detector(dictionary,detectorParams);

    // cv::aruco::detectMarkers(image,dictionary,markerCorners,markerIds);
    cv::aruco::detectMarkers(image, dictionary_, markerCorners, markerIds, parameters_, rejectedCandidates);

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

void Markers::runMarkerDetection(bool running, ImageConverter ic)
{
    unsigned int counter = 0;
    // ImageConverter ic;
    while (running == true)
    {
        counter++;
        if (counter == 10)
        {
            /** Detect marker IDs*/
            std::vector<int> detected_ids = Markers::detectMarker(ic.rgbd_ptr_->image);
            std_msgs::UInt16MultiArray msgIds;

            // Initialise dimensions
            msgIds.layout.dim.push_back(std_msgs::MultiArrayDimension());
            msgIds.layout.dim[0].size = detected_ids.size();
            msgIds.layout.dim[0].stride = 1;
            msgIds.layout.dim[0].label = "id";

            // Populate data
            msgIds.data.clear();
            msgIds.data.insert(msgIds.data.end(), detected_ids.begin(), detected_ids.end());

            // Publish the marker IDs
            // pubIds.advertise()

            /** Detect marker poses*/
            /** @todo*/
            counter = 0;
        }
    }
}

void Markers::runCalibration(bool running)
{
    unsigned int counter = 0;
    ImageConverter ic;
    while (running == true)
    {
        counter++;
        if (counter == 1)
        {
            double repError = Markers::calibrate(ic.rgbd_ptr_->image);
            counter = 0;
        }
    }
}