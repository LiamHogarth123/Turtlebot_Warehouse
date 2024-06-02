#include "calibration.h"

/** Calculate camera parameters*/
void Calibration::calculateCameraParameters(const std::vector<cv::Mat> &calibrationImages, const cv::Size &boardSize, float squareLength, float markerLength, cv::Mat &cameraMatrix, cv::Mat &distCoeffs)
{
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::Ptr<cv::aruco::CharucoBoard> charucoBoard = cv::aruco::CharucoBoard::create(boardSize.width, boardSize.height, squareLength, markerLength, dictionary);

    std::vector<std::vector<cv::Point2f>> allCorners;
    std::vector<std::vector<int>> allIds;
    cv::Size imgSize;

    for (const cv::Mat &image : calibrationImages)
    {
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners;

        // Detect ArUco markers in the image
        cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds);

        // Refine detected markers with the ChArUco board
        if (!markerIds.empty())
        {
            cv::Mat currentCharucoCorners, currentCharucoIds;
            cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, image, charucoBoard, currentCharucoCorners, currentCharucoIds);

            if (currentCharucoCorners.total() > 0)
            {
                std::vector<cv::Point2f> corners;
                corners.assign((cv::Point2f *)currentCharucoCorners.datastart, (cv::Point2f *)currentCharucoCorners.dataend);
                allCorners.push_back(corners);

                std::vector<int> ids;
                ids.assign((int *)currentCharucoIds.datastart, (int *)currentCharucoIds.dataend);
                allIds.push_back(ids);

                if (imgSize == cv::Size())
                {
                    imgSize = image.size();
                }
            }
        }
    }

    if (allCorners.size() < 10)
    {
        std::cerr << "Not enough corners detected for calibration. At least 10 valid images are required." << std::endl;
        return;
    }

    // Prepare object points
    std::vector<std::vector<cv::Point3f>> allObjectPoints(1);
    for (int i = 0; i < boardSize.height; ++i)
    {
        for (int j = 0; j < boardSize.width; ++j)
        {
            allObjectPoints[0].emplace_back(j * squareLength, i * squareLength, 0.0f);
        }
    }
    allObjectPoints.resize(allCorners.size(), allObjectPoints[0]);

    std::vector<cv::Mat> rvecs, tvecs;
    double rms = cv::aruco::calibrateCameraCharuco(allCorners, allIds, charucoBoard, imgSize, cameraMatrix, distCoeffs, rvecs, tvecs);

    std::cout << "Re-projection error reported by calibrateCameraCharuco: " << rms << std::endl;
    std::cout << "Camera Matrix: \n"
              << cameraMatrix << std::endl;
    std::cout << "Distortion Coefficients: \n"
              << distCoeffs << std::endl;
}