#ifndef TAGRECOG_H
#define TAGRECOG_H
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco/dictionary.hpp>
// #include <opencv2/objdetect/
#include <opencv2/aruco.hpp>

class TagRecog
{
public:
    /**
     * @brief Constructor
     */
    TagRecog();

    /**
     * @brief Destructor
     */
    ~TagRecog();

    /**
     * @brief Detect AR tag marker
     * @param[in] image Image from webcam
     * @return Number of tag observed OR return -1 if no tag
     */
    double detectMarker(cv::Mat image);
};

#endif