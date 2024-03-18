#ifndef TAGRECOG_H
#define TAGRECOG_H
#include "ros/ros.h"
#include <opencv2/highgui.hpp>
// #include <opencv2/objdetect/aruco_detector.hpp>

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
    int detectMarker(long image);
};

#endif