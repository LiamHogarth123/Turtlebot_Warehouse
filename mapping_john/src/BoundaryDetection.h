#ifndef BOUNDARYDETECTION_H
#define BOUNDARYDETECTION_H
#include "ros/ros.h"

class BoundaryDetection
{
public:
    /**
     * @brief Constructor
     */
    BoundaryDetection();

    /**
     * @brief Destructor
     */
    ~BoundaryDetection();

    /**
     * @brief Detect boundary colour
     * @param[in] image Image from webcam
     * @return True if boundary detected, else false
     */
    bool detectColour(long image);
};

#endif