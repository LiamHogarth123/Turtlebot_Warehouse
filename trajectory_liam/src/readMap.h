/**
 @file sample.h
 @brief Header file for the Sample class.
*/

#ifndef READMAP_H
#define READMAP_H
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <atomic>
#include <mutex>
#include <ros/package.h>
#include <geometry_msgs/Point.h>


//Keep only the headers needed






struct MapSpecs {
    cv::Mat mapImage;
    std::string image;
    float resolution;
    std::vector<float> origin;
    int negate;
    float occupied_thresh, free_thresh;
};

class readMap
{
public:

    readMap();

    void read_Map_data(const std::string& mapFile);

    MapSpecs loadMapSpecs( MapSpecs map);

    cv::Mat Load_Map(const std::string& mapFile);

    std::vector<geometry_msgs::Point> Found_Object_Points();









    MapSpecs map;
};

  



#endif // READMAP_H