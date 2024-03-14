/**
 @file sample.h
 @brief Header file for the Sample class.
*/

#ifndef SAMPLE_H
#define SAMPLE_H


#include <atomic>
#include <mutex>
#include "opencv2/opencv.hpp"
#include <ros/package.h>
#include <geometry_msgs/Point.h>


//Keep only the headers needed






struct MapSpecs {
    cv::mat mapImage;
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

    void read_Map_data(const std::string& yamlFile, const std::string& mapFile);

    MapSpecs loadMapSpecs(const std::string& yamlFile, MapSpecs map);

    cv::Mat Load_Map(const std::string& mapFile);

    std::vector<geometry_msgs::Point> Found_Object_Points();

 


public:
    MapSpecs map;






};

  



#endif // SAMPLE_H