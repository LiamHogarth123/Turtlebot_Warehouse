#include <yaml-cpp/yaml.h>
#include <iostream>
#include <string>
#include "opencv2/opencv.hpp"
#include "readMap.h"


readMap::readMap(){
    
}


void readMap::read_Map_data(const std::string& yamlFile, const std::string& mapFile){
    MapSpecs map;
    map = loadMapSpecs(yamlFile, map);
    map.mapImage = Load_Map(mapFile);

}

MapSpecs readMap::loadMapSpecs(const std::string& yamlFile, MapSpecs map) {
    YAML::Node config = YAML::LoadFile(yamlFile);

    
    map.image = config["image"].as<std::string>();
    map.resolution = config["resolution"].as<float>();
    map.origin = config["origin"].as<std::vector<float>>();
    map.negate = config["negate"].as<int>();
    map.occupied_thresh = config["occupied_thresh"].as<float>();
    map.free_thresh = config["free_thresh"].as<float>();

    return map;
}

cv::Mat readMap::Load_Map(const std::string& mapFile){
    cv::Mat mapImage = cv::imread(mapFile, cv::IMREAD_GRAYSCALE);
    if(mapImage.empty()) {
        std::cout << "Could not read the map file: " << specs.image << std::endl;
    }
    mapImage.
    return mapImage;
}

// std::vector<geometry_msgs::Point> radMap::Found_Object_Points(){
//     std::vector<geometry_msgs::Point> objects;
//     geometry_msgs::Point temp;
//     for(int i = 0; i < map.mapImage.rows; i++) {
//         for(int j = 0; j < map.mapImage.cols; j++) {
//             // Assuming mapImage is a grayscale image (CV_8UC1)
//             uchar cellValue = map.mapImage.at<uchar>(i, j);
//             if(cellValue < (255 * map.occupied_thresh)) {
//                 x = map.origin_x + (j * map.resolution);
//                 y = map.origin_y + (i * map.resolution);

//                 objects.push_back(temp);

//                 // This cell is considered occupied
//                 // Convert (i, j) to real-world coordinates (x, y) here
//             }
//         }
//     }
//     return objects;
// }




