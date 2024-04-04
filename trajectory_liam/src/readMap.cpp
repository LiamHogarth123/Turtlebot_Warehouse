
#include <iostream>
#include <string>
#include <opencv2/core/mat.hpp>
#include "readMap.h"


readMap::readMap() {
    // Empty constructor body
}


void readMap::read_Map_data( const std::string& mapFile){
   
    map = loadMapSpecs(map);
    map.mapImage = Load_Map(mapFile);

}

MapSpecs readMap::loadMapSpecs( MapSpecs map) {
   
    std::vector<float> temp;
    temp.push_back(-10.000000);
    temp.push_back(-10.000000);
    temp.push_back(0.000000);
    
    map.image = "/home/dan/map.pgm";
    map.resolution = 0.050000;
    map.origin = temp;
    map.negate = 0;
    map.occupied_thresh = 0.65;
    map.free_thresh = 0.196;

    return map;
}

cv::Mat readMap::Load_Map(const std::string& mapFile){
    cv::Mat mapImage = cv::imread(mapFile, cv::IMREAD_GRAYSCALE);
    if(mapImage.empty()) {
        std::cout << "Could not read the map file: "<< std::endl;
    }

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




