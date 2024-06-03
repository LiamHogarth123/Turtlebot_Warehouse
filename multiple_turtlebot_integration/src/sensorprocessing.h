#ifndef LASERPROCESSING_H
#define LASERPROCESSING_H

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>

#include <cmath>
#include <vector>
#include "tf/transform_datatypes.h"
#include <geometry_msgs/PoseArray.h>


class Sensorprocessing
{
public:
  /*! @brief Constructor that allocates internals
   *
   *  @param[in]    laserScan - laserScan to be processed
   */
  Sensorprocessing();

  void Newdata(sensor_msgs::LaserScan temp_data);

  geometry_msgs::Point polarToCart(unsigned int index);

  double findObstacle();

  std::vector<std::pair<float, int>> scanningRange(float scanRange);


  void PrintLaserSpec();

  private:
  sensor_msgs::LaserScan laserScan;
  double Turtlebot_min;
  double Turtlebot_max;

};



#endif // DETECTCABINET_H
