
#include "ros/ros.h"
#include "method.h"
#include "boundarydetection.h"
#include "errorcalc.h"
#include "tagrecog.h"
#include <iostream>
#include <thread>


int main(int argc, char **argv){


  // ros::init(argc, argv, "mapping_john");

  // ros::NodeHandle nh;

  /**
   * Let's start seperate thread first, to do that we need to create object
   * and thereafter start the thread on the function desired
   */
  // std::shared_ptr<Method> method(new Method(nh));
  // std::thread t(&Method::seperateThread, method);

  // BoundaryDetection boundary;

  // cv::Mat input = cv::imread("/home/john/Desktop/Turtlebot_Warehouse/mapping_john/test/test_blue_static1.jpg");

  // double flag = boundary.detectColour(input);

  // std::cout << flag << "\n";

  TagRecog tag;

  // tag.drawMarker(17);

  cv::Mat input = cv::imread("/home/john/Desktop/Turtlebot_Warehouse/mapping_john/test/test_marker_17.jpg");

  std::vector<int> tag_ids = tag.detectMarker(input);

  // std::cout << tag_ids;

  // std::thread t(&Sample::control,sample);

  // ros::spin();

  // ros::shutdown();

  // t.join();

  return 0;
}

