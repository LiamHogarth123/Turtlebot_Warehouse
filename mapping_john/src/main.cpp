
#include "ros/ros.h"
#include "method.h"
#include "boundarydetection.h"
#include "errorcalc.h"
#include "markers.h"
#include "imageconverter.h"
#include <iostream>
#include <thread>


int main(int argc, char **argv){


  ros::init(argc, argv, "mapping_john");

  ros::NodeHandle nh;

  /**
   * Let's start seperate thread first, to do that we need to create object
   * and thereafter start the thread on the function desired
   */
  // std::shared_ptr<Method> method(new Method(nh));
  // std::thread t(&Method::seperateThread, method);

  /** Convert images*/
  ImageConverter ic;

  cv::imshow("Window", ic.cam_ptr_->image);
  cv::waitKey(0);

  /** Detect boundary*/
  // BoundaryDetection boundary;

  // /** @test = Red Static #2*/
  // cv::Mat input = cv::imread("/home/john/Desktop/Turtlebot_Warehouse/mapping_john/test/test_red_static2.jpg");

  // double flag = boundary.detectColour(input);

  // std::cout << "flag = " << flag << std::endl;

  // Markers markers;

  /** Draw marker*/
  // markers.drawMarker(17);

  /** Detect marker*/
  // cv::Mat input = cv::imread("/home/john/Desktop/Turtlebot_Warehouse/mapping_john/test/test_marker_17.jpg");
  // std::vector<int> marker_ids = markers.detectMarker(input);

  /** Calibrate*/
  // cv::Mat calibImage = cv::imread("/home/john/Desktop/Turtlebot_Warehouse/mapping_john/test/test_charuco_online.jpg");
  // cv::imshow("Window",calibImage);
  // cv::waitKey(0);
  // double output = markers.calibrate(calibImage);
  
  // std::vector<int> marker_ids = markers.detectMarker(calibImage);

  // std::thread t(&Sample::control,sample);

  ros::spin();

  ros::shutdown();

  // t.join();

  return 0;
}

