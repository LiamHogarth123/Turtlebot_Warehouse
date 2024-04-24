
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

  /** Initialise image converter*/
  ImageConverter ic(nh);

  /** Initialise boundary detector*/
  BoundaryDetection boundary;

  /** Initialise marker functionality*/
  Markers markers;

  // cv::Mat input = cv::imread("/home/john/Desktop/Turtlebot_Warehouse/mapping_john/test/calibration_front.jpg");

  // if(!input.empty())
  // {
  //   double calibration = markers.calibrate(input);
  // }

  /** Window for showing input*/
  // cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);

  /** @test = Red Dynamic #2*/
  // ros::Rate loop_rate(10);
  // while (ros::ok()) {
  //   cv::Mat input = ic.getCam();
  //   if (!input.empty()) {
  //     double flag = boundary.runBoundaryDetection(1,input);
  //     std::cout << "flag = " << flag << std::endl;
  //     if (!input.empty()) {
  //       // Display the image
  //       cv::imshow("Image", input);
  //     }

  //     // Check for key press or ROS events
  //     char key = cv::waitKey(1);
  //       if (key == 27 || !ros::ok()) {  // Exit on ESC key press or ROS shutdown
  //       break;
  //     }
  //   cv::destroyAllWindows();
  //   }
  //   else{
  //     std::cout << "no image" << std::endl;
  //   }

  //   // Process any ROS callbacks
  //   ros::spinOnce();

  //   // Maintain loop rate
  //   loop_rate.sleep();
  // }

  // std::cout << "naw" << std::endl;

  /** @test = Neutral Static #3*/
  // cv::Mat input = cv::imread("/home/john/Desktop/Turtlebot_Warehouse/mapping_john/test/test_neutral_static3.jpg");

  // double flag = boundary.detectColour(input);

  // std::cout << "flag = " << flag << std::endl;

  // ImageConverter ic;

  // Markers markers;

  /** Draw marker*/
  // markers.drawMarker(17);

  /** Detect marker*/
  // cv::Mat input = cv::imread("/home/john/Desktop/Turtlebot_Warehouse/mapping_john/test/test_marker_17.jpg");
  // cv::Mat input = cv::imread("/home/john/Desktop/Turtlebot_Warehouse/mapping_john/test/calibration_front.jpg");
  cv::Mat input = cv::imread("/home/john/Desktop/Turtlebot_Warehouse/mapping_john/test/test_tags_alley.jpg");
  std::vector<int> marker_ids = markers.detectMarker(input);
  markers.markerPose(0);

  // markers.runMarkerDetection(1, ic);

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

