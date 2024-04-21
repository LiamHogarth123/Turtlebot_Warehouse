
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
  // ImageConverter ic;

  // std::cout << "yee" << std::endl;

  // cv::imshow("Window", ic.cam_ptr_->image);
  // cv::waitKey(0);

  // /** Detect boundary*/
  // BoundaryDetection boundary;

  ImageConverter ic(nh);

  // cv::Mat input = ic.getImage();

  // if (!input.empty())
  // {
  //   cv::imshow("Window",input);
  //   cv::waitKey(0);
  // }

  cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);

    // Main loop
    ros::Rate loop_rate(10);  // Adjust the loop rate as needed
    while (ros::ok()) {
        // Retrieve image from ImageConverter
        cv::Mat image = ic.getImage();

        // Check if image is not empty
        if (!image.empty()) {
            // Display the image
            cv::imshow("Image", image);
        }

        // Check for key press or ROS events
        char key = cv::waitKey(1);
        if (key == 27 || !ros::ok()) {  // Exit on ESC key press or ROS shutdown
            break;
        }

        // Process any ROS callbacks
        ros::spinOnce();

        // Maintain loop rate
        loop_rate.sleep();
    }

    // Clean up
    cv::destroyAllWindows();

  // if (ic.current_image_.size() != 0)
  // {
  //   cv::imshow("Window",ic.current_image_);
  //   cv::waitKey(0);
  // }
  

  // std::cout << "haw" << std::endl;

  // double flag = boundary.runBoundaryDetection(1);

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
  // cv::Mat input = cv::imread("/home/john/Desktop/Turtlebot_Warehouse/mapping_john/test/test_tags_long_alley.jpg");
  // std::vector<int> marker_ids = markers.detectMarker(input);

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

