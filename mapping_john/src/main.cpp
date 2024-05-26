#include "ros/ros.h"
#include "method.h"
#include "boundarydetection.h"
#include "errorcalc.h"
#include "markers.h"
#include "imageconverter.h"
#include <iostream>
#include <thread>

int main(int argc, char **argv)
{
  /** Initialise node*/
  ros::init(argc, argv, "mapping_john");

  // /** Create node handle*/
  // ros::NodeHandle nh;

  /** Read namespace*/
  std::string tb;

  // if (nh.getParam("tb", tb))
  // {
  //   ROS_INFO("The 'tb' namespace is %s", tb.c_str());
  // }
  // else
  // {
  //   std::cout << "No 'tb' namespace set!" << std::endl;
  // }

  if (argc > 1) {
    tb = argv[1]; // Read the first argument as 'tb'
    ROS_INFO("The 'tb' argument is %s", tb.c_str());
  } else {
    std::cout << "No 'tb' argument provided!" << std::endl;
  }

  /** Create node handle*/
  ros::NodeHandle nh;

  // if (argc > 1) // Check if at least one argument is provided
  // {
  //   tb = argv[1]; // Read the first argument as 'tb'
  //   ROS_INFO("The 'tb' argument is %s", tb.c_str());
  // }
  // else
  // {
  //   std::cout << "No 'tb' argument provided!" << std::endl;
  // }

  /** Initialise image converter*/
  ImageConverter ic(nh, tb);

  /** Initialise boundary detector*/
  BoundaryDetection boundary(nh, tb);

  /** Initialise marker functionality*/
  Markers markers(nh, tb);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    cv::Mat rgbdInput = ic.getRGBD();
    cv::Mat camInput = ic.getCam();
    std::cout << "running" << std::endl;
    if (!rgbdInput.empty())
    {
      std::vector<int> marker_ids = markers.detectMarker(rgbdInput);
      markers.markerPose(1);
    }
    else
    {
      std::cout << "no RGBD image" << std::endl;
    }

    if (!camInput.empty())
    {
      boundary.runBoundaryDetection(camInput);
    }
    else
    {
      std::cout << "no webcam image" << std::endl;
    }

    /** Process any ROS callbacks*/
    ros::spinOnce();

    /** Maintain loop rate*/
    loop_rate.sleep();
  }

  ros::spin();

  ros::shutdown();

  return 0;
}