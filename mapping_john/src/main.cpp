#include "ros/ros.h"
#include "method.h"
#include "boundarydetection.h"
#include "errorcalc.h"
#include "markers.h"
#include "imageconverter.h"
#include "calibration.h"
#include <iostream>
#include <thread>

int main(int argc, char **argv)
{
  /** Initialise node*/
  ros::init(argc, argv, "mapping_john");

  /** Create node handle*/
  ros::NodeHandle nh;

  /** Read namespace*/
  std::string tb;

  /** Store value if argument passed*/
  if (argc > 1)
  {
    tb = argv[1];
    ROS_INFO("The 'tb' argument is %s", tb.c_str());
  }
  /** Otherwise continue with no namespace*/
  else
  {
    std::cout << "No 'tb' argument provided!" << std::endl;
  }

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