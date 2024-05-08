
#include "ros/ros.h"
#include "method.h"
#include <thread>
#include "yaml-cpp/yaml.h"


int main(int argc, char **argv){


  ros::init(argc, argv, "multiple_turtlebot_integration");

  ros::NodeHandle nh;

  /**
   * Let's start seperate thread first, to do that we need to create object
   * and thereafter start the thread on the function desired
   */
  std::shared_ptr<Method> method(new Method(nh));
  std::thread t(&Method::separateThread, method);

  // std::thread t(&Sample::control,sample);

  ros::spin();

  ros::shutdown();

  t.join();

  return 0;
}

