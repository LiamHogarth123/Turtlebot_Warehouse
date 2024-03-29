#include "method.h"
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <time.h>
#include <nav_msgs/Odometry.h>
#include <image_data_struct.h>
// #include <kobuki_msgs/DigitalOutput.h>
#include "ros/ros.h"

#include <fstream>



using std::cout;
using std::endl;

Method::Method(ros::NodeHandle nh) :
  nh_(nh)
  // laserProcessingPtr_(nullptr)
{
//Ros construction
//Subscribing to TurtleBot3 ROS features

 Threading_switch = false;
 debuggingMode = false;
 telop_mode = false;



 goal_index = 0;

// Robot 1 -----------------------------------------------------
  sub1_ = nh_.subscribe("tb3_0/odom", 1000, &Method::odomCallback,this);

  sub2_ = nh_.subscribe("tb3_0/scan", 10, &Method::LidaCallback,this);

  sub3_ = nh_.subscribe("tb3_0/camera/rgb/image_raw", 1000, &Method::RGBCallback, this);

  sub4_ = nh_.subscribe("tb3_0/camera/depth/image_raw", 1000, &Method::ImageDepthCallback, this);

  cmd_velocity_tb1 = nh_.advertise<geometry_msgs::Twist>("tb3_0/cmd_vel",10);

  // Robot 2 guider ---------------------

  sub5_ = nh_.subscribe("tb3_1/odom", 1000, &Method::guiderOdomCallback,this);

  cmd_velocity_tb2 = nh.advertise<geometry_msgs::Twist>("tb3_1/cmd_vel",10);


  
}

/**
 * @brief Control the robots based on user input and threading preferences.
 *
 * This function allows the user to specify the control method for the robots and whether multi-threading
 * should be used. It provides options for reading goals, providing a single goal, or manually controlling
 * the guider robot. Based on the user's input, it configures the control method and threading accordingly.

 * The function starts by presenting the user with a menu to select the desired control method. The available
 * options are:
 * 1. Use provided goals (readGoal function).
 * 2. Provide a single goal.
 * 3. Control the guider robot with user input (teleoperation mode).

 * Depending on the selected option, the function configures the control method accordingly. If multi-threading
 * is chosen, it runs 'multiThread' for concurrent control. In teleoperation mode, it spawns a thread to run
 * 'followingRobotThread' and a separate thread for teleoperation. In other cases, it runs 'singleThread' to
 * control the robots sequentially. User preferences for multi-threading are considered when setting 'Threading_switch'.

 * @note This function serves as the entry point for controlling the robots, allowing users to specify the
 * control method and threading settings.
 */
void Method::seperateThread() {
  //User input
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  std::string userInput;
  std::cout << "Please enter control method"<< std::endl;
  std::cout << "1. use provide goals"<< std::endl;
  std::cout << "2. provide a goal"<< std::endl;
  std::cout << "3. control the guider robot with user input"<< std::endl;
  std::cin >> userInput;
  int input_int = std::stoi(userInput);

  switch (input_int) {
    case 1: {
      default_goals = true;
      break;
      }
    case 2:{
      Leader_goals.clear();
      geometry_msgs::Point temp;
      std::cout << "enter x"<< std::endl;
      std::cin >> userInput;
      temp.x = std::stoi(userInput);
      std::cout << "enter y"<< std::endl;
      std::cin >> userInput;
      temp.y = std::stoi(userInput);
      Leader_goals.push_back(temp);
      break;
      }
    case 3:{
      telop_mode = true;
      break;
      }
    default:{
    std::cout << "invalid input - therefore using default goals"<< std::endl;
    }
  }

  



  std::cout << "Please input environment type (h = house, e = empty)"<< std::endl;
  std::cin >> userInput;
  if (userInput == "h"){
    
    house = true;
    if (default_goals){
      readGoal(true);
    }
  }
  else {
    house = false;
    if (default_goals){
      readGoal(false);
    }
  }





  //Code start
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  if (Threading_switch){
    multiThread();
  }
  else if(telop_mode){
    while (true){
      
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

  }
  else{
    while (true){
      singleThread();
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
  }
}


/**
 * @brief Execute both following and guider robot control sequentially in a single thread.
 *
 * This function sequentially executes both the following robot control and the guider robot control
 * within the same thread. It allows you to manage the behavior of both robots in a single-threaded fashion.

 * The function starts by calling 'followingRobotRun' to control the behavior of the following robot, and
 * then it proceeds to call 'guiderBotMovement' to manage the guider robot's movement.

 * This single-threaded approach is suitable when you do not require concurrent control of the robots and
 * wish to execute their behaviors sequentially in a coordinated manner.

 * @note This function is used for sequential execution of control logic for both the following and guider robots,
 * one after the other, within a single thread.
 */
void Method::singleThread() {
 
  guiderBotMovement();
}

/**
 * @brief Start multi-threaded control for the guider and following robots.
 *
 * This function initiates multi-threaded control for the guider and following robots. It creates a new thread
 * to run the 'guiderBotMovement' function, which controls the guider robot's movement. Meanwhile, in the main
 * thread, it continuously calls the 'followingRobotRun' function to control the behavior of the following robot.

 * The function begins by creating a new thread named 'Lead_robot_thread' to execute the 'guiderBotMovement'
 * function, allowing simultaneous control of the guider robot. In the main thread, it enters an infinite loop,
 * continuously executing the 'followingRobotRun' function, which manages the behavior of the following robot.

 * This multi-threaded approach enables concurrent control of both robots, ensuring that the guider and following
 * robots can operate independently in parallel.

 * @note This function sets up a multi-threaded environment for controlling the guider and following robots,
 * enabling concurrent execution of their respective behaviors.
 */
void Method::multiThread(){  
  std::thread Lead_robot_thread(&Method::guiderBotMovement, this);
  while (true){
    
  }
}

/**
 * @brief Continuously execute the following robot control thread.
 *
 * This function continuously executes the following robot control thread by repeatedly calling
 * the 'followingRobotRun' function in an infinite loop. It ensures that the following robot remains
 * responsive and actively follows the leader or performs the designated tasks.

 * The function enters a never-ending loop where 'followingRobotRun' is repeatedly invoked. This is a
 * typical pattern for controlling a thread dedicated to a specific task, ensuring that the thread runs
 * continuously and stays responsive to external events.

 * @note This function is responsible for maintaining the continuous operation of the following robot's
 * control thread, allowing it to track and follow the leader or perform other relevant actions without
 * interruption.
 */
void Method::followingRobotThread(){
  while(true){
    
  }
}


/**
 * @brief Control the movement of a guider robot based on leader goals.
 *
 * This function is responsible for controlling the movement of a guider robot based on the leader's goals.
 * It allows the guider robot to navigate towards specified goals while taking into account the current
 * threading state (Threading_switch).

 * If threading is enabled (Threading_switch is true), the function performs the following steps:
 * - Pauses for 500 milliseconds to allow for synchronization.
 * - Iterates through the list of leader goals (Leader_goals) and sets the guider's goal accordingly.
 * - Calculates the trajectory to reach the goal using the 'reachGoal' function from GuiderGPS.
 * - Sends the calculated trajectory to control the guider's movement using 'Send_cmd_tb2'.

 * If threading is not enabled (Threading_switch is false), the function follows a similar process:
 * - Sets the guider's goal based on the goal at the current 'goal_index' in the list of leader goals.
 * - Calculates the trajectory for the guider to reach this goal using 'guiderReachGoal' function.
 * - Sends the trajectory to control the guider's movement using 'Send_cmd_tb2'.
 * - Checks if the guider has reached the goal using 'goal_hit', and if so, increments the goal_index.

 * @note This function is a key part of the guider robot's behavior, allowing it to follow leader goals and
 * adjust its movement based on threading settings and goal-reaching status.
 */
void Method::guiderBotMovement(){
  
  if (Threading_switch){
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    for (int i = 0; i < Leader_goals.size(); i++){
      
      guiderGoal = Leader_goals.at(i);

      GuiderGPS.newGoal(guiderGoal, guider_Odom);
      geometry_msgs::Twist traj = GuiderGPS.reachGoal();
      Send_cmd_tb2(traj);
    }
  }
  else{

    // std::cout<< "guiderbotopening" << std::endl;
    geometry_msgs::Point guiderGoal;

    guiderGoal = Leader_goals.at(goal_index);
   

    GuiderGPS.newGoal(guiderGoal, guider_Odom);
    geometry_msgs::Twist guiderTraj = GuiderGPS.reachGoal();
    Send_cmd_tb2(guiderTraj);
    if (GuiderGPS.goal_hit(guiderGoal, guider_Odom)){
      if (goal_index != Leader_goals.size()){
         goal_index++;
      }
    }
  }
}


// Publishing functions 
///////////////////////////////////////////////////////////////////////////////////////////
void Method::Send_cmd_tb1(geometry_msgs::Twist intructions){
  cmd_velocity_tb1.publish(intructions);
}

void Method::Send_cmd_tb2(geometry_msgs::Twist intructions){
  cmd_velocity_tb2.publish(intructions);
}



//callbacks
///////////////////////////////////////////////////////////////////////////////////////////////

void Method::odomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg){
  std::unique_lock<std::mutex> lck3 (odom_locker);
  Current_Odom = *odomMsg;
}

void  Method::RGBCallback(const sensor_msgs::Image::ConstPtr& Msg){
  std::unique_lock<std::mutex> lck3 (RGB_locker);
  updated_RGB = *Msg;

}

void Method::LidaCallback(const sensor_msgs::LaserScan::ConstPtr& Msg){
  std::unique_lock<std::mutex> lck3 (Lida_locker);
  updated_Lida = *Msg;
}

void Method::ImageDepthCallback(const sensor_msgs::Image::ConstPtr& Msg){
  std::unique_lock<std::mutex> lck3 (ImageDepth_locker);
  updated_imageDepth = *Msg;
}

void Method::guiderOdomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg){
  std::unique_lock<std::mutex> lck3 (odom_locker2);
  guider_Odom = *odomMsg;
}

RobotData Method::Update_Robot_Image_data(){
  
  std::unique_lock<std::mutex> lck1 (RGB_locker);
  std::unique_lock<std::mutex> lck2 (Lida_locker);
  std::unique_lock<std::mutex> lck3 (ImageDepth_locker);

  Image_data.depthImage = updated_imageDepth;
  Image_data.laserScan = updated_Lida;
  Image_data.rgbImage = updated_RGB;

  return Image_data;
}




/**
 * @brief Read and parse goal points from a text file.
 *
 * This function reads goal points from a text file, parses the contents, and populates the 'Leader_goals' vector
 * with the extracted points. It allows you to specify the filename of the text file containing goal information.

 * @return 'true' if the goal points were successfully read and parsed, 'false' otherwise.

 * The function begins by defining the filename of the text file that contains the goal points. It attempts to open
 * the file for reading and checks for any errors during the file opening process. If the file cannot be opened, it
 * reports an error and returns 'false' as an error code.

 * If the file is successfully opened, the function reads each line from the file and parses it to extract the
 * 'x', 'y', and 'z' coordinates of a goal point. If parsing is successful, the extracted point is added to the
 * 'Leader_goals' vector. If any line cannot be parsed, an error message is displayed.

 * After reading and parsing all the goal points, the function closes the file and prints the parsed points to the console.

 * @note This function is crucial for loading and configuring goal points from a text file, enabling the robot to
 * navigate to predefined locations.
 */

// Read/Load goals
//////////////////////////////////////////////////////////////////////
bool Method::readGoal(bool house) {
    // Define the filename of the text file you want to read
    // std::string filename = "../data/Goals.TXT";
    std::string filename;
    std::string default_filename;
    std::string path;
    
    if (house){
      path = ros::package::getPath("turtleboi");
      path += "/data/"; //Looking at data subfolder
      default_filename = path + "House.TXT";
      nh_.param<std::string>("goals", filename, default_filename);
    }
    else {
      path = ros::package::getPath("turtleboi");
      path += "/data/"; //Looking at data subfolder
      default_filename = path + "Goals.TXT";
      nh_.param<std::string>("goals", filename, default_filename);
    }


    // Open the file for reading
    std::ifstream file(filename, std::ios::in);

    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        if (file.fail()) {
            std::cerr << "Error code: " << file.rdstate() << std::endl;
        }
        return false; // Return an error code
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream lineStream(line);
        geometry_msgs::Point point;
        if (lineStream >> point.x >> point.y >> point.z) {
          Leader_goals.push_back(point);
        }
        
        else {
          std::cerr << "Error parsing line: " << line << std::endl;    
        }
    }

    file.close();

    for (const auto& point : Leader_goals) {
        std::cout << "Point: (" << point.x << ", " << point.y << ", " << point.z << ")\n";
    }

    return true; 
}



geometry_msgs::Point Method::motion_dection(std::vector<geometry_msgs::Point> old_points, std::vector<geometry_msgs::Point> newPoints){
  geometry_msgs::Point Motion_point;
  double movementThreshold = 0.01;
  for (const auto& newPoint : newPoints) {
    bool pointMatched = false;
    for (auto& oldPoint : old_points) {
        if (distance(oldPoint, newPoint) < movementThreshold) {
          pointMatched = true;
          oldPoint = newPoint; // Update old point with the new data
          break;
        }
      }
    if (!pointMatched) {
      old_points.push_back(newPoint); // New point detected
      std::cout << "New point detected: (" << newPoint.x << ", " << newPoint.y << ")\n";
      // Perform actions for new points
    }
  }

    // Identify moving points that have changed their positions
  for (const auto& oldPoint : old_points) {
    for (const auto& newPoint : newPoints) {
      if (distance(oldPoint, newPoint) > movementThreshold) {
        std::cout << "Moving point detected: (" << oldPoint.x << ", " << oldPoint.y << ")\n";
        Motion_point = newPoint;
        break;
      }
    }
  }
  
  return Motion_point;
}


double Method::distance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
  return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}