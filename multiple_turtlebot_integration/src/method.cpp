#include "method.h"
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <time.h>
#include <nav_msgs/Odometry.h>
#include "ros/ros.h"
#include <fstream>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"



Method::Method(ros::NodeHandle nh) :
  nh_(nh)

{

  teleop_mode = false;

  goal_index = 0;

  missionComplete = false;

  pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array",3,false);


  cmd_velocity_tb1 = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",10);

  // Robot 2 guider ---------------------



  cmd_velocity_tb2 = nh.advertise<geometry_msgs::Twist>("tb3_1/cmd_vel",10);


  // Map ros topics
  mapSub = nh_.subscribe("/map", 1000, &Method::mapCallback, this);
  
  mapMetadataSub = nh_.subscribe("/map_metadata", 1000, &Method::mapMetadataCallback, this);

  marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 100);


 
    zero_vel.linear.x = 0.0;
    zero_vel.linear.y = 0.0;
    zero_vel.linear.z = 0.0;
    zero_vel.angular.x = 0.0;
    zero_vel.angular.y = 0.0;
    zero_vel.angular.z = 0.0;

  
  
}

void Method::separateThread() {

  std::cout << "working1" << std::endl;
    while (latestMapData_.data.empty()){
    std::cout << "stuck" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  
  //Create turtlebot section
  /////////////////////////////////////////////////////////

  std::cout << "Turtlebot Creation" << std::endl;
  int numTurtlebot = 2;
  std::vector<DefaultTurtleBot*> turtlebots;
  

  // MAnualy adding transform from odom to map
  geometry_msgs::Transform map1;
  geometry_msgs::Transform map2;


  // Initialize translation to zero
  map2.translation.x = 0.0;
  map2.translation.y = 0.0;
  map2.translation.z = 0.0;

    // Initialize rotation to zero (quaternion zero initialization means no rotation)
    map2.rotation.x = 0.0;
    map2.rotation.y = 0.0;
    map2.rotation.z = 0.0;
    map2.rotation.w = 1.0; 

    // Initialize translation to zero
    map1.translation.x = 0.0;
    map1.translation.y = 0.0;
    map1.translation.z = 0.0;

    // Initialize rotation to zero (quaternion zero initialization means no rotation)
    map1.rotation.x = 0.0;
    map1.rotation.y = 0.0;
    map1.rotation.z = 0.0;
    map1.rotation.w = 1.0; 
  



  ros::NodeHandle nh_1;
  DefaultTurtleBot* tb1 =  new DefaultTurtleBot("tb3_0", nh_1, map1);
  ros::NodeHandle nh_2;
  DefaultTurtleBot* tb2 =  new DefaultTurtleBot("tb3_1", nh_2, map2);
  ros::NodeHandle nh_3;
  DefaultTurtleBot* tb3 =  new DefaultTurtleBot("tb3_2", nh_3, map2);
  std::vector<std::thread> threads;

  
  
  turtlebots.push_back(tb1);
  turtlebots.push_back(tb2);
  turtlebots.push_back(tb3);

  numTurtlebot = turtlebots.size();


  std::cout << "Turtlebot Created-----------------" << std::endl;

  //Automtic turtlebot creation and data collection
  ////////////////////////////////////////////////////////////////////////////////////////////
  // for (int i = 0; i < numTurtlebot; i++){
  //   std::string node_name = "tb" + std::to_string(i);
  //   std::string nodeHandle_name = "nhtb" + std::to_string(i);
  //   ros::NodeHandle nh_i(nodeHandle_name);
  //   turtlebots.emplace_back(node_name, nh_i);
  // }



  //CREATING GPS
  /////////////////////////////////////////////////////////////////////////////////////////////

  // Control TurtleGPS;
  std::vector<Control*> Turtle_Controllers;
  for (size_t j = 0; j < turtlebots.size(); j++) {  
    Turtle_Controllers.push_back(new Control());
  }

  //misson completed boolean vector
  Finished.clear();
  for (size_t k = 0; k < turtlebots.size(); k++){
    Finished.push_back(false);
  }


  //Store turtlebot positions
  ///////////////////////////////////////////////////////////////////////////////////////
  // std::cout << "setting positions" << std::endl;
  std::vector<geometry_msgs::Point> RobotPos;
  for (auto& element : turtlebots) {
    nav_msgs::Odometry temp;
    temp = element->GetCurrent_Odom();
      // std::cout << "stuck" << std::endl;
    RobotPos.push_back(temp.pose.pose.position);
  }

    

  //Brendan task allocation
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  // std::vector<geometry_msgs::Point> RobotPos;
  std::cout << "Task allocation------------------------------" << std::endl;
  TA.SetGoals();
  TA.SetTurtlebotPositions(RobotPos);
  std::vector<std::vector<std::pair<int, geometry_msgs::Point>>> RobotGoals;
  RobotGoals = TA.taskAllocation();

  //  std::cout << "here allocation next state" << std::endl;
  size_t largestSize = 0;
  for (const auto& vec : RobotGoals) {
    if (vec.size() > largestSize) {
      largestSize = vec.size(); // Update the largest size
    }
  }


  //Liam Map generation
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  char user_input;
  bool map_choice;
  bool edit;
 
  std::cout << "Configuring PRM-----------------------------------";
  std::cout << "Would you like a custom map or predefined map Y/N";
  std::cin >> user_input;

  if (user_input == 'y' || user_input == 'Y') {
      map_choice = true;
    } else if (user_input == 'n' || user_input == 'N') {
      map_choice = false;
    } else {
      std::cout << "invalid input deflaut being used" << std::endl;
      map_choice = false;
    }

  std::cout << "Would you like to manually edit the PRM";
  std::cin >> user_input;

  if (user_input == 'y' || user_input == 'Y') {
      edit = true;
    } else if (user_input == 'n' || user_input == 'N') {
      edit = false;
    } else {
      std::cout << "invalid input deflaut being used" << std::endl;
      edit = false;
    }

  prmMap.GeneratePRM(latestMapData_, latestMapMetaData_, map_choice, edit);
  // std::vector<geometry_msgs::Point> trajectory;
  std::vector<std::vector<geometry_msgs::Point>> trajectory(numTurtlebot);
  geometry_msgs::Point goal;
  geometry_msgs::Point start;
  nav_msgs::Odometry odometry_start;

 
  //path aviodance initalisation
  /////////////////////////////////////////
  path_avoidance_ path_checker;
  bool Synchronous;
  std::cout << "system Ready" << std::endl;
  std::cout << "Would you like to run Synchronous turtlebot movement (y)? or Asynchronous (N) ";
  std::cout << "Note Asynchronous is more efficieny has higher risk of collision";
  std::cin >> user_input;

  if (user_input == 'y' || user_input == 'Y') {
      Synchronous = true;
    } else if (user_input == 'n' || user_input == 'N') {
      Synchronous = false;
    } else {
      std::cout << "invalid input deflaut being used" << std::endl;
      Synchronous = true;
    }


  
  //Synchous coding Main loop generation
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (Synchronous){
    for (int i = 0; i < largestSize; i++){

      for (size_t j = 0; j < turtlebots.size(); j++) {
        // std::cout << "index" << j << std::endl;
        auto& element = turtlebots[j];
        // std::cout << "pointer done" << j << std::endl;
        goal = RobotGoals.at(j).at(i).second;
        // std::cout << "goal done" << j << std::endl;

        odometry_start = element->GetCurrent_Odom();
        // std::cout << "odom done" << j << std::endl;
        trajectory.at(j) = prmMap.A_star_To_Goal(odometry_start.pose.pose.position, goal);
      

      }
      
      std::vector<geometry_msgs::Point> combinedPoints;
      for (const auto& points : trajectory) {
        // Append each point to the combinedPoints vector
        for (const auto& point : points) {
          combinedPoints.push_back(point);
        }
      }
 
      
      // Path planning collison avoidance
      /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      std::vector<geometry_msgs::Point> collided_points;
      

      path_checker.Update_paths(trajectory);
      
      int max_iterations = 100;  // Maximum number of iterations to prevent infinite loop
      int iteration_count = 0;
      while (path_checker.find_collisions(collided_points) && iteration_count < max_iterations) {
        iteration_count++;
        // visualisation
        visualization_msgs::MarkerArray tempory = publishMarkers(path_checker.get_all_interpolated_points(), marker_pub);
        publishCollisons(collided_points, marker_pub, tempory);
        tempory.markers.clear();

        //Regenerate Path withouut collisions
        std::vector<std::pair<int, geometry_msgs::Point>> robot_with_collisiosn = path_checker.GetCollisionsWithId();
        // std::cout << "size -----> " << robot_with_collisiosn.size() << std::endl;
        CollisionMap collision_map;
        processCollisions(robot_with_collisiosn, collision_map);
        
        // Iterate over each collision in the collision map
        for (const auto& entry : collision_map) {
          const auto& collision_point = entry.first;
          const auto& robot_pairs = entry.second;

          for (const auto& pair : robot_pairs) {
            int robot1_id = pair.first;
            int robot2_id = pair.second;

            // Determine which robot has the larger trajectory
            int robot_to_replan = (trajectory[robot1_id].size() > trajectory[robot2_id].size()) ? robot1_id : robot2_id;

            // Generate blacklist for re-planning
            std::vector<geometry_msgs::Point> collision_points_for_robot;
            for (const auto& col : robot_with_collisiosn) {
                if (col.first == robot_to_replan) {
                    collision_points_for_robot.push_back(col.second);
                }
            }
            // Recalculate trajectory
            geometry_msgs::Point start = trajectory[robot_to_replan].front();
            geometry_msgs::Point goal = trajectory[robot_to_replan].back();
            trajectory[robot_to_replan] = prmMap.A_star_To_Goal_With_Blacklist(start, goal, collision_points_for_robot);
          }
        }
        path_checker.Update_paths(trajectory);
      }


      // Leader_goals = trajectory;
      goal_index = 0;

      
      //Dan control start
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      std::vector<std::thread> threads;
      


      for (size_t j = 0; j < turtlebots.size(); j++) {
        threads.emplace_back(&Method::Function, std::ref(*this), trajectory.at(j), std::ref(turtlebots.at(j)), std::ref(Turtle_Controllers.at(j)));
        
      }
      for (auto& thread : threads) {
        thread.join();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));

      // std::this_thread::sleep_for(std::chrono::milliseconds(5000000));




    }
  }

  // Asychous turtlebots
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  else if (true) {
    PrmData Gernerated_Map = prmMap.ExportPrmData();


     // Individuaal Turtlebot PRM;
    ///////////////////////////////////////////////
    std::vector<PRM*> Turtle_Maps;
    for (size_t j = 0; j < turtlebots.size(); j++) {  
      Turtle_Maps.push_back(new PRM());
    }
    for (size_t j = 0; j < turtlebots.size(); j++) {
      Turtle_Maps.at(j)->Load_PRM(Gernerated_Map);
    }




    for (size_t j = 0; j < turtlebots.size(); j++) {
      threads.emplace_back(&Method::Function2, std::ref(*this), std::ref(Turtle_Maps.at(j)), std::ref(turtlebots.at(j)), std::ref(Turtle_Controllers.at(j)), RobotGoals.at(j));
      }
    for (auto& thread : threads) {
      thread.join();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));



  }

  // Asychous Turtlebot with Trajectory thread
  else if (false) {

  // }

 
  // for (auto ptr : turtlebots) {
  //   delete ptr;
  }
}


void Method::Function(std::vector<geometry_msgs::Point> trajectory , DefaultTurtleBot* turtleboi, Control* Turtle_GPS){
  

  geometry_msgs::Point targetGoal;
  int loop_interation =0;
  int index = 0;
  bool done = false;
  geometry_msgs::Twist botTraj;
  int falsePositiveCheck;
  // t.join();

 while (!done){
    
    

  // goes to first goal way point to allign itself
  if (loop_interation < 1) {
    targetGoal = trajectory.at(loop_interation+1);
    Turtle_GPS->updateControlParam(targetGoal, calcDistance(turtleboi->GetCurrent_Odom().pose.pose.position, trajectory.back()), turtleboi->GetCurrent_Odom(), turtleboi->Getupdated_Lida());
    botTraj = Turtle_GPS->reachGoal();
    
  }
  else {

    targetGoal = findLookAheadPoint(trajectory, turtleboi->GetCurrent_Odom().pose.pose.position, 0.5);
    Turtle_GPS->updateControlParam(targetGoal, calcDistance(turtleboi->GetCurrent_Odom().pose.pose.position, trajectory.back()), turtleboi->GetCurrent_Odom(), turtleboi->Getupdated_Lida());
    botTraj = Turtle_GPS->reachGoal();

  }

  if (Turtle_GPS->goal_hit(targetGoal, turtleboi->GetCurrent_Odom())){
      loop_interation++;  
    }

  if (Turtle_GPS->goal_hit(trajectory.back(), turtleboi->GetCurrent_Odom())){
      // end of goals reached
      missionComplete = true;
      done = true;
      botTraj.angular.z = 0;
      // adjust TurtleBot to be on top of last goal
      botTraj.linear.x = 0.10;
      turtleboi->Send_cmd_tb1(botTraj);
      std::cout << "Sleeping for 1000ms to adjust position..." << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(3000));
      botTraj.linear.x = 0;
      std::cout << "Goal Reached!!!!!!!!" << std::endl;
    }






    // Checks for boundary and kills program if detected
    if (turtleboi->getBoundaryStatus().data == 1){ // blue detected
      falsePositiveCheck++;
      if (falsePositiveCheck > 3) {
        std::cout << "Boundary Detected!! Seek Operator Assistance" << std::endl;
        while (turtleboi->GetCurrentSpeed() > 0){
          turtleboi->Send_cmd_tb1(zero_vel);
        }

        break;
      }
    } else { // red = 2, nothing = 0
      falsePositiveCheck = 0;
    }


    //if all good
    ///////////////////////////////////////////////////////////////////////////////////////////
    turtleboi->Send_cmd_tb1(botTraj);
  
    // std::cout << "Look-Ahead Point: (" << targetGoal.x << ", " << targetGoal.y << ")" << std::endl;
    // publishLookAheadMarker(targetGoal);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

  }

  // AR tag finding
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // if (goal_list_index < RobotGoals[0].size() - 1){
  //   int count = 0;
  //   while (tagAlignment(RobotGoals[0][goal_list_index], Turtle_GPS->angleToGoal(turtleboi->GetCurrent_Odom(), RobotGoals[0][goal_list_index].second)) == false){
  //     std::this_thread::sleep_for(std::chrono::milliseconds(200));
  //     count++;
  //     if (count == 20){ // roughly looks 86degrees turning in the direction of the tag
  //       std::cout << "Could not find Tag: " << RobotGoals[0][goal_list_index].first << std::endl;
  //       break;
  //     }
  //   }
  // }



  while (turtleboi->GetCurrentSpeed() > 0){
    turtleboi->Send_cmd_tb1(zero_vel);
  }

  // Tag Allignment
  // tagAlignment(std::pair<int, geometry_msgs::Point> temp_tag, double temp_angleToGoal, turtleboi);
  //while (turtleboi->GetCurrentSpeed() > 0){
  //   turtleboi->Send_cmd_tb1(zero_vel);
  // }


  missionComplete = false;
  goal_index = 0;


}


void Method::Function2(PRM* Trajectory_Planner, DefaultTurtleBot* turtleboi, Control* Turtle_GPS, std::vector<std::pair<int, geometry_msgs::Point>> Goals){
  for (int i = 0; i < Goals.size(); i++){
    geometry_msgs::Point Current_Goal = Goals.at(i).second;
    geometry_msgs::Point start = turtleboi->GetCurrent_Odom().pose.pose.position;
    int index = 0;
    bool done = false;
    int falsePositiveCheck = 0;
    int loop_interation = 0;
    geometry_msgs::Point targetGoal;
    geometry_msgs::Twist botTraj;



    std::vector<geometry_msgs::Point> trajectory = Trajectory_Planner->A_star_To_Goal(start, Current_Goal);

  

    while (!done){
      // goes to first goal way point to allign itself
      if (loop_interation < 1) {
        targetGoal = trajectory.at(loop_interation+1);
        Turtle_GPS->updateControlParam(targetGoal, calcDistance(turtleboi->GetCurrent_Odom().pose.pose.position, trajectory.back()), turtleboi->GetCurrent_Odom(), turtleboi->Getupdated_Lida());
        botTraj = Turtle_GPS->reachGoal();
        
      }
      else {

        targetGoal = findLookAheadPoint(trajectory, turtleboi->GetCurrent_Odom().pose.pose.position, 0.5);
        Turtle_GPS->updateControlParam(targetGoal, calcDistance(turtleboi->GetCurrent_Odom().pose.pose.position, trajectory.back()), turtleboi->GetCurrent_Odom(), turtleboi->Getupdated_Lida());
        botTraj = Turtle_GPS->reachGoal();

      }

      if (Turtle_GPS->goal_hit(targetGoal, turtleboi->GetCurrent_Odom())){
          loop_interation++;  
        }

      if (Turtle_GPS->goal_hit(trajectory.back(), turtleboi->GetCurrent_Odom())){
        // end of goals reached
        missionComplete = true;
        done = true;
        botTraj.angular.z = 0;
        // adjust TurtleBot to be on top of last goal
        botTraj.linear.x = 0.10;
        turtleboi->Send_cmd_tb1(botTraj);
        std::cout << "Sleeping for 1000ms to adjust position..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        botTraj.linear.x = 0;
        std::cout << "Goal Reached!!!!!!!!" << std::endl;
      }






      // Checks for boundary and kills program if detected
      if (boundaryStatus.data == 1){ // blue detected
        falsePositiveCheck++;
        if (falsePositiveCheck > 3) {
          std::cout << "Boundary Detected!! Seek Operator Assistance" << std::endl;
          break;
        }
      } else { // red = 2, nothing = 0
        falsePositiveCheck = 0;
      }

      std::cout << botTraj.linear.x << std::endl;
      turtleboi->Send_cmd_tb1(botTraj);
    
      // std::cout << "Look-Ahead Point: (" << targetGoal.x << ", " << targetGoal.y << ")" << std::endl;
      // publishLookAheadMarker(targetGoal);
      
      std::this_thread::sleep_for(std::chrono::milliseconds(200));

    }


    while (turtleboi->GetCurrentSpeed() > 0){
      turtleboi->Send_cmd_tb1(zero_vel);
    }

    // Tag Allignment
    // tagAlignment(std::pair<int, geometry_msgs::Point> temp_tag, double temp_angleToGoal, turtleboi);
    //while (turtleboi->GetCurrentSpeed() > 0){
    //   turtleboi->Send_cmd_tb1(zero_vel);
    // }


    missionComplete = false;
    goal_index = 0;
  }

  // // t.join();
 
}






void Method::turtleMovement(){

    
}




// High level functions DefaultTurtleBot* tb1
//////////////////////////////////////////////////





bool Method::tagAlignment(std::pair<int, geometry_msgs::Point> temp_tag, double temp_angleToGoal, DefaultTurtleBot* tb1){
  // current tag info
  marker_msgs::marker arTag = tb1->getARtag();
  int tagID = temp_tag.first;
  geometry_msgs::Point tagPosition = temp_tag.second;
  // velocity variables
  geometry_msgs::Twist rotation;
  rotation.angular.z = 0;
  double angle = temp_angleToGoal; // heading angle to goal used to turn in the optimal direction of the goal
  
  // searches array to find the target tag and gets the index
  auto it = std::find(arTag.ids.data.begin(), arTag.ids.data.end(), tagID);

  if (it != arTag.ids.data.end()) {
    // The value was found, output the index
    int index = std::distance(arTag.ids.data.begin(), it);

    // Gets the associated yaw
    if (index < arTag.yaws.data.size()) {
      float yawError = arTag.yaws.data[index];
      
      if (fabs(yawError) > 0.05){ // within tolerance
      // simple proportional control
      float yawControl = 0.1 * yawError;

      rotation.angular.z = yawControl;
      } else{
        rotation.angular.z = 0;
        tb1->Send_cmd_tb1(rotation);
        std::cout << "Found and Confirmed AR Tag: " << tagID << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        return true;
      }
    } 
  } else {
    // The value was not found in the array        
    // rotates using simple proportional control in the tags direction until tag detected
    float yawControl = 0;
    if (abs(angle) > 0.1){
      yawControl = 0.1 * angle;
      rotation.angular.z = yawControl;
      
    } 
  }
  std::cout << "sending rotation: " << rotation.angular.z << std::endl;
  tb1->Send_cmd_tb1(rotation);

  return false;

}


// bool Method::tagAlignment(std::pair<int, geometry_msgs::Point> temp_tag, double temp_angleToGoal){
  










//High levl functions
/////////////////////////////////////////////////////////////////////////////////////////////////////

// bool Method::goalInObstacleCheck() { // doesnt work if there is only 1 goal and it is inside an object

//   geometry_msgs::Point currentGoal = Leader_goals.at(goal_index);
//   geometry_msgs::Point currentPos = tb1->GetCurrent_Odom().pose.pose.position;
//   geometry_msgs::Point referenceGoal;

//   if (goal_index != Leader_goals.size() - 1) {                              // for cases when there is a next goal
//     geometry_msgs::Point referenceGoal = Leader_goals.at(goal_index + 1);
//   } else if (Leader_goals.size() != 1) {                                    // for cases when it is the last goal and there is a previous goal
//     geometry_msgs::Point referenceGoal = Leader_goals.at(goal_index - 1);
//   }

//   // cosine rule
//   double a = hypot(referenceGoal.x - currentPos.x, referenceGoal.y - currentPos.y); // distance between next goal and bot
//   double b = hypot(currentGoal.x - currentPos.x, currentGoal.y - currentPos.y); // distance between current goal and bot
//   double c = hypot(referenceGoal.x - currentGoal.x, referenceGoal.y - currentGoal.y); // distance between goals

//   double A = (acos((b*b + c*c - a*a) / (2 * b * c))) * 180.0 / M_PI; // cosine rule for angle in degrees

//   if (A < 100 && A > 80) {   // checks if the bot is directly next to the current goal with reference to the next goal
//     return true;
//   }

//   return false;
// }




geometry_msgs::Point Method::findLookAheadPoint(const std::vector<geometry_msgs::Point>& path, const geometry_msgs::Point& current_position, double look_ahead_distance) {
    double cumulative_distance = 0.0;
    double closest_goal = 9999999;
    size_t current_gaol_id;
    geometry_msgs::Point look_ahead_point = path[0];

    // Find the closest point on the path to the current position (could be in front or behind the TB)
    for (size_t j = 0; j< path.size(); j++){
      
      double temp = calcDistance(path[j], current_position);
      if (temp < closest_goal){
        look_ahead_point = path[j];
        current_gaol_id = j;
        closest_goal = temp;
      }
    }

    // // Checks if the lookahead is inside object
    // if (goalInObstacleCheck()){
    //   current_gaol_id++;
    //   look_ahead_point = path[current_gaol_id];
    // }
  
  // Ensure that the current goal / closest point is the one in front of the TB
  double p1_p2 = calcDistance(path[current_gaol_id], path[current_gaol_id + 1]); // between current(p1) and next(p2) points
  double p1_p3 = calcDistance(path[current_gaol_id], current_position); // between current point(p1) and current pos(p3)
  double p2_p3 = calcDistance(path[current_gaol_id + 1], current_position); // between next point(p2) and current pos(p3)
  // Determine if goal is behind TB (which is bad)
    if (p1_p2 >= p2_p3 && p1_p2 >= p1_p3){ // current pos is middle point so current point is behind the TB, thus we must increment to the next point
      current_gaol_id++;
      look_ahead_point = path[current_gaol_id];
    }

  // Calculate cumulative distance along the path starting from the current position
  double distanceToNextPoint = calcDistance(current_position, path[current_gaol_id]);
  cumulative_distance += distanceToNextPoint; 
  for (size_t i = current_gaol_id; i < path.size() - 1; ++i) {
    double segment_length = calcDistance(path[i], path[i+1]);
    cumulative_distance += segment_length;
    // sets lookahead point
    if (cumulative_distance >= look_ahead_distance) {
      double overshoot = cumulative_distance - look_ahead_distance;
      double ratio = (segment_length - overshoot) / segment_length;
      look_ahead_point.x = path[i].x + ratio * (path[i+1].x - path[i].x);
      look_ahead_point.y = path[i].y + ratio * (path[i+1].y - path[i].y);
      return look_ahead_point;
    }
  }

  return path.back();

}



double Method::calcDistance(geometry_msgs::Point temp_point1, geometry_msgs::Point temp_point2){
  return sqrt(pow(temp_point2.x - temp_point1.x, 2) + pow(temp_point2.y - temp_point1.y, 2));
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


void Method::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  ROS_INFO("Received map with resolution %f meters/pixel", msg->info.resolution);
  std::unique_lock<std::mutex> lck3 (MapData_Lock);
  latestMapData_ = *msg;
}

void Method::mapMetadataCallback(const nav_msgs::MapMetaData::ConstPtr& msg) {
  ROS_INFO("Received map metadata. Width: %d, Height: %d", msg->width, msg->height);
  std::unique_lock<std::mutex> lck3 (MapMetaData_Lock);
  latestMapMetaData_ = *msg;  
}





visualization_msgs::MarkerArray Method::publishMarkers(const std::vector<geometry_msgs::Point>& nodes, ros::Publisher& marker_pub) {
  visualization_msgs::MarkerArray markerArray;
  id = 0; // Unique ID for each marker

  for (const auto& node : nodes) {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "odom"; // or your relevant frame
      marker.header.stamp = ros::Time::now();
      marker.ns = "nodes";
      marker.id = id++; // Assign and increment the unique ID
      marker.type = visualization_msgs::Marker::SPHERE; // Use SPHERE, CUBE, etc., as preferred
      marker.action = visualization_msgs::Marker::ADD;
      
      marker.pose.position.x = node.x;
      marker.pose.position.y = node.y;
      marker.pose.position.z = 0; // Assuming a flat map, set z to 0
      marker.pose.orientation.w = 1.0;

      marker.scale.x = 0.1; // Specify the size of the individual markers
      marker.scale.y = 0.1;
      marker.scale.z = 0.1; // Add z dimension for SPHERE, CUBE, etc.

      marker.color.r = 1.0; // Color: Red
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0; // Alpha (transparency)

      markerArray.markers.push_back(marker); // Add the marker to the array

  }
  
  marker_pub.publish(markerArray); // Publish the entire array]
  return markerArray;

}

void Method::publishCollisons(const std::vector<geometry_msgs::Point>& nodes, ros::Publisher& marker_pub, visualization_msgs::MarkerArray trajectory_makers) {
  
  for (const auto& node : nodes) {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "odom"; // or your relevant frame
      marker.header.stamp = ros::Time::now();
      marker.ns = "nodes";
      marker.id = id++; // Assign and increment the unique ID
      marker.type = visualization_msgs::Marker::SPHERE; // Use SPHERE, CUBE, etc., as preferred
      marker.action = visualization_msgs::Marker::ADD;
      
      marker.pose.position.x = node.x;
      marker.pose.position.y = node.y;
      marker.pose.position.z = 0; // Assuming a flat map, set z to 0
      marker.pose.orientation.w = 1.0;

      marker.scale.x = 0.3; // Specify the size of the individual markers
      marker.scale.y = 0.3;
      marker.scale.z = 0.3; // Add z dimension for SPHERE, CUBE, etc.

      marker.color.r = 0.0; // Color: Red
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      marker.color.a = 1.0; // Alpha (transparency)

      trajectory_makers.markers.push_back(marker); // Add the marker to the array
  
  }
  
  marker_pub.publish(trajectory_makers); // Publish the entire array]
 

}


void Method::processCollisions(const std::vector<std::pair<int, geometry_msgs::Point>>& robot_with_collisions, CollisionMap& collision_map) {
    std::map<int, std::vector<std::pair<double, double>>> robot_points;
    // Group collision points by robot IDs
    for (const auto& collision : robot_with_collisions) {
        int robot_id = collision.first;
        double x = collision.second.x;
        double y = collision.second.y;
        
        std::pair<double, double> collision_point = std::make_pair(x, y);
        robot_points[robot_id].push_back(collision_point);
    }

    // Check for collisions between different robots
    for (const auto& robot1 : robot_points) {
        for (const auto& point1 : robot1.second) {
            for (const auto& robot2 : robot_points) {
                if (robot1.first != robot2.first) {
                    for (const auto& point2 : robot2.second) {
                        if (point1 == point2) {
                            CollisionPair robot_pair = std::make_pair(std::min(robot1.first, robot2.first), std::max(robot1.first, robot2.first));
                            collision_map[point1].insert(robot_pair);
                        }
                    }
                }
            }
        }
    }
}


void Method::printCollisions(const CollisionMap& collision_map) {
     int total_collision_pairs = 0;

    for (const auto& entry : collision_map) {
        const auto& point = entry.first;
        const auto& robot_pairs = entry.second;
        
        std::cout << "Collision at point (" << point.first << ", " << point.second << "):" << std::endl;
        for (const auto& pair : robot_pairs) {
            std::cout << "  Turtlebots " << pair.first << " and " << pair.second << std::endl;
            total_collision_pairs++;
        }
    }

    std::cout << "Total unique collision points: " << collision_map.size() << std::endl;
    std::cout << "Total collision pairs: " << total_collision_pairs << std::endl;
}