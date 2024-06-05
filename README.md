# Turtlebot_Warehouse
UTS ROBOTICS 2 multi turtlebot warehouse program


# TurtleBot Warehouse Simulation
## Introduction
This project showcases a fully-functional simulation and real-life integration of autonomous warehouse operations using TurtleBot robots. We achieved significant milestones by not only simulating the functional aspects of the project but also by successfully localizing and integrating two robots, with preliminary functionality extending to three robots. The project leverages ROS (Robot Operating System) and demonstrates complex multi-robot coordination and navigation within a dynamic warehouse environment.

## Running the Project
Prerequisites
This project is developed using ROS Melodic but is compatible with any ROS 1 distributions, assuming all dependencies are met. Ensure that your ROS workspace is set up with OpenCV and all necessary packages installed.

## Simulation
To run the project in a simulated environment, use one of the following commands based on your setup:

For a single TurtleBot simulation:
```roslaunch turtlebot_fake turtlebot_fake.launch```
For simulating multiple TurtleBots:
```roslaunch turtlebot_fake multi_turtlebot_fake.launch```
For an advanced simulation with three TurtleBots:
```roslaunch turtlebot_fake 3_hail_mary_multiple_turtlebot.launch```

## Real-life Integration
For real-life deployment:

Single TurtleBot localized using the default TurtleBot navigation:
``rosrun single_turtlebot_integration``
For two or three TurtleBots, use the following command to handle localization:
```roslaunch multi_cartographer tb3_localisation```

## Subsystem Testing and Development
Each subsystem within the project is contained in its respective folder, allowing for modular testing and development. This structure supports individual component analysis, debugging, and enhancement without impacting other parts of the project. The testing procedures for each subsystem are documented within their folders, providing clear guidelines on how to ensure functionality and performance are up to standards.

Additional Information
For further details on the project setup, dependencies, and advanced configurations, please refer to the specific README files within each directory.
