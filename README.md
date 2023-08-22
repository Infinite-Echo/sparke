# SPARKE (Servo Powered Autonomous Robot with Kinematic Enhancements)
Welcome to the SPARKE project! This is a personal project I'm working on during my free time, aiming to create a mini version of the Boston Dynamics Spot robot. Although this started out as a ROS2 port of mike4192's spotMicro project (https://github.com/mike4192/spotMicro), much of the original code has been replaced or will be to better suit ROS2. The project is built on ROS2 and utilizes 3D models from the Spot Micro Open Source Project.

## Project Overview
The goal of this project is to replicate the functionality of the Boston Dynamics Spot robot on a smaller scale, leveraging the power of ROS2 and Gazebo simulation. As a robotics intern with a focus on computer engineering, I'm excited to explore and experiment with various concepts within this project.

## Current Progress
At the moment, this project is a work in progress. As a full time college undergraduate student, I find it hard to consistenly work on this project. There will often be large gaps of time between updates.

### Gazebo Simulation
A gazebo simulation is in development for sparke. The simulation was only functional with the galactic branch of https://github.com/rohitmenon86/ros2_controllers (note: the robot would still struggle to move forwards but motor controls worked). As of August 21 2023, the project has been migrated to humble and simulation will not be functional for the time being.

Fortunately, humble should have the necessary additions to the joint trajectory controller for the simulation to work without the need for external changes. Unfortunately, as a result of humble being native to ubuntu 22.04 and myself being on 20.04, I need to create a development docker so that I can run gazebo on 22.04. I hope to have this done within the next few days. 

### Custom Kinematics Solutions
In addition to using existing ROS2 control mechanisms, I've been developing custom kinematics solutions tailored to the specific dynamics of the mini Spot robot. This involves fine-tuning the motion control and ensuring smooth navigation within the simulation. (https://github.com/Infinite-Echo/sparkeKinematics)

## Planned Features
Looking ahead, I have exciting plans for expanding the capabilities of the mini Spot robot:

### Follow the Leader
One of the major upcoming features is the ability to make the robot follow a designated leader. The robot will be able to lock onto a person and autonomously navigate while avoiding obstacles. This feature will showcase the integration of perception, motion planning, and obstacle avoidance, making the robot a dynamic companion.

## How to Contribute
While this project is currently a personal endeavor, I'm open to collaboration and contributions. If you're passionate about robotics, ROS2, Gazebo simulation, or any related field, feel free to reach out. Let's discuss potential areas of collaboration and innovation.

Contact
If you're interested in this project or have any questions, you can reach out to me via email at InfiniteEchoRobotics@gmail.com. I'm looking forward to connecting with fellow enthusiasts and creators!

> Note: This README provides an overview of the project and its current status. For detailed technical information, code documentation, and updates, please refer to the project's source code and documentation.
