# Robotics-Project
Autonomous Toy Collection with POMDP Planning
This project implements an autonomous robotic system designed to collect valuable toys using Partially Observable Markov Decision Process (POMDP) planning. The robot must navigate an environment with uncertain toy locations, collect toys efficiently, and maximize reward collection within a limited number of actions.
Key Features  

Autonomous decision-making using Autonomous Operating System (AOS)
Probabilistic toy placement and reward generation
ROS and Gazebo simulation environment
POMDP-based planning strategy

Problem Description
The robot faces a complex task:

Collect toys from 4 different locations
Each toy has a different color and reward value
Initial toy locations are probabilistically determined
Limited number of pick actions available
Goal: Maximize total reward by strategically collecting toys


Planning Method

Uses Partially Observable Markov Decision Process (POMDP)
Applies Monte Carlo Tree Search for decision-making
Handles uncertainty in toy locations and rewards

Simulation Components

ROS (Robot Operating System)
Gazebo simulation environment
Custom environment configuration
Probabilistic state initialization


Developed as part of an Robotics (AOS) mini-project (Prof. Ronen Brafman)
