# Hand-Exoskeleton Design, Modeling and Optimization
This repository presents the design and modeling of a **rehabilitation hand exoskeleton** aimed at restoring finger movements in post-stroke patients. It includes comprehensive kinematic and dynamic analysis of both the human hand fingers and the proposed design of the exoskeleton. The CAD design was conducted using **SolidWorks** and then exported to **MATLAB**, and the simulation was conducted with  **Simscape**.The repositpry also combines **manipulability** analysis and optomization using **Genetic Algorithms** to achive safety in human-robot interaction.

<p align="center">
  <img src="Images/simulation.gif" width="700">
</p>

## Key Contributions

**Mechanical Design**: Develop a lightweight,, self-alignment hand exoskeleton compatible with natural hand anatomy with integrated safty and rehabilitaion ascpect.

**Mathematical Modeling**: Develope accurate kinematic and dynamic models for both human hand and exoskeleton systems. 

**Performance Optimization**: Analyze kinematic and dynamic manipulability to ensure best force transmission and safe movements within the boundary of human hand range of motion.

## Abbreviations 

**DIP**_Distal Inter-Phalangeal  

**HES**_Hand Exo-Skeleton  

**MCP**_Meta-Carpo-Phalangeal

**P**_Prismatic joint 

**PIP**_Proximal Inter-Phalangeal 

**R**_Revolute joint 

**ROM**_Range of Motion 


##  System Architecture
 ### 1. Human Hand Modeling
*** Anatomical Structure:**** 5 digits with varying degrees of freedom (DOF)

4 fingers (index, middle, ring, little): 4 phalanges, 3 articulations each

Thumb: 3 phalanges, 4 articulations

**Joint Constraints:** Natural Range of Motion (ROM) limits for each joint
Natural ROM boundaries ensuring patient safety


| Joint     | Flexion | Extension | Abduction/Adduction |
|-----------|---------|-----------|---------------------|
| Index MCP |   90°   |  30°-40°  |          60°        |
| Index PIP |   110   |    0°     |          0°         |
| Thumb MCP | 75°-80° |    0°     |          5°         |



**Kinematic Chains:** Denavit-Hartenberg (DH) parameterization for forward/inverse kinematics

**Dynamic Modeling:** Euler-Lagrange formulation including stiffness and damping factors
### 2. Exoskeleton Design
-**MCP Joint Mechanism:** **(P3RP2R)** closed-loop chain for flexion/extension and abduction/adduction

-**PIP Joint Mechanism:** Four-bar linkage **(4R)** kinematic chain for flexion/extension

**Design Features:**

* Under-actuated Cable-driven transmission

* Misalignment compensation for human-robot axes
* 
* Normal force application on fingers

* 
## Modeling and Analysis

### Kinematic and Dynamics
**DH Parameters:** Complete tables for human fingers and exoskeleton joints

**Transformation Matrices:** Homogeneous transformations for position/orientation

**Inverse Kinematics:** Geometric methods with physiological angle constraints

**Loop Closure Equations:** For MCP and PIP closed-chain mechanisms

## Dynamic Analysis
**Energy Formulations:** Kinetic, potential (gravitational + elastic), and dissipation

**Motion Equations:** Euler-Lagrange derivation with joint torques

**Virtual Work Principle:** Force/torque transmission analysis

**Manipulability Measures:** Local and global kinematic/dynamic performance metrics

## Manipulability Evaluation
Manipulability measures how well the exoskeleton can move and apply force to the finger.

High manipulability → smoother motion , lower motor torque and safer rehabilitation. 
The optimizatipon of link ligth is done using ** Genetic algorithms**guided by manipulability analysis. 

##  Topics Summary

| Topic                        | Type          | Published By          |
| ---------------------------- | ------------- | --------------------- |
| `/scan`                      | LaserScan     | simulated_lidar       |
| `/cmd_vel`                   | Twist         | maze_navigator        |
| `/robot_path`                | nav_msgs/Path | maze_navigator        |
| `/visualization_marker`      | Marker        | maze_marker_publisher |
| TF `world → base_link`       | TF            | maze_navigator        |
| TF `base_link → hokuyo_link` | TF            | simulated_lidar       |


## Navigation Behavior and Control algorithm
The Control algorithm used to escape the maze can be described by these rules:
* Stops & turns if obstacle < **0.5 m** in front
* Right-hand **wall following** (0.3 m distance)
* Free-space turning preference
* Odometry simulated from `/cmd_vel`
* Path recorded continuously
## Demo

*Autonomous navigation with LiDAR scanning, obstacle avoidance, and path planning in simulated maze environment.*

![Robot Navigation Demo](differential_robot/media/Demo.gif)

## Project Structure

```
differential_robot/
├── media/
│   └── Demo.gif          
├── launch/
│   └── rviz.launcher          # Main launch file for visualization
├── robot/
│   ├── materials.xacro        # material definitions for robot components
│   └── robot.xacro            # Complete robot URDF model
├── rviz/
│   └── robot.rviz             # RViz configuration preset
├──sr c/
│   ├── arduino.py             # Arduino motor controller emulator
│   ├── broad_lidar.py         # Simulated LiDAR publisher
│   ├── broad_robot.py         # Main navigation controller
│   ├── client.py              # Service client for pose reset
│   └── lidar_data.py          # Maze visualization publisher
├── srv/
│   └── reset.srv              # Custom service for pose reset
└── Images
```


