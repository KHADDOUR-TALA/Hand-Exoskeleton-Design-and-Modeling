# Hand-Exoskeleton-Design-and-Modeling
This repository presents the design and modeling of a **rehabilitation hand exoskeleton** aimed at restoring finger movements in post-stroke patients. It includes comprehensive kinematic and dynamic analysis of both the human hand fingers and the proposed design of the exoskeleton. The CAD design was conducted using **SolidWorks** and then exported to **MATLAB**, and the simulation was conducted with  **Simscape**. The limitation of the human movements taken into consideration and the alignment with human joints to ensure safety in human-robot interaction are represented in the equation as tolerance. The local and global manipulability was also analyzed. 


## Key Contributions

**Mechanical Design**: Develop a lightweight,, self-alignment hand exoskeleton compatible with natural hand anatomy with integrated safty and rehabilitaion ascpect.

**Mathematical Modeling**: Develope accurate kinematic and dynamic models for both human hand and exoskeleton systems. 

**Performance Optimization**: Analyze kinematic and dynamic manipulability to ensure best force transmission and safe movements within the boundary of human hand range of motion.

## Abbreviations used in this README 
**DIP** Distal Inter-Phalangeal  
**HES** Hand Exo-Skeleton  
**MCP** Meta-Carpo-Phalangeal  
**P** Prismatic joint 
**PIP** Proximal Inter-Phalangeal 
**R** Revolute joint 
**ROM** Range of Motion 

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
├── src/
│   ├── arduino.py             # Arduino motor controller emulator
│   ├── broad_lidar.py         # Simulated LiDAR publisher
│   ├── broad_robot.py         # Main navigation controller
│   ├── client.py              # Service client for pose reset
│   └── lidar_data.py          # Maze visualization publisher
├── srv/
│   └── reset.srv              # Custom service for pose reset
└── CMakeLists.txt & package.xml
```

##  System Architecture
1. ### Human Hand Modeling
*** Anatomical Structure:**** 5 digits with varying degrees of freedom (DOF)

4 fingers (index, middle, ring, little): 4 phalanges, 3 articulations each

Thumb: 3 phalanges, 4 articulations

***Joint Constraints:** Natural Range of Motion (ROM) limits for each joint
Natural ROM boundaries ensuring patient safety


| Joint     | Flexion | Extension | Abduction/Adduction |
|-----------|---------|-----------|---------------------|
| Index MCP |   90°   |  30°-40°  |          60°        |
| Index PIP |   110   |    0°     |          0°         |
| Thumb MCP | 75°-80° |    0°     |          5°         |



**Kinematic Chains:** Denavit-Hartenberg (DH) parameterization for forward/inverse kinematics

**Dynamic Modeling:** Euler-Lagrange formulation including stiffness and damping factors

2. ### Exoskeleton Design
-**MCP Joint Mechanism:** **(P3RP2R)** closed-loop chain for flexion/extension and abduction/adduction

-**PIP Joint Mechanism:** Four-bar linkage **(4R)** kinematic chain for flexion/extension

**Design Features:**

* Under-actuated configuration

* Cable-driven transmission

* Misalignment compensation for human-robot axes

* Normal force application on fingers
**MCP Joint Mechanism:** (P3RP2R) closed-loop chain for flexion/extension and abduction/adduction

**PIP Joint Mechanism:** Four-bar linkage (4R) kinematic chain for flexion/extension

This design of the exoskeleton include only theindex fingure and can be augmented to actuarte the other fingures and the to  the MCP and PIP joint actua :
```bash
cd ~/catkin_ws/src
git clone https://github.com/<your_username>/differential_robot.git
cd ..
catkin_make
source devel/setup.bash
```

## Launch the Simulation

1. Start robot, LiDAR, navigation, markers, and RViz:
```bash
roslaunch differential_robot rviz.launch
```


## Nodes Overview

### **1. simulated_lidar (`broad_lidar.py`)**

Simulates 360° LiDAR using ray-casting against maze walls.

**Publishes**

* `/scan` (sensor_msgs/LaserScan)
* TF: `base_link → hokuyo_link`

---

### **2. maze_navigator (`broad_robot.py`)**

Autonomous robot controller: obstacle avoidance + wall following + odometry.

**Publishes**

* `/cmd_vel`
* `/robot_path` (nav_msgs/Path)
* TF: `world → base_link`

**Subscribes**

* `/scan`

**Service**

* `/reset_pose` (reset.srv)


### **3. maze_marker_publisher (`lidar_data.py`)**

Publishes maze wall lines using RViz markers.

**Publishes**

* `/visualization_marker`


### **4. arduino_emulator (`arduino.py`)**

Receives velocity commands and prints them (debug motor interface).

**Subscribes**

* `/cmd_vel`


### **5. reset_pose_client (`client.py`)**

Calls reset service:

```bash
rosrun differential_robot client.py
```


##  URDF Robot

Robot defined in:

* `robot/robot.xacro`
* `robot/materials.xacro`

Includes:

* Base link
* Chassis
* Left & right wheels
* Caster wheel
* LiDAR sensor
* Materials (blue, white, black)


##  Reset Pose Service

### Service definition

```
float32 x
float32 y
float32 yaw

```

### Call manually
You can call this service manulay by using this:
```bash
rosservice call /reset_pose 0 0 0
```


## RViz Visualization

Load the provided configuration:

```bash
rviz -d $(rospack find differential_robot)/rviz/robot.rviz
```

Displays:

* URDF model
* TF tree
* LiDAR scan
* Maze walls
* Robot path


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

## Author

This package was developed by Tala Khaddour



