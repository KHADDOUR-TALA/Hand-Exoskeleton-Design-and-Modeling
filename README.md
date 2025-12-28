# Hand-Exoskeleton Design, Modeling and Optimization
This repository presents the design and modeling of a **rehabilitation hand exoskeleton** aimed at restoring finger movements in post-stroke patients. It includes comprehensive kinematic and dynamic analysis of both the human hand fingers and the proposed design of the exoskeleton. The CAD design was conducted using **SolidWorks** and then exported to **MATLAB**, and the simulation was conducted with  **Simscape**.The repositpry also combines **manipulability** analysis and optomization using **Genetic Algorithms** to achive safety in human-robot interaction.

<p align="center">
  <img src="Images/Simulation.gif" width="500">
</p>

## Key Contributions

**Mechanical Design**: Develop a lightweight,, self-alignment hand exoskeleton compatible with natural hand anatomy with integrated safty and rehabilitaion ascpect.

**Mathematical Modeling**: Develope accurate kinematic and dynamic models for both human hand and exoskeleton systems. 

**Performance Optimization**: Analyze kinematic and dynamic manipulability to ensure best force transmission and safe movements within the boundary of human hand range of motion.

## Abbreviations 

**DIP**_Distal Inter-Phalangeal  **HES**_Hand Exo-Skeleton **MCP**_Meta-Carpo-Phalangeal  **P**_Prismatic joint  **PIP**_Proximal Inter-Phalangeal   **R**_Revolute joint  **ROM**_Range of Motion 

##  System Architecture
 ### 1. Human Hand Modeling
  - 5 digits, anatomically accurate DOF

  - Full ROM constraints, intra-finger & inter-finger coupling

  - DH-based kinematics and Euler-Lagrange dynamics

  - Stiffness & damping modeling for impaired joints

<p align="center">

  <img src="Images/Kinematic chain of the index and thumb fingers.jpg" width="400">
  
</p>

### 2. Exoskeleton Design

| Joint                       | Mechanism         | Function          |
| ---------------------------- | ------------- | --------------------- |
| **MCP**                     | P3RP2R closed chain    |	Flex/Ext + Abd/Add       |
| **PIP**                   | 4R closed chain        | closed chain	Flex/Ext        |
| **DIP**                | 	Biomechanically coupled | Passive        |


**MCP exoskeleton kinematic chain**

<p align="center">

 <img src="Images/P3RP2R chain for MCP hand exoskeleton with human hand.png" width="380"  style="margin-right: 20px;">  
</p>





## Modeling and Analysis

### Kinematic and Dynamics
- Developed accurate **kinematic and dynamic models** of human fingers and the exoskeleton.

- **Forward and inverse kinematics** derived using standard DH parameters, ensuring alignment with physiological joint limits.

- **Dynamic analysis** includes kinetic, gravitational, and elastic energies, with Euler-Lagrange equations for joint torques.

- **Manipulability** analysis performed to optimize smooth motion, reduce motor torque, and ensure safe rehabilitation.

- Full CAD models of the **human hand and exoskeleton** created and exported for simulation in Simscape.


**The developed CAD model of the human hand** 
<p align="center">

  <img src="Images/Human Hand CAD1.PNG" width="200" height="200" style="margin-right: 20px;">
  <img src="Images/Human Hand CAD2.PNG" width="200" height="200">
</p>

**The CAD model of the hand exoskeleton showing the design for the MCP and PIP closed-loop chains**
<p align="center">

  <img src="Images/CAD Hand EXoskeleton.png" width="400">
</p>



## Manipulability Evaluation
Manipulability measures how well the exoskeleton can move and apply force to the finger.

High manipulability **→** smoother motion , lower motor torque and safer rehabilitation. 
The optimizatipon of link ligth is done using **Genetic algorithms**guided by manipulability analysis. 


**Optimization Results**

| MCP Chain(mm)                       | PIP Chain(mm)          |
| ---------------------------- | ------------- |
| L1=11.33                     | L7=30    |
| L2=19.61                   | L8=30        | 
| L3=41.83               |       | 
| L4=17.4               | 	 | 
