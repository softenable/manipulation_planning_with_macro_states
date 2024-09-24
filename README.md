# SoftEnable Deliverable 2.3: Software for Manipulation Planning with Macro States
This deliverable is based on the publication "Dynamic Manipulation of Deformable Objects using Imitation Learning with Adaptation to Hardware Constraints"
The page contains code from the following project: https://sites.google.com/view/bilbo-bag.

Below is a brief description of the contents of each folder:

**[Data](Data/)** contais a subfolder with the human demonstration used for dynamically manipulating a deformable bag which is used along the provided code.

**[scripts](scripts/)** contains MATLAB scripts for processing human demonstrations so that they can be used in the constrained DMPs and for generating plots.

**[scripts/OptDMP](scripts/OptDMP)** contains code adapted from https://github.com/Slifer64/novel-DMP-constraints, which is adapted to implement a third constrained DMP version.

**[skymul_natnet_ros_cpp](skymul_natnet_ros_cpp/)**: we use the following ROS driver used to record the markers on the bag to compute metrics of the manipulation performance https://github.com/SkyMul/skymul_natnet_ros_cpp/tree/main.

**[franka_david](franka_david/)** contains code for running the robots. The subfolder "scripts" contains the logic of the experiments, with the exception of "franka.py". This file and the remaining folders contain code related to implementing the controllers and motion-generators for the robots, and it builds upon code used in previous projects of the research group. As the robot control code is not the key focus of this project we do not prioritize its readability. 

# Usage
The main script for generating the joint and cartesian trajectories using the constrained DMP is [run_OptDMP.m](scripts/run_OptDMP.m).

For using this script it is necessary to provide a demonstration CSV file which encodes the demonstration trajectory. We provide the file [10l_bag_flip.csv](Data/demos/10l_bag_flip.csv) as a reference of the format style.
In the script we adapt the width using the generate demo, which gets the demonstration trajectory from the CSV file.
The demonstration trajectory has the X, Y, Z and quaternion of both left and right hand, as well as the time starting from zero of each datapoint.
When importing the demonstration trajectory we use the inverse kinematics of the robot to compute the joints' position and jacobian matrices. Check the files [createFranka.m](scripts/utils/createFranka.m), [ForwardKinematics.m](scripts/utils/ForwardKinematics.m) and [InverseKinematics.m](scripts/utils/InverseKinematics.m) and modify as necessary if you plan to use a different robot.

Before using the script [run_OptDMP.m](scripts/run_OptDMP.m) you **must run the script [install_osqp.m](scripts/install_osqp.m)** to install the [OSQP library](https://github.com/osqp/osqp-matlab).

The [run_OptDMP.m](scripts/run_OptDMP.m) file performs the following:
1. Import the demonstration trajectory. When selecting the demonstration trajectory we decide which bag size we want to manipulate.

2. Set the number of kernels, the standard deviation, the robot DOF and the training method (where we use Least Squares).

3. Define a GMP using these values and train it with the demonstration.

4. Define the same initial state, goal state and horizon as in the demonstration.

5. Define the robot hardware position, velocity and acceleration limits (hard-coded in the script).

6. Solve the optimisation problem of the constrained DMP taking into account the hardware constraints using Opt-DMP and the OSQP solver.

7. Get the joint positions and plot the constrained and unconstrained positions.
The DMP trajectories will be saved in **Data/trajectories**. There you can find the cartesian pose, joint positions, velocities and accelerations of the Opt-DMP as well as the joint position and velocities of the unconstrained DMP.
The plots will be saved in **Data/plots**.


## Dependencies
The dependencies of the project and the associated licenses are the following:
1. Matlab
2. OSQP Matlab - Apache-2.0 license
1. skymul_natnet_ros_cpp - GPL-3.0 license
2. novel-DMP-constraints - MIT license
4. ROS1
5. Libfranka - Apache-2.0 license (only required for running the software with the Franka Emika Panda robot)

# Copyright
Copyright 2024 (c) SoftEnable - All Rights Reserved

Author: David Blanco-Mulero <dblancom@iri.upc.edu>
