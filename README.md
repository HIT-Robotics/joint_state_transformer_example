# joint_state_transformer_example
<p align="center">
  <img src="docs/logos/autonox_Robotics_Logo.png" height="60" style="margin-right: 40px;">
  <img src="docs/logos/hhn_logo.png" height="80">
</p>


This repository demonstrates the usage of the **constraint_to_gazebo** and the **joint_state_transformer** for constrained robot kinematics. It closes the loops created with **constraint** elements of the URDF.  Each robot is provided as its own package. The robot-specific URDF and meshes are provided in the **descriptions** package.

The *constraint_to_gazebo* closes the loops in the description and helps to create the gazebo specific elements for simulation.
The *joint_state_transformer* closes the loops in the ROS Environment during runtime to maintin the closed state of the robot.

The following robots are supported at the moment:

- Autonox *A_00036* 4-DoF Delta robot with a telescopic shaft

Additional robots will be added in the future.

The repository includes the following packages:

- **urdfdom_headers** fork: adds the constraints to the URDF
- **urdfdom** fork: parses the extended URDF with the constraints
- **pinocchio**: used for kinematic tree calculations. Requires recompilation due to the custom **urdfdom** fork
- **robot_model**: solves constraint equations for closed-loop structures
- **joint_state_transformer** package: uses the **robot_model** to transform the trajectory and calculate the passive angles
- **descriptions**: provides the URDF files and meshes for the robots
- **constraint_to_gazebo**: scripts to close the loops of an URDF using the **robot_model** package and create the gazebo specific elements
- **bringup**: Package to close the loops and start gazebo, and the ros packages for the robot using a choosen robot specific MoveIt-Package
- robot-specific MoveIt packages in the demo folder: provide a simple demonstration with RViz using the demo.launch.py

## Prerequisites

- Docker
- A running X server as a display

## Usage

1. Initialize the submodules

   ```bash
   git submodule update --init --recursive
   ```

2. Create the 'buffer' directory with the 'build', 'install' and 'log' subfolders, or remove these volume mounts from the compose file.
   The compose file references a buffer folder that holds the built packages for container rebuilds, to save compiling time after adaptations on the container. 

3. Start the container

   ```bash
   docker compose up
   ```  

4. Inside the container: Build (since we need to rebuild pinocchio to link the urdfdom forks, this may take a while)

   ```bash
   colcon build
   source install/setup.bash
   ```

5. Inside the container: Start the simulation using the bringup package

   ```bash
   ros2 launch bringup sim.launch.py moveit_config_package:=<moveit_config_package_name> urdf_file:=<urdf_file_in_config_directory> joint_state_transformer_input_action:=<moveit_output_action> joint_state_transformer_output_action:=<ros2_control_input_action>
   ```

   Example for the included demo robot (the URDF/Xacro is resolved inside the `config` directory of the given MoveIt config package):

   ```bash
   ros2 launch bringup sim.launch.py moveit_config_package:=a_00036 urdf_file:=DELTA_4DoF_γ.urdf.xacro
   ```
   

## Roadmap

- Add more examples (e.g. the Para-Engineer robots)
- Test the process with real hardware
- Use a kinematics plugin to solve kinematics in MoveIt and enable a collision detection (Depending on the pull request to URDFDOM)
- Add a robot_state_publisher fork to update the joint state directly without the joint_state_transformer (Depending on the pull request to URDFDOM)

  
## Acknowledgement

The repository is developed and maintained by Heilbronn University of Applied Sciences

Supported by Autonox robotics GmbH
