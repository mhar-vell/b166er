# B166ER Robot Package

Complete robot package for the B166ER mobile manipulator system.

## Structure

- `launch/`: Organized launch files for hardware, simulation, and control
- `config/`: Robot configuration and parameter files
- `urdf/`: Robot description files (URDF/Xacro)
- `scripts/`: Executable Python scripts
- `src/`: Python package source code
- `rviz/`: RViz configuration files
- `meshes/`: 3D mesh files for visualization
- `worlds/`: Gazebo world files
- `docs/`: Documentation

## Quick Start

```bash
# Complete system (simulation)
roslaunch b166er_robot b166er_complete.launch

# Hardware only
roslaunch b166er_robot b166er_complete.launch use_hardware:=true use_simulation:=false

# Simulation without arm
roslaunch b166er_robot b166er_complete.launch enable_arm:=false
```