# ROS models for the dVRK

This repository is usually cloned along the rest of the dVRK repositories using the `vcs` command.  See build instructions for the dVRK: https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki ➡️ **Software installation**.

## Content

This repository contains:
* meshes for the dVRK, both Classic and Si
* URDF files for the different arms (ECM, PSM, MTM) as well setup joints (SUJ)
* launch files and RViz configuration files

The directories for meshes and URDF are the same for ROS 1 and ROS 2.

The top level directories `launch` and `rviz` are for ROS 1.  For ROS 2, the directories are under `ros2/launch` and `ros2/rviz` are the equivalent for ROS 2.  When using ROS 2, the files are installed in the shared directory for this package and can be used with `ros2 launch dvrk_model ...`.

## CAD files

Most meshes are in STL format.  Some of the original CAD files can be found in the repository https://github.com/jhu-dvrk/dvrk_cad.  The CAD files are stored in a different repository as they tend to be fairly large.
