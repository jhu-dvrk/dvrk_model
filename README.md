# ROS models for the dVRK

This repository is usually cloned along the rest of the dVRK repositories using the `vcs` command.  See build instructions for the dVRK: https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki ➡️ **Software installation**.

## Content

This repository contains:
* meshes for the dVRK, both Classic and Si
* URDF files for the different arms (ECM, PSM, MTM) as well setup joints (SUJ)
* launch files and RViz configuration files

The directories for meshes and URDF are the same for ROS 1 and ROS 2.

The top level directories `launch` and `rviz` are for ROS 1.  For ROS
2, the directories are under `ros2/launch` and `ros2/rviz` are the
equivalent for ROS 2.  When using ROS 2, the files are installed in
the shared directory for this package and can be used with `ros2
launch dvrk_model ...`.

## Launch files

The top launch files for the dVRK are either for individual arms
(MTML, MTMR, PSM1...) or, either the patient cart or the surgeon's
console.  All launch files in the `dvrk_model` package are used for
RViz visualization.

All launch files require a `generation` argument
(`generation:=Classic` or `generation:=Si`).  As of 2024, the MTMs and
the surgeon's console can only be "Classic" so the the `generation`
argument is not used for the MTML, MTMR and surgeon's console launch
files.

There is also an optional `simulated` argument (default is `True`).
When this argument is set the launch files starts the node `dvrk_robot
dvrk_system` with a configuration file to simulate the dVRK.
When `simulated:=False`, the dVRK system node is not started.  This can be
used to visualize the physical robot assuming the user has already
started the dVRK system node in a separate terminal.

Examples:
```bash
roslaunch dvrk_model arm.launch arm:=PSM1 generation:=Si
roslaunch dvrk_model arm.launch arm:=MTMR generation:=Classic simulated:=False
roslaunch dvrk_model arm.launch arm:=ECM generation:=Classic
roslaunch dvrk_model patient_cart.launch generation:=Si
roslaunch dvrk_model patient_cart.launch generation:=Classic simulated:=False
roslaunch dvrk_model surgeon_console.launch
```

## URDF files

Files using all capitalized names are for full arms, e.g. `ECM`,
`MTMR`, `MTML`, `PSM1`, `PSM2`, `PSM3`.  Files using all lower case
names are for parts or full system (e.g. `patient_cart`,
`surgeon_console`).

`xacro` stands for XML macro, i.e. parameterized XML code.  `xacro` files can be "compiled" using:
```bash
rosrun xacro xacro mtm.urdf.xacro .... > result.urdf
```

Parts:
* `common.urdf.xacro`: general material info
* `mtm.urdf.xacro`: definition of xacros for da Vinci MTMs

Arms:
* `PSM{1,2,3}.urdf.xacro`: definition of PSM1, PSM2 and PSM3 using xacros from psm*.urdf.xacro
* `MTM{L,R}.urdf.xacro`: definition of MTML and MTMR using xacros from mtm.urdf.xacro

Systems:
* `patient_cart.urdf.xacro`: definition of the full patient cart (SUJs, PSMs and ECM)
* `surgeon_console.urdf.xacro`: definition of the surgeon's console (MTML and MTMR)

## CAD files

Most meshes are in STL format.  Some of the original CAD files can be
found in the repository https://github.com/jhu-dvrk/dvrk_cad.  The CAD
files are stored in a different repository as they tend to be fairly
large.
