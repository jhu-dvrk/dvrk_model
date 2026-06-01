# ROS models for the dVRK

This repository is usually cloned along the rest of the dVRK repositories using the `vcs` command. See build instructions for the dVRK:  
https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki ➡️ **Software installation**

---

## Content

This repository contains:

- **Meshes** for the dVRK, both Classic and Si
  - Robot arm bodies (PSM, ECM, MTM, USM)
  - Replaceable surgical instruments (tips, rolls, covers)
  - Setup joints (SUJ) for cart positioning

- **URDF files** for the different arms (ECM, PSM, MTM) as well as setup joints (SUJ)

- **Launch files** and RViz configuration files

The directories for meshes and URDF are the same for ROS 1 and ROS 2.

The top-level directories `launch` and `rviz` are for ROS 1. For ROS 2, the equivalent directories are under `ros2/launch` and `ros2/rviz`. When using ROS 2, the files are installed in the shared directory for this package and can be used with:

```bash
ros2 launch dvrk_model ...
```

---

## Mesh Organization

As of 2026, meshes are organized by **function** rather than generation:

```
meshes/
├── arms/              # Robot arm bodies (fixed to cart)
│   ├── Classic/       # Classic PSM, ECM, MTM arm mechanisms
│   └── Si/            # Si USM arm mechanism
├── instruments/       # Replaceable surgical tools
│   ├── rolls/         # Shared roll assemblies (Classic 5mm, Si 8mm)
│   ├── covers/        # Sterile adapter covers
│   └── tips/          # Tool-specific wrist/jaw assemblies
│       ├── Classic/400006/  # Large Needle Driver
│       ├── Classic/400049/  # Cadiere Forceps
│       └── Si/420006/       # Large Needle Driver Si
├── SUJ/               # Setup joints (cart positioning hardware)
│   ├── Classic/
│   └── Si/
└── Si/                # Si-specific components (tower)
```

---

## Instrument Mesh Naming

Instrument meshes follow **DH parameter naming** for clarity:

| DH Joint | Function        | Mesh Name Pattern              | Example |
|----------|----------------|--------------------------------|---------|
| Joint 0  | Roll (shaft rotation) | `PSM_roll*.{dae,stl}`     | `PSM_roll.dae` |
| Joint 1  | Wrist pitch    | `PSM_tip_*_pitch.{dae,stl}`    | `PSM_tip_006_pitch.dae` |
| Joint 2  | Wrist yaw      | `PSM_tip_*_yaw.{dae,stl}`      | `PSM_tip_006_yaw.dae` |
| Joint 3  | Jaw open/close | `PSM_tip_*_jaw*.{dae,stl}`     | `PSM_tip_006_jaw.dae` |

For detailed mesh structure documentation, see [`structure.md`](structure.md).

---

## Launch files

The top launch files for the dVRK are either for individual arms (MTML, MTMR, PSM1...), or for the patient cart or surgeon's console.

All launch files in the `dvrk_model` package are used for RViz visualization.

All launch files require a `generation` argument:

- `generation:=Classic`
- `generation:=Si`

As of 2024, the MTMs and the surgeon's console can only be "Classic", so the `generation` argument is not used for those launch files.

There is also an optional `simulated` argument (default: `True`).

- `simulated:=True` → starts the `dvrk_robot dvrk_system` node with a simulated configuration
- `simulated:=False` → only visualization (assumes system node is running elsewhere)

### Examples

```bash
ros2 launch dvrk_model arm.launch.py arm:=PSM1 generation:=Si
ros2 launch dvrk_model arm.launch.py arm:=MTMR generation:=Classic simulated:=True
ros2 launch dvrk_model arm.launch.py arm:=ECM generation:=Classic
ros2 launch dvrk_model patient_cart.launch.py generation:=Si
ros2 launch dvrk_model patient_cart.launch.py generation:=Classic simulated:=True
ros2 launch dvrk_model surgeon_console.launch.py
```

---

## URDF files

Files using all-capitalized names are full arms (e.g. ECM, MTMR, MTML, PSM1, PSM2, PSM3).  
Files using all lowercase names are parts or full systems (e.g. patient_cart, surgeon_console).

`xacro` stands for XML macro (parameterized XML code). Xacro files can be compiled using:

```bash
ros2 run xacro xacro mtm.urdf.xacro > result.urdf
```

---

## URDF Structure

```
urdf/
├── common/                    # Generation-agnostic instrument macros
│   ├── PSM_instrument_macros.urdf.xacro
│   └── PSM_tips_macros.urdf.xacro
├── Classic/                   # Classic arm definitions
│   ├── ECM.urdf.xacro
│   ├── MTML.urdf.xacro
│   ├── MTMR.urdf.xacro
│   ├── PSM{1,2,3}.urdf.xacro
│   ├── SUJ.urdf.xacro
│   └── *_macros.urdf.xacro
└── Si/                        # Si arm definitions
    ├── ECM.urdf.xacro
    ├── PSM{1,2,3}.urdf.xacro
    ├── SUJ.urdf.xacro
    └── *_macros.urdf.xacro
```

---

## Key Files

### Entry points (standalone arms)

- `PSM{1,2,3}.urdf.xacro`: PSM arm with instrument (arg: `instrument`, default: 400006 Classic, 420006 Si)
- `MTM{L,R}.urdf.xacro`: Master Tool Manipulator (left/right)
- `ECM.urdf.xacro`: Endoscope Camera Manipulator

### Systems

- `patient_cart.urdf.xacro`: Full patient cart (SUJs, PSMs, ECM)
- `surgeon_console.urdf.xacro`: Surgeon console (MTML and MTMR)

### Macros

- `common.urdf.xacro`: Shared material/color definitions
- `common/PSM_instrument_macros.urdf.xacro`: Instrument assembly (roll + wrist + tip)
- `common/PSM_tips_macros.urdf.xacro`: All PSM tip variants (006, 049, etc.)

For detailed URDF architecture, see `structure.md`.

---

## CAD files

Most meshes are in STL format. Some original CAD files can be found in:

https://github.com/jhu-dvrk/dvrk_cad

CAD files are stored separately due to their large size.

---

## Recent Changes

### 2026 Mesh Reorganization

- Separated arms (fixed hardware) from instruments (replaceable tools)
- Adopted DH parameter naming for instrument joints
- Flattened directory structure for maintainability

### 2024

- Added Si patient cart and ECM support
- Added surgeon console URDF

See `CHANGELOG.md` for complete release history.