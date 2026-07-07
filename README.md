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

Meshes are organized by subsystem and then by reusable instrument stage:

```
meshes/
├── arms/              # Robot arm bodies fixed to the cart
│   ├── Classic/
│   └── Si/
├── instruments/       # Replaceable surgical tools
│   ├── Classic/
│   │   ├── housing/
│   │   ├── pitch/
│   │   ├── roll/
│   │   ├── tip/
│   │   └── yaw/
│   └── Si/
│       ├── housing/
│       ├── pitch/
│       ├── roll/
│       ├── tip/
│       └── yaw/
├── SUJ/               # Setup joints and cart positioning hardware
│   ├── Classic/
│   └── Si/
└── Si/                # Si-specific fixed hardware such as the tower
```

---

## Supported PSM Tool Families

Only mesh-backed PSM tool families are wired into the current URDF:

| Family | Instrument codes | Mesh support | Verified dimensions |
|--------|------------------|--------------|---------------------|
| 006 | `400006`, `420006` | Classic and Si | wrist yaw offset `A = 0.0091 m`, `tool_tip y = 0.0102 m` |
| 049 | `400049` | Classic meshes only | wrist yaw offset `A = 0.0091 m` |

Shared roll dimensions remain generation-specific in [urdf/common/PSM_instrument.urdf.xacro](urdf/common/PSM_instrument.urdf.xacro):

- Classic roll origin `D = 0.4162 m`
- Si roll origin `D = 0.4670 m`

Families `172`, `117`, and `183` are intentionally not wired because this repository does not contain complete production meshes for them.
Current unsupported-family notes:
- `117`: prior support was only a parsing stub; wrist and jaw kinematics for the 5 mm snake tool are not validated here.
- `172`: no dedicated 172 meshes are present; prior versions reused 006 geometry as a placeholder.
- `183`: no verified cautery or no-jaw production mesh set is wired in the current dispatcher.

## Instrument Mesh Naming

Instrument meshes follow **DH parameter naming** for clarity:

| DH Joint | Function        | Mesh Name Pattern              | Example |
|----------|----------------|--------------------------------|---------|
| Joint 0  | Roll (shaft rotation) | `PSM_roll*.{dae,stl}`     | `PSM_roll.dae` |
| Joint 1  | Wrist pitch    | `PSM_*_pitch.{dae,stl}`    | `PSM_006_pitch.dae` |
| Joint 2  | Wrist yaw      | `PSM_*_yaw.{dae,stl}`      | `PSM_006_yaw.dae` |
| Joint 3  | Jaw open/close | `PSM_*_jaw*.{dae,stl}`     | `PSM_006_jaw.dae` |

For detailed mesh structure documentation, see [structure.md](structure.md).

## Adding a New Instrument Mesh Family

When adding a new tool family, keep common geometry shared and only add new files when the physical part is actually different.

1. Add the mesh files under the existing stage folders in `meshes/instruments/<Generation>/`:
  - `roll/` for a new rolling shaft or connector
  - `pitch/` and `yaw/` for the meshes that make up a wrist stage
  - `tip/` for jaw or end-effector meshes
2. Use the established naming pattern, for example `PSM_XXX_pitch`, `PSM_XXX_yaw`, `PSM_XXX_jaw_1`, `PSM_XXX_jaw_2`.
3. Reuse the existing roll and shaft assembly in [urdf/common/PSM_instrument.urdf.xacro](urdf/common/PSM_instrument.urdf.xacro) unless the shaft length or roll geometry truly changes.
4. Add a new family file in `urdf/common/psm_tools/` only when the tool needs a distinct wrist or tip combination.
5. Reuse the shared files in `urdf/common/psm_tools/`, `wrist/`, and `tip/` wherever the mechanical build matches an existing stage.
6. Add one explicit dispatch branch in [urdf/common/PSM_instrument.urdf.xacro](urdf/common/PSM_instrument.urdf.xacro) for the new instrument code.
7. Validate the result by expanding the xacro for at least one Classic or Si arm that uses the new code.

---

## Launch files

The top launch files for the dVRK are either for individual arms (MTML, MTMR, PSM1...), or for the patient cart or surgeon's console.

All launch files in the `dvrk_model` package are used for RViz visualization.

All launch files require a `generation` argument:

- `generation:=Classic`
- `generation:=Si`

The current ROS 2 launch mapping exposes `Classic` and `Si` generations.
Virtual PSM files are available under `urdf/Virtual/` for direct xacro use.

As of 2024, the MTMs and the surgeon's console can only be "Classic", so the `generation` argument is not used for those launch files.

There is also an optional `simulated` argument (default: `True`).

- `simulated:=True` → starts the `dvrk_robot dvrk_system` node with a simulated configuration
- `simulated:=False` → only visualization (assumes system node is running elsewhere)

There is also an optional `instrument` argument for PSM arms.

- `instrument:=420006` → explicit Si 006 selection
- `instrument:=400006` → explicit Classic 006 selection
- `instrument:=006` → shorthand; resolves to `420006` for `generation:=Si`, `400006` for `generation:=Classic`

If `instrument` is omitted, `arm.launch.py` selects by generation:

- `generation:=Si` defaults to `instrument:=420006`
- `generation:=Classic` defaults to `instrument:=400006`

For Si 420006, default mesh paths in URDF/Xacro use the current OBJ files
(`PSM_housing.OBJ`, `PSM_roll_8mm.OBJ`, `PSM_006_pitch.OBJ`, `PSM_006_yaw.OBJ`,
`PSM_006_jaw_1.OBJ`, `PSM_006_jaw_2.OBJ`) and not the `_archived.stl` files.

### Examples

```bash
ros2 launch dvrk_model arm.launch.py arm:=PSM1 generation:=Si instrument:=420006
ros2 launch dvrk_model arm.launch.py arm:=PSM1 generation:=Si instrument:=006
ros2 launch dvrk_model arm.launch.py arm:=PSM1 generation:=Classic instrument:=420006
ros2 launch dvrk_model arm.launch.py arm:=MTMR generation:=Classic simulated:=True
ros2 launch dvrk_model arm.launch.py arm:=ECM generation:=Classic
ros2 launch dvrk_model patient_cart.launch.py generation:=Si
ros2 launch dvrk_model patient_cart.launch.py generation:=Classic simulated:=True
ros2 launch dvrk_model surgeon_console.launch.py
# Mesh-frame debug for Si 420006
ros2 run xacro xacro $(ros2 pkg prefix dvrk_model)/share/dvrk_model/urdf/si_arm/PSM_420006_zero_check.urdf.xacro > /tmp/psm_420006_zero_check.urdf
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
│   ├── PSM_instrument.urdf.xacro
│   └── psm_tools/             # PSM instrument-tool subassemblies only
│       ├── PSM_housing.urdf.xacro
│       ├── PSM_roll.urdf.xacro
│       ├── wrist/             # Combined wrist stages by dimension/style
│       └── tip/               # Jaw and end-effector stages by family
├── classic_arm/               # Classic arm definitions
│   ├── ECM.urdf.xacro
│   ├── MTML.urdf.xacro
│   ├── MTMR.urdf.xacro
│   ├── PSM{1,2,3}.urdf.xacro
│   ├── PSM1_snake.urdf.xacro
│   ├── archive/               # Legacy snake/old tool macros kept for compatibility
│   ├── SUJ.urdf.xacro
│   └── *base*.urdf.xacro
├── si_arm/                    # Si arm definitions
│   ├── ECM.urdf.xacro
│   ├── PSM{1,2,3}.urdf.xacro
│   ├── SUJ.urdf.xacro
│   └── *base*.urdf.xacro
└── Virtual/                   # Virtual PSM entry points and base
    ├── PSM{1,2,3}.urdf.xacro
    └── PSM_base_virtual.urdf.xacro
```

---

## Key Files

### Entry points (standalone arms)

- `PSM{1,2,3}.urdf.xacro`: PSM arm with instrument (arg: `instrument`, default: 400006 Classic, 420006 Si)
- `Virtual/PSM{1,2,3}.urdf.xacro`: Virtual PSM entry points using shared `common/PSM_instrument.urdf.xacro`
- `MTM{L,R}.urdf.xacro`: Master Tool Manipulator (left/right)
- `ECM.urdf.xacro`: Endoscope Camera Manipulator

### Systems

- `patient_cart.urdf.xacro`: Full patient cart (SUJs, PSMs, ECM)
- `surgeon_console.urdf.xacro`: Surgeon console (MTML and MTMR)

### Macros

- `common.urdf.xacro`: Shared material/color definitions
- `common/PSM_instrument.urdf.xacro`: Shared instrument assembly and instrument-code dispatch
- `common/psm_tools/PSM_housing.urdf.xacro`: Instrument housing stage
- `common/psm_tools/PSM_roll.urdf.xacro`: Generation-specific roll and shaft stage
- `common/psm_tools/wrist/*`, `common/psm_tools/tip/*`: Stage files reused by supported instruments

### Hybrid PSM Layout

The current PSM instrument layout is split by responsibility:

- [urdf/common/PSM_instrument.urdf.xacro](urdf/common/PSM_instrument.urdf.xacro) is the single top-level assembly path for supported PSM instruments.
- `common/psm_tools/` contains the reusable physical subassemblies only.
- `common/psm_tools/PSM_housing.urdf.xacro`, `common/psm_tools/PSM_roll.urdf.xacro`, `wrist/`, and `tip/` follow the mechanical stages reused between instruments.

This keeps the active layout centered on common mechanical stages, while wrist files still reference the underlying `pitch/` and `yaw/` meshes they are built from.

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