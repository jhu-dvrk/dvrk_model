# dvrk_model — Target File Structure

This document describes the **target state** of the package after the naming and
architecture refactor. It is the authoritative reference for where files live and
why.

---

## Directory Tree

```
dvrk_model/
├── CMakeLists.txt
├── package.xml
├── README.md
├── structure.md                          ← this file
├── CHANGELOG.md
│
├── urdf/
│   ├── common.urdf.xacro                 ← shared material/color macros
│   │
│   ├── common/                           ← generation-agnostic macros
│   │   ├── PSM_instrument.urdf.xacro
│   │   └── psm_tools/
│   │       ├── PSM_housing.urdf.xacro
│   │       ├── PSM_roll.urdf.xacro
│   │       ├── wrist/
│   │       ├── tip/
│   │       └── PSM_0091m_wrist.urdf.xacro
│   │
│   ├── classic_arm/
│   │   ├── ECM.urdf.xacro
│   │   ├── ECM_base.urdf.xacro
│   │   ├── MTM.urdf.xacro
│   │   ├── MTML.urdf.xacro
│   │   ├── MTMR.urdf.xacro
│   │   ├── PSM1.urdf.xacro
│   │   ├── PSM2.urdf.xacro
│   │   ├── PSM3.urdf.xacro
│   │   ├── PSM1_snake.urdf.xacro
│   │   ├── PSM_base.urdf.xacro
│   │   ├── SUJ.urdf.xacro
│   │   └── archive/
│   │
│   ├── si_arm/
│   │   ├── ECM.urdf.xacro
│   │   ├── ECM_base.urdf.xacro
│   │   ├── PSM1.urdf.xacro
│   │   ├── PSM2.urdf.xacro
│   │   ├── PSM3.urdf.xacro
│   │   ├── PSM_420006_zero_check.urdf.xacro
│   │   ├── PSM_base.urdf.xacro
│   │   ├── SUJ.urdf.xacro
│   │   ├── SUJ_base.urdf.xacro
│   │   └── SUJ_column.urdf.xacro
│   │
│   └── Virtual/
│       ├── PSM1.urdf.xacro
│       ├── PSM2.urdf.xacro
│       ├── PSM3.urdf.xacro
│       └── PSM_base_virtual.urdf.xacro
│
├── meshes/
│   ├── dVRK-cube.stl
│   │
│   ├── arms/
│   │   ├── Classic/
│   │   │   ├── ECM/
│   │   │   ├── MTM/
│   │   │   └── PSM/
│   │   └── Si/
│   │       └── USM/
│   │
│   ├── instruments/
│   │   ├── Classic/
│   │   │   ├── housing/
│   │   │   ├── pitch/
│   │   │   ├── roll/
│   │   │   ├── tip/
│   │   │   └── yaw/
│   │   └── Si/
│   │       ├── housing/
│   │       ├── pitch/
│   │       ├── roll/
│   │       ├── tip/
│   │       └── yaw/
│   │
│   ├── SUJ/
│   │   ├── Classic/
│   │   └── Si/
│   │
│   ├── tower.stl
│   └── _review/
│       └── orphans/
│
├── launch/
├── ros2/
└── rviz/
```

---

## Naming Conventions

### DH Parameter Alignment

| DH Joint | Function | Mesh Name Pattern | Example |
|----------|----------|-------------------|---------|
| Joint 0  | Roll     | `PSM_roll*.{dae,stl}` | `PSM_roll.dae` |
| Joint 1  | Wrist pitch | `PSM_*_pitch.{dae,stl}` | `PSM_006_pitch.dae` |
| Joint 2  | Wrist yaw   | `PSM_*_yaw.{dae,stl}` | `PSM_006_yaw.dae` |
| Joint 3  | Jaw         | `PSM_*_jaw*.{dae,stl}` | `PSM_006_jaw.dae` |

---

### General Rules

| Element | Rule | Example |
|---------|------|---------|
| ARM acronym | UPPERCASE | PSM, MTM, ECM |
| Link name | `{ARM}_{desc}_link` | PSM_yaw_link |
| Joint name | functional | yaw, pitch, jaw |
| Mesh file | `{ARM}_{desc}.{stl,dae}` | PSM_yaw.stl |
| URDF entry | UPPERCASE arm | PSM1.urdf.xacro |
| Macros | lowercase | psm_base_macros |

---

## PSM/ECM Macro Architecture

```text
PSM_base (Classic/Si)
        │
        ▼
PSM_instrument_macros (common/)
        │
        ├── housing stage (common/psm_tools/PSM_housing.urdf.xacro)
        ├── roll/shaft stage (common/psm_tools/PSM_roll.urdf.xacro)
        ├── wrist stage (common/psm_tools/wrist)
        └── tip stage (common/psm_tools/tip)
```

---

## Virtual Variant Joint Limits

| Joint | Type | Range |
|------|------|------|
| yaw | revolute | -π/2 to π/2 |
| pitch | revolute | -π/4 to π/4 |
| insertion | prismatic | 0 to 0.24 m |

Virtual PSM entry files call `common/PSM_instrument.urdf.xacro` directly, so
tool-family dispatch and Classic/Si instrument compatibility remain identical
to the physical-arm paths.

---

## PSM Tip Families

| Tip | Notes |
|-----|------|
| 006 | standard needle driver |
| 049 | cadiere forceps |
| ECM | camera tool |

Only mesh-backed PSM tool families are active in the current URDF dispatch. Placeholder families without complete meshes are excluded from the supported set until assets and dimensions are verified.

Unsupported-family notes are kept in the docs only:
- 117: prior support was only a parsing stub; kinematics are unverified.
- 172: no dedicated 172 meshes are present; prior versions reused 006 geometry as a placeholder.
- 183: no verified cautery or no-jaw production mesh set is wired in the active dispatcher.

The implemented layout follows the physical instrument breakdown directly: shared body stage (housing plus roll and shaft), then wrist and tip stage files, with one small family file per supported tool code to call the common stages.

---

## Bug Fixes

| File | Issue | Fix |
|------|------|-----|
| ECM_base | tool_name missing zero | SF0826001 |
| MTM blend | typo | MTM_components.blend |
| PSM tool files | misplaced joints | moved to instrument macros |

---

## Legacy Files

- urdf/classic_arm/archive/psm_base.urdf.xacro (legacy snake base)
- urdf/classic_arm/archive/psm_tool_*.urdf.xacro (legacy snake/old tool files)
- snake_tool meshes (review pending)
- _review/orphans (unclassified assets)

---