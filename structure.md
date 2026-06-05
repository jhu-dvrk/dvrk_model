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
│   │   └── PSM_tips.urdf.xacro
│   │
│   ├── Classic/
│   │   ├── ECM.urdf.xacro
│   │   ├── MTML.urdf.xacro
│   │   ├── MTMR.urdf.xacro
│   │   ├── PSM1.urdf.xacro
│   │   ├── PSM2.urdf.xacro
│   │   ├── PSM3.urdf.xacro
│   │   ├── PSM1_snake.urdf.xacro
│   │   ├── SUJ.urdf.xacro
│   │   ├── ECM_base_macros.urdf.xacro
│   │   ├── MTM_macros.urdf.xacro
│   │   ├── PSM_base_macros.urdf.xacro
│   │   └── psm_tool_*.urdf.xacro
│   │
│   ├── Si/
│   │   ├── ECM.urdf.xacro
│   │   ├── PSM1.urdf.xacro
│   │   ├── PSM2.urdf.xacro
│   │   ├── PSM3.urdf.xacro
│   │   ├── SUJ.urdf.xacro
│   │   ├── patient_cart.urdf.xacro
│   │   ├── ECM_base_macros.urdf.xacro
│   │   ├── PSM_base_macros.urdf.xacro
│   │   ├── SUJ_base_macros.urdf.xacro
│   │   └── SUJ_column_macros.urdf.xacro
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
│   │   ├── rolls/
│   │   ├── covers/
│   │   └── tips/
│   │
│   ├── SUJ/
│   │   ├── Classic/
│   │   └── Si/
│   │
│   ├── Si/
│   │   └── tower.stl
│   │
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
PSM_base_macros (Classic/Si)
        │
        ▼
PSM_instrument_macros (common/)
        │
        ├── roll (shaft)
        ├── wrist_pitch
        ├── wrist_yaw
        └── jaw
```

---

## Virtual Variant Joint Limits

| Joint | Type | Range |
|------|------|------|
| yaw | revolute | -π/2 to π/2 |
| pitch | revolute | -π/4 to π/4 |
| insertion | prismatic | 0 to 0.24 m |

---

## PSM Tip Families

| Tip | Notes |
|-----|------|
| 006 | standard needle driver |
| 049 | cadiere forceps |
| 117 | 5mm snake tool |
| 183 | no jaw variants |
| ECM | camera tool |

---

## Bug Fixes

| File | Issue | Fix |
|------|------|-----|
| ECM_base | tool_name missing zero | SF0826001 |
| MTM blend | typo | MTM_components.blend |
| PSM tool files | misplaced joints | moved to instrument macros |

---

## Legacy Files

- psm_base.urdf.xacro (deprecated)
- psm_tool_*.urdf.xacro (legacy tools)
- snake_tool meshes (review pending)
- _review/orphans (unclassified assets)

---