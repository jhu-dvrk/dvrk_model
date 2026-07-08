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
│   │   └── SUJ_cart.urdf.xacro
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

## Patient Cart Link Trees

These trees describe the expanded patient cart URDFs.  They use the format:

```text
parent_link --[joint_name (joint_type)]--> child_link
```

The SUJ patient cart uses this naming contract:

| Element | Rule | Example |
|---------|------|---------|
| Cart root link | `SUJ_cart` | `SUJ_cart` |
| SUJ arm link | `SUJ_{ARM}_L#` | `SUJ_PSM1_L2` |
| SUJ moving joint | `SUJ_{ARM}_J#` | `SUJ_PSM1_J2` |
| SUJ fixed numbered joint | `SUJ_{ARM}_fixed_J#` | `SUJ_PSM1_fixed_J4` |
| SUJ fixed RCM joint | `SUJ_{ARM}_fixed_RCM` | `SUJ_PSM1_fixed_RCM` |
| SUJ RCM link | `SUJ_{ARM}_RCM` | `SUJ_PSM1_RCM` |

The SUJ-to-arm attachment frames are `SUJ_PSM1_RCM`, `SUJ_PSM2_RCM`,
`SUJ_PSM3_RCM`, and `SUJ_ECM_RCM`. Standalone PSM/ECM arm URDFs should attach
their arm root to these frames when the patient cart is present.

### Si Patient Cart

```text
world --[SUJ_fixed_cart (fixed)]--> SUJ_cart
  SUJ_cart --[SUJ_ECM_J0 (prismatic)]--> SUJ_ECM_L0
    SUJ_ECM_L0 --[SUJ_ECM_J1 (revolute)]--> SUJ_ECM_L1
      SUJ_ECM_L1 --[SUJ_ECM_J2 (revolute)]--> SUJ_ECM_L2
        SUJ_ECM_L2 --[SUJ_ECM_J3 (revolute)]--> SUJ_ECM_L3
          SUJ_ECM_L3 --[SUJ_ECM_fixed_J4 (fixed)]--> SUJ_ECM_L4
            SUJ_ECM_L4 --[SUJ_ECM_fixed_RCM (fixed)]--> SUJ_ECM_RCM
              SUJ_ECM_RCM --[ECM_rcm_fixed (fixed)]--> ECM_RCM_link
                ECM_RCM_link --[ECM_base_fixed (fixed)]--> ECM_base_link
                  ECM_base_link --[yaw (revolute)]--> ECM_yaw_link
                    ECM_yaw_link --[pitch (revolute)]--> ECM_pitch_link
                      ECM_pitch_link --[pitch_2 (revolute)]--> ECM_pitch_2_link
                        ECM_pitch_2_link --[pitch_3 (revolute)]--> ECM_pitch_3_link
                          ECM_pitch_3_link --[insertion (prismatic)]--> ECM_insertion_link
                            ECM_insertion_link --[roll (revolute)]--> ECM_roll_link
                              ECM_roll_link --[endoscope_body (fixed)]--> ECM_endoscope_body_link
                                ECM_endoscope_body_link --[endoscope_frame (fixed)]--> ECM_endoscope_frame_link
                                  ECM_endoscope_frame_link --[endoscope_tip (fixed)]--> ECM_endoscope_tip_link

  SUJ_cart --[SUJ_PSM1_J0 (prismatic)]--> SUJ_PSM1_L0
    SUJ_PSM1_L0 --[SUJ_PSM1_J1 (revolute)]--> SUJ_PSM1_L1
      SUJ_PSM1_L1 --[SUJ_PSM1_J2 (revolute)]--> SUJ_PSM1_L2
        SUJ_PSM1_L2 --[SUJ_PSM1_J3 (revolute)]--> SUJ_PSM1_L3
          SUJ_PSM1_L3 --[SUJ_PSM1_fixed_J4 (fixed)]--> SUJ_PSM1_L4
            SUJ_PSM1_L4 --[SUJ_PSM1_fixed_RCM (fixed)]--> SUJ_PSM1_RCM
              SUJ_PSM1_RCM --[PSM1_rcm_fixed (fixed)]--> PSM1_RCM_link
                PSM1_RCM_link --[PSM1_base_fixed (fixed)]--> PSM1_base_link
                  PSM1_base_link --[yaw (revolute)]--> PSM1_yaw_link
                    PSM1_yaw_link --[pitch (revolute)]--> PSM1_pitch_link
                      PSM1_pitch_link --[pitch_2 (revolute)]--> PSM1_pitch_2_link
                        PSM1_pitch_2_link --[pitch_3 (revolute)]--> PSM1_pitch_3_link
                          PSM1_pitch_3_link --[insertion (prismatic)]--> PSM1_adaptor_link
                            PSM1_adaptor_link --[PSM1_instrument_housing_fixed (fixed)]--> PSM1_instrument_housing_link
                            PSM1_adaptor_link --[roll (revolute)]--> PSM1_roll_link
                              PSM1_roll_link --[roll_fixed (fixed)]--> PSM1_roll_housing_link
                                PSM1_roll_housing_link --[shaft_fixed (fixed)]--> PSM1_shaft_link
                              PSM1_roll_link --[wrist_pitch (revolute)]--> PSM1_wrist_pitch_0091m_link
                                PSM1_wrist_pitch_0091m_link --[wrist_yaw (revolute)]--> PSM1_wrist_yaw_0091m_link

  SUJ_cart --[SUJ_PSM2_J0 (prismatic)]--> SUJ_PSM2_L0
    SUJ_PSM2_L0 --[SUJ_PSM2_J1 (revolute)]--> SUJ_PSM2_L1
      SUJ_PSM2_L1 --[SUJ_PSM2_J2 (revolute)]--> SUJ_PSM2_L2
        SUJ_PSM2_L2 --[SUJ_PSM2_J3 (revolute)]--> SUJ_PSM2_L3
          SUJ_PSM2_L3 --[SUJ_PSM2_fixed_J4 (fixed)]--> SUJ_PSM2_L4
            SUJ_PSM2_L4 --[SUJ_PSM2_fixed_RCM (fixed)]--> SUJ_PSM2_RCM
              SUJ_PSM2_RCM --[PSM2_rcm_fixed (fixed)]--> PSM2_RCM_link
                PSM2_RCM_link --[PSM2_base_fixed (fixed)]--> PSM2_base_link
                  PSM2_base_link --[yaw (revolute)]--> PSM2_yaw_link
                    PSM2_yaw_link --[pitch (revolute)]--> PSM2_pitch_link
                      PSM2_pitch_link --[pitch_2 (revolute)]--> PSM2_pitch_2_link
                        PSM2_pitch_2_link --[pitch_3 (revolute)]--> PSM2_pitch_3_link
                          PSM2_pitch_3_link --[insertion (prismatic)]--> PSM2_adaptor_link
                            PSM2_adaptor_link --[PSM2_instrument_housing_fixed (fixed)]--> PSM2_instrument_housing_link
                            PSM2_adaptor_link --[roll (revolute)]--> PSM2_roll_link
                              PSM2_roll_link --[roll_fixed (fixed)]--> PSM2_roll_housing_link
                                PSM2_roll_housing_link --[shaft_fixed (fixed)]--> PSM2_shaft_link
                              PSM2_roll_link --[wrist_pitch (revolute)]--> PSM2_wrist_pitch_0091m_link
                                PSM2_wrist_pitch_0091m_link --[wrist_yaw (revolute)]--> PSM2_wrist_yaw_0091m_link

  SUJ_cart --[SUJ_PSM3_J0 (prismatic)]--> SUJ_PSM3_L0
    SUJ_PSM3_L0 --[SUJ_PSM3_J1 (revolute)]--> SUJ_PSM3_L1
      SUJ_PSM3_L1 --[SUJ_PSM3_J2 (revolute)]--> SUJ_PSM3_L2
        SUJ_PSM3_L2 --[SUJ_PSM3_J3 (revolute)]--> SUJ_PSM3_L3
          SUJ_PSM3_L3 --[SUJ_PSM3_J4 (revolute)]--> SUJ_PSM3_L4
            SUJ_PSM3_L4 --[SUJ_PSM3_fixed_J5 (fixed)]--> SUJ_PSM3_L5
              SUJ_PSM3_L5 --[SUJ_PSM3_fixed_RCM (fixed)]--> SUJ_PSM3_RCM
                SUJ_PSM3_RCM --[PSM3_rcm_fixed (fixed)]--> PSM3_RCM_link
                  PSM3_RCM_link --[PSM3_base_fixed (fixed)]--> PSM3_base_link
                    PSM3_base_link --[yaw (revolute)]--> PSM3_yaw_link
                      PSM3_yaw_link --[pitch (revolute)]--> PSM3_pitch_link
                        PSM3_pitch_link --[pitch_2 (revolute)]--> PSM3_pitch_2_link
                          PSM3_pitch_2_link --[pitch_3 (revolute)]--> PSM3_pitch_3_link
                            PSM3_pitch_3_link --[insertion (prismatic)]--> PSM3_adaptor_link
                              PSM3_adaptor_link --[PSM3_instrument_housing_fixed (fixed)]--> PSM3_instrument_housing_link
                              PSM3_adaptor_link --[roll (revolute)]--> PSM3_roll_link
                                PSM3_roll_link --[roll_fixed (fixed)]--> PSM3_roll_housing_link
                                  PSM3_roll_housing_link --[shaft_fixed (fixed)]--> PSM3_shaft_link
                                PSM3_roll_link --[wrist_pitch (revolute)]--> PSM3_wrist_pitch_0091m_link
                                  PSM3_wrist_pitch_0091m_link --[wrist_yaw (revolute)]--> PSM3_wrist_yaw_0091m_link
```

### Classic Patient Cart

```text
world --[SUJ_fixed_cart (fixed)]--> SUJ_cart
  SUJ_cart --[SUJ_ECM_J0 (prismatic)]--> SUJ_ECM_L0
    SUJ_ECM_L0 --[SUJ_ECM_J1 (continuous)]--> SUJ_ECM_L1
      SUJ_ECM_L1 --[SUJ_ECM_J2 (continuous)]--> SUJ_ECM_L2
        SUJ_ECM_L2 --[SUJ_ECM_J3 (continuous)]--> SUJ_ECM_L3
          SUJ_ECM_L3 --[SUJ_ECM_fixed_RCM (fixed)]--> SUJ_ECM_RCM

  SUJ_cart --[SUJ_PSM1_J0 (prismatic)]--> SUJ_PSM1_L0
    SUJ_PSM1_L0 --[SUJ_PSM1_J1 (continuous)]--> SUJ_PSM1_L1
      SUJ_PSM1_L1 --[SUJ_PSM1_J2 (continuous)]--> SUJ_PSM1_L2
        SUJ_PSM1_L2 --[SUJ_PSM1_J3 (continuous)]--> SUJ_PSM1_L3
          SUJ_PSM1_L3 --[SUJ_PSM1_J4 (continuous)]--> SUJ_PSM1_L4
            SUJ_PSM1_L4 --[SUJ_PSM1_fixed_RCM (fixed)]--> SUJ_PSM1_RCM

  SUJ_cart --[SUJ_PSM2_J0 (prismatic)]--> SUJ_PSM2_L0
    SUJ_PSM2_L0 --[SUJ_PSM2_J1 (continuous)]--> SUJ_PSM2_L1
      SUJ_PSM2_L1 --[SUJ_PSM2_J2 (continuous)]--> SUJ_PSM2_L2
        SUJ_PSM2_L2 --[SUJ_PSM2_J3 (continuous)]--> SUJ_PSM2_L3
          SUJ_PSM2_L3 --[SUJ_PSM2_J4 (continuous)]--> SUJ_PSM2_L4
            SUJ_PSM2_L4 --[SUJ_PSM2_fixed_RCM (fixed)]--> SUJ_PSM2_RCM

  SUJ_cart --[SUJ_PSM3_J0 (prismatic)]--> SUJ_PSM3_L0
    SUJ_PSM3_L0 --[SUJ_PSM3_J1 (continuous)]--> SUJ_PSM3_L1
      SUJ_PSM3_L1 --[SUJ_PSM3_J2 (continuous)]--> SUJ_PSM3_L2
        SUJ_PSM3_L2 --[SUJ_PSM3_J3 (continuous)]--> SUJ_PSM3_L3
          SUJ_PSM3_L3 --[SUJ_PSM3_J4 (continuous)]--> SUJ_PSM3_L4
            SUJ_PSM3_L4 --[SUJ_PSM3_fixed_RCM (fixed)]--> SUJ_PSM3_RCM
```

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
