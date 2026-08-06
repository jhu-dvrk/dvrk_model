# dvrk_model — Current File Structure

This document describes the current package layout after the naming and
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
├── LICENSE
├── TESTING.md
├── mesh_rename_map.md
│
├── urdf/
│   ├── common.urdf.xacro                 ← shared material/color macros
│   │
│   ├── common/                           ← generation-agnostic macros
│   │   ├── instrument.urdf.xacro
│   │   ├── endoscope.urdf.xacro
│   │   ├── RCM_visual.urdf.xacro
│   │   ├── endoscopes/
│   │   │   ├── Classic_SD_straight.urdf.xacro
│   │   │   └── Si_straight.urdf.xacro
│   │   └── instruments/
│   │       ├── instruments.yaml
│   │       ├── housing/
│   │       │   ├── housing_Classic.urdf.xacro
│   │       │   └── housing_Si.urdf.xacro
│   │       ├── roll/
│   │       │   ├── roll_4162.urdf.xacro
│   │       │   └── roll_4670.urdf.xacro
│   │       ├── wrist_pitch/
│   │       │   ├── wrist_pitch_0091.urdf.xacro
│   │       │   ├── wrist_pitch_0107.urdf.xacro
│   │       │   └── wrist_pitch_0107_80.urdf.xacro
│   │       ├── wrist_yaw/
│   │       │   ├── wrist_yaw_0091.urdf.xacro
│   │       │   ├── wrist_yaw_0107.urdf.xacro
│   │       │   └── wrist_yaw_0107_90.urdf.xacro
│   │       └── tip/
│   │           ├── tip_006.urdf.xacro
│   │           ├── tip_049.urdf.xacro
│   │           ├── tip_093.urdf.xacro
│   │           ├── tip_179.urdf.xacro
│   │           ├── tip_309.urdf.xacro
│   │           └── tip_placeholder.urdf.xacro
│   │
│   ├── Classic/
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
│   ├── Si/
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
│   │   ├── housing/
│   │   │   ├── Classic/
│   │   │   │   └── housing_Classic.dae
│   │   │   └── Si/
│   │   │       ├── housing_Si.obj
│   │   │       ├── housing_Si.mtl
│   │   │       ├── housing_Si_archived.stl
│   │   │       └── housing_Si_ecm.stl
│   │   ├── roll/
│   │   │   ├── 4162/
│   │   │   │   ├── roll_4162.dae
│   │   │   │   └── roll_4162_yaw.dae
│   │   │   └── 4670/
│   │   │       ├── roll_4670.obj
│   │   │       ├── roll_4670.mtl
│   │   │       ├── roll_4670_connector.stl
│   │   │       ├── roll_4670_archived.stl
│   │   │       └── roll_4670_ecm.stl
│   │   ├── wrist_pitch/
│   │   │   └── 0091/
│   │   │       ├── wrist_pitch_0091.obj
│   │   │       ├── wrist_pitch_0091.mtl
│   │   │       ├── wrist_pitch_0091_archived.stl
│   │   │       ├── wrist_pitch_0091.dae
│   │   │       └── wrist_pitch_0091.stl
│   │   ├── wrist_yaw/
│   │   │   └── 0091/
│   │   │       ├── wrist_yaw_0091.obj
│   │   │       ├── wrist_yaw_0091.mtl
│   │   │       ├── wrist_yaw_0091_archived.stl
│   │   │       ├── wrist_yaw_0091.dae
│   │   │       └── wrist_yaw_0091.stl
│   │   └── tip/
│   │       ├── 006/
│   │       │   ├── tip_006_1.obj
│   │       │   ├── tip_006_1.mtl
│   │       │   ├── tip_006_1_archived.stl
│   │       │   ├── tip_006_2.obj
│   │       │   ├── tip_006_2.mtl
│   │       │   ├── tip_006_2_archived.stl
│   │       │   └── Classic/
│   │       │       └── tip_006.dae
│   │       ├── 049/
│   │       │   └── tip_049.stl
│   │       ├── 093/
│   │       │   ├── tip_093_1.obj
│   │       │   ├── tip_093_1.mtl
│   │       │   ├── tip_093_2.obj
│   │       │   └── tip_093_2.mtl
│   │       ├── 179/
│   │           ├── tip_179_1.obj
│   │           ├── tip_179_1.mtl
│   │           ├── tip_179_2.obj
│   │           └── tip_179_2.mtl
│   │       └── 309/
│   │           ├── tip_309_1.obj
│   │           ├── tip_309_1.mtl
│   │           ├── tip_309_2.obj
│   │           └── tip_309_2.mtl
│   │
│   ├── endoscopes/
│   │   ├── Classic_SD_straight/
│   │   └── Si_straight/
│   │
│   ├── SUJ/
│   │   ├── Classic/
│   │   └── Si/
│   │       ├── ECM/
│   │       └── PSM/
│   │
│   └── tower.stl
│
├── launch/
├── ros2/
│   ├── launch/
│   ├── ros2_control/
│   └── rviz/
└── rviz/
    ├── Classic/
    ├── Si/
    └── dvrk_si.rviz
```

---

## Naming Conventions

### DH Parameter Alignment

| DH Joint | Function | Mesh Name Pattern | Example |
|----------|----------|-------------------|---------|
| Joint 0  | Roll     | `roll_<D>.{dae,obj,stl}` | `roll_4670.obj` |
| Joint 1  | Wrist pitch | `wrist_pitch_<A>.{dae,obj,stl}` | `wrist_pitch_0091.obj` |
| Joint 2  | Wrist yaw   | `wrist_yaw_<A>.{dae,obj,stl}` | `wrist_yaw_0091.obj` |
| Joint 3  | Jaw         | `tip_<code>*.{dae,obj,stl}` | `tip_006_1.obj` |

---

### General Rules

| Element | Rule | Example |
|---------|------|---------|
| ARM acronym | UPPERCASE | PSM, MTM, ECM |
| Link name | `{ARM}_{desc}_link` | PSM_yaw_link |
| Joint name | functional | yaw, pitch, jaw |
| Mesh file | `<part>_<id>.{stl,dae,obj}` for shared instrument parts | wrist_yaw_0091.obj |
| URDF entry | UPPERCASE arm | PSM1.urdf.xacro |
| Macros | lowercase | instrument |

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
                            PSM1_adaptor_link --[PSM1_housing_fixed (fixed)]--> PSM1_housing_link
                            PSM1_adaptor_link --[roll (revolute)]--> PSM1_roll_link
                              PSM1_roll_link --[wrist_pitch (revolute)]--> PSM1_wrist_pitch_link
                                PSM1_wrist_pitch_link --[wrist_yaw (revolute)]--> PSM1_wrist_yaw_link

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
                            PSM2_adaptor_link --[PSM2_housing_fixed (fixed)]--> PSM2_housing_link
                            PSM2_adaptor_link --[roll (revolute)]--> PSM2_roll_link
                              PSM2_roll_link --[wrist_pitch (revolute)]--> PSM2_wrist_pitch_link
                                PSM2_wrist_pitch_link --[wrist_yaw (revolute)]--> PSM2_wrist_yaw_link

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
                              PSM3_adaptor_link --[PSM3_housing_fixed (fixed)]--> PSM3_housing_link
                              PSM3_adaptor_link --[roll (revolute)]--> PSM3_roll_link
                                PSM3_roll_link --[wrist_pitch (revolute)]--> PSM3_wrist_pitch_link
                                  PSM3_wrist_pitch_link --[wrist_yaw (revolute)]--> PSM3_wrist_yaw_link
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
instrument (common/instrument.urdf.xacro)
        │
        ├── housing stage (common/instruments/housing)
        ├── roll/shaft stage (common/instruments/roll)
        ├── wrist_pitch stage (common/instruments/wrist_pitch)
        ├── wrist_yaw stage (common/instruments/wrist_yaw)
        └── tip stage (common/instruments/tip)

ECM_base (Classic/Si)
        │
        ▼
endoscope dispatcher (common/endoscope.urdf.xacro)
        │
        └── endoscope variants (common/endoscopes)
            ├── Classic_SD_straight.urdf.xacro
            └── Si_straight.urdf.xacro
```

The endoscope dispatcher includes exactly one variant based on the `endoscope`
xacro argument. Each variant owns the ECM endoscope visual/collision geometry
and camera/tip reference frames.

---

## Virtual Variant Joint Limits

| Joint | Type | Range |
|------|------|------|
| yaw | revolute | -π/2 to π/2 |
| pitch | revolute | -π/4 to π/4 |
| insertion | prismatic | 0 to 0.24 m |

Virtual PSM entry files call `common/instrument.urdf.xacro` directly, so
tool-family dispatch and Classic/Si instrument compatibility remain identical
to the physical-arm paths.

---

## PSM Tip Families

| Tip | Notes |
|-----|------|
| 006 | standard needle driver |
| 049 | cadiere forceps |
| 093 | prograsp forceps |
| 179 | monopolar curved scissors |
| 309 | mega suturecut needle driver |
| placeholder | 5 mm box visuals for supported tools without tip meshes |
| ECM | camera tool |

Many tool families are active in the current URDF dispatch using JSON-derived roll and wrist yaw dimensions. Tips without production meshes use 5 mm box placeholders until assets are added.

Unsupported-family notes are kept in the docs only:
- 117 and 143: JSON files do not use the standard wrist_pitch/wrist_yaw joints.
- 183 and 184: wrist yaw offset A = 0.0093 m.
- 194: wrist yaw offset A = 0.0112 m.
- 410298: stapler-specific roll and wrist geometry.

The implemented layout follows the physical instrument breakdown directly: shared body stage (housing plus roll and shaft), then wrist and tip stage files. Tool families with missing tip meshes share `tip_placeholder.urdf.xacro`.

---

## Bug Fixes

| File | Issue | Fix |
|------|------|-----|
| ECM_base | tool_name missing zero | SF0826001 |
| MTM blend | typo | MTM_components.blend |
| PSM tool files | misplaced joints | moved to instrument macros |

---

## Legacy Files

- urdf/Classic/archive/psm_base.urdf.xacro (legacy snake base)
- urdf/Classic/archive/psm_tool_*.urdf.xacro (legacy snake/old tool files)
- snake_tool meshes (review pending)

---
