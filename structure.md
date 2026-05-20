# dvrk_model — Target File Structure

This document describes the **target state** of the package after the naming and
architecture refactor.  It is the authoritative reference for where files live and
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
│   ├── common.urdf.xacro                 ← shared material/color macros (unchanged)
│   │
│   ├── common/                           ← NEW: generation-agnostic macros
│   │   ├── PSM_instrument.urdf.xacro     ← roll + shaft + tip dispatch (all generations)
│   │   └── PSM_tips.urdf.xacro           ← all PSM tip family macros
│   │meshes
│   ├── Classic/
│   │   │  Entry points (UPPERCASE arm name):
│   │   ├── ECM.urdf.xacro                ← standalone ECM (Classic)
│   │   ├── MTML.urdf.xacro               ← standalone MTM left
│   │   ├── MTMR.urdf.xacro               ← standalone MTM right
│   │   ├── PSM1.urdf.xacro               ← PSM1, arg: tool_model
│   │   ├── PSM2.urdf.xacro               ← PSM2, arg: tool_model
│   │   ├── PSM3.urdf.xacro               ← PSM3, arg: tool_model
│   │   ├── PSM1_snake.urdf.xacro         ← PSM1 with snake tool (5 mm)
│   │   ├── SUJ.urdf.xacro                ← full Classic patient cart
│   │   │
│   │   │  Macro includes (lowercase):
│   │   ├── ecm_base.urdf.xacro           ← ECM arm macro (3 DOF + insertion + roll)
│   │   ├── mtm.urdf.xacro                ← MTM arm macro
│   │   └── PSM_base.urdf.xacro           ← PSM arm macro: 3 DOF only (yaw+pitch+insertion)
│   │                                        ends at PSM_tool_parent_link
│   │
│   ├── Si/
│   │   │  Entry points:
│   │   ├── ECM.urdf.xacro
│   │   ├── PSM1.urdf.xacro               ← arg: tool_model (default 420006)
│   │   ├── PSM2.urdf.xacro
│   │   ├── PSM3.urdf.xacro
│   │   ├── SUJ.urdf.xacro                ← full Si patient cart
│   │   ├── patient_cart.urdf.xacro       ← full Si patient cart with all arms
│   │   │
│   │   │  Macro includes:
│   │   ├── ecm.urdf.xacro                ← Si ECM arm macro
│   │   ├── PSM_base.urdf.xacro           ← Si PSM arm macro: 3 DOF only
│   │   │                                    ends at PSM_tool_parent_link
│   │   ├── suj.urdf.xacro                ← Si SUJ arm macro
│   │   └── suj_column.urdf.xacro         ← Si column/tower macro
│   │
│   └── Virtual/                          ← NEW: simulator-only (no arm geometry)
│       ├── PSM1.urdf.xacro               ← args: tool_model, shaft_length,
│       ├── PSM2.urdf.xacro               │        rcm_xyz, rcm_rpy
│       ├── PSM3.urdf.xacro               ←
│       └── PSM_base_virtual.urdf.xacro   ← 3 virtual DOF at RCM point
│                                            small inertia, proper joint limits
│
├── meshes/
│   ├── dVRK-cube.stl                     ← calibration cube (unchanged)
│   │
│   ├── Classic/
│   │   ├── ECM/
│   │   │   ├── ECM_base.stl
│   │   │   ├── ECM_outer_yaw.stl
│   │   │   ├── ECM_pitch_front.stl
│   │   │   ├── ECM_pitch_bottom.stl
│   │   │   ├── ECM_pitch_end.stl
│   │   │   ├── ECM_pitch_top.stl
│   │   │   ├── ECM_pitch_back.stl
│   │   │   ├── ECM_main_insertion.stl
│   │   │   ├── ECM_tool.stl
│   │   │   └── ECM_RCM.stl
│   │   │
│   │   ├── MTM/
│   │   │   ├── MTM_top_panel.stl/.dae
│   │   │   ├── MTM_outer_yaw.stl/.dae          (was OutPitch_Shoulder)
│   │   │   ├── MTM_shoulder_pitch.stl/.dae     (was ArmParallel)
│   │   │   ├── MTM_shoulder_pitch_parallel.stl/.dae  (was ArmParallel1)
│   │   │   ├── MTM_elbow_pitch.stl/.dae        (was BottomArm)
│   │   │   ├── MTM_arm.stl/.dae                (was Link — clarify usage)
│   │   │   ├── MTM_wrist_platform.stl/.dae     (was WristPlatform)
│   │   │   ├── MTM_wrist_pitch.stl/.dae        (was WristPitch)
│   │   │   ├── MTM_wrist_yaw.stl/.dae          (was WristYaw)
│   │   │   ├── MTM_wrist_roll.stl/.dae         (was WristRoll)
│   │   │   └── MTM_components.blend            (was mtm_omponents.blend — typo fix)
│   │   │
│   │   ├── PSM/
│   │   │   │  Arm meshes (used by Classic/PSM_base.urdf.xacro):
│   │   │   ├── PSM_base.dae
│   │   │   ├── PSM_yaw.dae               (was yaw.dae)
│   │   │   ├── PSM_pitch_back.dae        (was pitch_back.dae)
│   │   │   ├── PSM_pitch_front.dae       (was pitch_front.dae)
│   │   │   ├── PSM_pitch_bottom.dae      (was pitch_bottom.dae)
│   │   │   ├── PSM_pitch_top.dae         (was pitch_top.dae)
│   │   │   ├── PSM_insertion.dae         (was insertion.dae)
│   │   │   │
│   │   │   │  Standard instrument meshes (8 mm shaft, shared tip geometry):
│   │   │   ├── PSM_shaft.dae                   (was tool_main.dae)
│   │   │   ├── PSM_wrist.dae                   (was tool_wrist_link.dae)
│   │   │   ├── PSM_wrist_shaft.dae             (was tool_wrist_shaft_link.dae)
│   │   │   │
│   │   │   │  Standard wrist tip - 006 (most instruments, A=0.0091):
│   │   │   ├── PSM_tip_006_wrist.dae           (was tool_wrist_sca_link.dae)
│   │   │   ├── PSM_tip_006_shaft.dae           (was tool_wrist_sca_shaft_link.dae)
│   │   │   ├── PSM_tip_006_jaw.dae             (was tool_wrist_sca_link_2.dae)
│   │   │   │
│   │   │   │  Cadiere tip - 049 (different jaw, same wrist as 006):
│   │   │   ├── PSM_tip_049_wrist.stl           (was tool_wrist_caudier_link_1.stl)
│   │   │   ├── PSM_tip_049_shaft.stl           (was tool_wrist_caudier_link_1_shaft.stl)
│   │   │   ├── PSM_tip_049_jaw.stl             (was tool_wrist_caudier_link_2.stl)
│   │   │   │
│   │   │   │  Snake tool (5 mm shaft, extra coupled wrist joints):
│   │   │   └── snake_tool/                     (contents unchanged pending review)
│   │   │
│   │   └── SUJ/
│   │       ├── SUJ_base.stl                    (was base_link.stl)
│   │       ├── SUJ_ECM_link_0.stl              (was suj_ecm_L0.stl)
│   │       ├── SUJ_ECM_link_1.stl
│   │       ├── SUJ_ECM_link_2.stl
│   │       ├── SUJ_ECM_link_3.stl
│   │       ├── SUJ_PSM1_link_0.stl             (was suj_psm1_L0.stl)
│   │       ├── SUJ_PSM1_link_1.stl
│   │       ├── SUJ_PSM1_link_2.stl
│   │       ├── SUJ_PSM1_link_3.stl
│   │       ├── SUJ_PSM1_link_4.stl
│   │       ├── SUJ_PSM2_link_0.stl             (was suj_psm2_L0.stl)
│   │       ├── SUJ_PSM2_link_1.stl ... link_4.stl
│   │       ├── SUJ_PSM3_link_0.stl             (was suj_psm3_L0.stl)
│   │       └── SUJ_PSM3_link_1.stl ... link_4.stl
│   │
│   ├── Si/
│   │   ├── tower.stl                           (was tower.STL — lowercase extension)
│   │   │
│   │   ├── PSM_ECM/                            (shared arm body for Si PSM and ECM)
│   │   │   ├── PSM_ECM_base.stl                (was link_0.STL)
│   │   │   ├── PSM_ECM_yaw.stl           (was link_1.STL)
│   │   │   ├── PSM_ECM_pitch.stl         (was link_2.STL)
│   │   │   ├── PSM_ECM_pitch_bottom.stl  (was link_3.STL)
│   │   │   └── PSM_ECM_pitch_top.stl     (was link_4.STL)
│   │   │
│   │   └── SUJ/
│   │       ├── ECM/
│   │       │   ├── SUJ_ECM_link_0.stl          (was link_0.STL)
│   │       │   ├── SUJ_ECM_link_1.stl
│   │       │   ├── SUJ_ECM_link_2.stl
│   │       │   └── SUJ_ECM_link_3.stl
│   │       └── PSM/
│   │           ├── PSM12/                      (was 12/ — PSM1 and PSM2 variant)
│   │           │   ├── SUJ_PSM_link_0.stl      (was link_0.STL)
│   │           │   ├── SUJ_PSM_link_1.stl
│   │           │   ├── SUJ_PSM_link_2.stl
│   │           │   └── SUJ_PSM_link_3.stl
│   │           └── PSM3/                       (was 3/)
│   │               ├── SUJ_PSM_link_0.stl
│   │               ├── SUJ_PSM_link_1.stl
│   │               ├── SUJ_PSM_link_2.stl
│   │               ├── SUJ_PSM_link_3.stl
│   │               └── SUJ_PSM_link_4.stl
│   │
│   └── instruments/                            ← per-product-number instrument meshes
│       │   Note: Classic tip meshes live in Classic/PSM/ (shared with Classic arm pkg).
│       │   This folder holds Si-specific geometry and instrument-unique meshes.
│       │
│       ├── 420006/                             ← Large Needle Driver (S/Si, 8 mm, wrist-006)
│       │   ├── PSM_shaft.stl                   (was tool_main_link.STL)
│       │   ├── PSM_wrist.stl                   (was tool_wrist_link.STL)
│       │   ├── PSM_wrist_shaft.stl             (was tool_wrist_shaft_link.STL)
│       │   ├── PSM_tip_006_wrist.stl           (was tool_wrist_sca_link.STL)
│       │   ├── PSM_tip_006_shaft.stl           (was tool_wrist_sca_shaft_link.STL)
│       │   ├── PSM_tip_006_left.stl            (was tool_wrist_scal_link.STL — typo fixed)
│       │   ├── PSM_tip_jaw_1.stl               (was tool_wrist_sca_ee_link_1.STL)
│       │   └── PSM_tip_jaw_2.stl               (was tool_wrist_sca_ee_link_2.STL)
│       │
│       └── SF0826001/                          ← Si ECM camera tip
│           ├── ECM_shaft.stl                   (was tool_main_link.STL)
│           └── ECM_tip_roll.stl                (was tool_roll_link.STL)
│
├── launch/                                     (unchanged)
├── ros2/                                       (unchanged)
└── rviz/                                       (unchanged)
```

---

## Naming Rules

| Element | Rule | Example |
|---|---|---|
| ARM acronym | Always UPPERCASE | `PSM`, `MTM`, `ECM`, `SUJ` |
| Link name | `{ARM}_{description}_link` | `PSM_yaw_link` |
| Joint name | bare functional name (ROS compat) | `yaw`, `wrist_pitch`, `jaw` |
| Structural joints | descriptive + no suffix | `base_fixed`, `world_fixed` |
| Mesh file | `{ARM}_{description}.stl` | `PSM_yaw.stl` |
| Mesh extension | always lowercase | `.stl`, `.dae` |
| Xacro prefix arg | arm instance | `PSM1_`, `ECM_`, `MTML_` |
| URDF entry points | UPPERCASE arm name | `PSM1.urdf.xacro` |
| URDF macro files | lowercase | `PSM_base.urdf.xacro` |

---

## PSM/ECM Macro Architecture

The PSM and ECM arm is split across three layers, cleanly separating the arm
mechanism from the instrument:

```
Generation-specific:          Generation-agnostic (urdf/common/):
┌─────────────────────┐       ┌────────────────────────────────────────────────────┐
│  PSM_base           │──────▶│  PSM_instrument                                    │
│  (3 DOF)            │       │  params: shaft_length, tool_model                  │
│                     │       │                                                     │
│  Joint 1: yaw │       │  Joint 4: roll  (D = shaft_length)                 │
│  Joint 2: pitch│      │     └── PSM_roll_link (shaft mesh)                 │
│  Joint 3: insertion │       │           └── PSM_wrist_link                       │
│                     │       │   ┌──────────────────────────────────────────┐     │
│  Ends at:           │       │   │  PSM_tip_XXX macro (selected by model)   │     │
│  PSM_tool_parent_link│      │   │  Joint 5: wrist_pitch                    │     │
└─────────────────────┘       │   │  Joint 6: wrist_yaw                      │     │
                              │   │  Joint 7: jaw (+ jaw_1, jaw_2 mimics)    │     │
                              │   └──────────────────────────────────────────┘     │
                              └────────────────────────────────────────────────────┘
```

### Three PSM/ECM variants

| Variant | Folder | PSM_base behaviour |
|---|---|---|
| `Classic` | `urdf/Classic/` | Full parallelogram arm geometry; shaft D=0.4162 m |
| `Si` | `urdf/Si/` | Full arm geometry; shaft D=0.4670 m |
| `Virtual` | `urdf/Virtual/` | 3 massless DOFs at the fixed RCM frame; no arm mesh |

All three variants hand off to **the same** `PSM_instrument` macro at
`PSM_tool_parent_link`.

### Virtual variant — joint specification

The Virtual base places all three arm DOFs at the trocar (RCM) frame.
Joint limits and small inertia are provided for physics-simulator compatibility.

| Joint | Type | Axis | Lower | Upper | Effort | Velocity |
|---|---|---|---|---|---|---|
| `yaw` | revolute | z | −π/2 | π/2 | 50 N·m | 1 rad/s |
| `pitch` | revolute | y | −π/4 | π/4 | 50 N·m | 1 rad/s |
| `insertion` | prismatic | z | 0.0 m | 0.240 m | 50 N | 0.4 m/s |

All three virtual links: `mass=0.001 kg`, `ixx=iyy=izz=1e-6 kg·m²`.

Joint names are **identical** to the full model so existing ROS controllers and
state publishers work without modification.

---

## PSM Tip Family Taxonomy

| Tip macro | Shaft diam. | Wrist A | Jaw joints | Instruments (examples) |
|---|---|---|---|---|
| `PSM_tip_006` | 8 mm | 0.0091 m | jaw (virtual) + jaw_1 + jaw_2 | 420006, 400049\*, 420049, 400036, 420036 |
| `PSM_tip_172` | 8 mm | 0.0107 m | jaw + jaw_1 + jaw_2 | 400172, 420172 |
| `PSM_tip_049` | 8 mm | 0.0091 m | jaw + jaw_left + jaw_right | 400049 Classic Cadiere variant |
| `PSM_tip_117` | 5 mm | varies | extra coupled wrist joints | 400117 (NEEDLE_DRIVER 5mm snake) |
| `PSM_tip_183` | 8 mm | N/A | no jaw | 400183, 420183, 400179, 420179 |
| `PSM_tip_ECM` | — | N/A | no jaw | SF0826001 |

The `shaft_length` parameter (carried by `PSM_instrument`) determines the visual
length of `PSM_roll_link` (the shaft mesh).  The tip macros are shaft-length
agnostic.

---

## Bug Fixes

| File | Issue | Fix |
|---|---|---|
| `Classic/psm_base.urdf.xacro` | Includes `roll` + `roll_shaft` — instrument joints in arm file | Move to `PSM_instrument` |
| `Si/psm_base.urdf.xacro` | Missing `insertion` joint — deferred to tool file | Add `insertion` to PSM_base |
| `Si/ecm.urdf.xacro` line 4 | `tool_name="SF826001"` missing leading zero | Fix to `SF0826001` |
| `Classic/MTM/mtm_omponents.blend` | Typo in filename | Rename to `MTM_components.blend` |
| `instruments/420006/tool_wrist_scal_link.STL` | Likely typo (`scal` vs `sca`) | Rename to `PSM_tip_006_left.stl` |
