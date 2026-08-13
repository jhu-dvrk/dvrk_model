# Mesh Rename Map

This file groups mesh renames discovered in `yvonne-testing` for review. Each entry shows the old location and the new location after the refactor.

## Overview

| Section | Count |
| --- | ---: |
| Classic SUJ | 20 |
| Classic ECM | 10 |
| Classic MTM | 24 |
| Classic PSM | 24 |
| Si SUJ | 13 |
| Si USM | 5 |
| Si Other | 1 |
| Instruments / Si | 10 |
| **Total** | **107** |

<!-- Remove all arm name from the beginning of the mesh files? Same with launch files? -->

## Classic SUJ

| Old mesh path | New mesh path |
| --- | --- |
| `meshes/Classic/SUJ/base_link.stl` | `meshes/SUJ/Classic/SUJ_cart.stl` |
| `meshes/Classic/SUJ/suj_ecm_L0.stl` | `meshes/SUJ/Classic/SUJ_Classic_L0.stl` |
| `meshes/Classic/SUJ/suj_ecm_L1.stl` | `meshes/SUJ/Classic/SUJ_Classic_L1.stl` |
| `meshes/Classic/SUJ/suj_ecm_L2.stl` | `meshes/SUJ/Classic/SUJ_Classic_L2.stl` |
| `meshes/Classic/SUJ/suj_ecm_L3.stl` | `meshes/SUJ/Classic/SUJ_ECM_L3.stl` |
| `meshes/Classic/SUJ/suj_psm1_L0.stl` | `meshes/SUJ/Classic/SUJ_Classic_L0.stl` |
| `meshes/Classic/SUJ/suj_psm1_L1.stl` | `meshes/SUJ/Classic/SUJ_Classic_L1.stl` |
| `meshes/Classic/SUJ/suj_psm1_L2.stl` | `meshes/SUJ/Classic/SUJ_Classic_L2.stl` |
| `meshes/Classic/SUJ/suj_psm1_L3.stl` | `meshes/SUJ/Classic/SUJ_Classic_PSM12_L3.stl` |
| `meshes/Classic/SUJ/suj_psm1_L4.stl` | `meshes/SUJ/Classic/SUJ_Classic_PSM_L4.stl` |
| `meshes/Classic/SUJ/suj_psm2_L0.stl` | `meshes/SUJ/Classic/SUJ_Classic_L0.stl` |
| `meshes/Classic/SUJ/suj_psm2_L1.stl` | `meshes/SUJ/Classic/SUJ_Classic_L1.stl` |
| `meshes/Classic/SUJ/suj_psm2_L2.stl` | `meshes/SUJ/Classic/SUJ_Classic_L2.stl` |
| `meshes/Classic/SUJ/suj_psm2_L3.stl` | `meshes/SUJ/Classic/SUJ_Classic_PSM12_L3.stl` |
| `meshes/Classic/SUJ/suj_psm2_L4.stl` | `meshes/SUJ/Classic/SUJ_Classic_PSM_L4.stl` |
| `meshes/Classic/SUJ/suj_psm3_L0.stl` | `meshes/SUJ/Classic/SUJ_PSM3_L0.stl` |
| `meshes/Classic/SUJ/suj_psm3_L1.stl` | `meshes/SUJ/Classic/SUJ_PSM3_L1.stl` |
| `meshes/Classic/SUJ/suj_psm3_L2.stl` | `meshes/SUJ/Classic/SUJ_Classic_L2.stl` |
| `meshes/Classic/SUJ/suj_psm3_L3.stl` | `meshes/SUJ/Classic/SUJ_PSM3_L3.stl` |
| `meshes/Classic/SUJ/suj_psm3_L4.stl` | `meshes/SUJ/Classic/SUJ_Classic_PSM_L4.stl` |

## Classic ECM

| Old mesh path | New mesh path |
| --- | --- |
| `meshes/Classic/ECM/ecm_base_link.stl` | `meshes/arms/Classic/ECM/ECM_base.stl` |
| `meshes/Classic/ECM/ecm_main_insertion_link.stl` | `meshes/arms/Classic/ECM/ECM_main_insertion.stl` |
| `meshes/Classic/ECM/ecm_pitch_back_link.stl` | `meshes/arms/Classic/ECM/ECM_pitch_back.stl` | <!-- Did not rename as pitch 2-5 like PSM -->
| `meshes/Classic/ECM/ecm_pitch_bottom_link.stl` | `meshes/arms/Classic/ECM/ECM_pitch_bottom.stl` |
| `meshes/Classic/ECM/ecm_pitch_end_link.stl` | `meshes/arms/Classic/ECM/ECM_pitch_end.stl` |
| `meshes/Classic/ECM/ecm_pitch_front_link.stl` | `meshes/arms/Classic/ECM/ECM_pitch_front.stl` | 
| `meshes/Classic/ECM/ecm_pitch_top_link.stl` | `meshes/arms/Classic/ECM/ECM_pitch_top.stl` |
| `meshes/Classic/ECM/ecm_remote_center_link.stl` | `meshes/arms/Classic/ECM/ECM_RCM.stl` | <!-- Rename back to remote center link? -->
| `meshes/Classic/ECM/ecm_tool_link.stl` | `meshes/arms/Classic/ECM/ECM_tool.stl` |
| `meshes/Classic/ECM/ecm_yaw_link.stl` | `meshes/arms/Classic/ECM/ECM_yaw.stl` |

## Classic MTM

| Old mesh path | New mesh path |
| --- | --- |
| `meshes/Classic/MTM/ArmParallel.dae` | `meshes/arms/Classic/MTM/MTM_shoulder_pitch.dae` | 
| `meshes/Classic/MTM/ArmParallel.stl` | `meshes/arms/Classic/MTM/MTM_shoulder_pitch.stl` |
| `meshes/Classic/MTM/ArmParallel1.dae` | `meshes/arms/Classic/MTM/MTM_shoulder_pitch_parallel.dae` |
| `meshes/Classic/MTM/ArmParallel1.stl` | `meshes/arms/Classic/MTM/MTM_shoulder_pitch_parallel.stl` |
| `meshes/Classic/MTM/BottomArm.dae` | `meshes/arms/Classic/MTM/MTM_elbow_pitch.dae` |
| `meshes/Classic/MTM/BottomArm.stl` | `meshes/arms/Classic/MTM/MTM_elbow_pitch.stl` |
| `meshes/Classic/MTM/Link.dae` | `meshes/arms/Classic/MTM/MTM_arm.dae` | <!-- Rename as just arm? -->
| `meshes/Classic/MTM/Link.stl` | `meshes/arms/Classic/MTM/MTM_arm.stl` |
| `meshes/Classic/MTM/OutPitch_Shoulder.dae` | `meshes/arms/Classic/MTM/MTM_outer_yaw.dae` |
| `meshes/Classic/MTM/OutPitch_Shoulder.stl` | `meshes/arms/Classic/MTM/MTM_outer_yaw.stl` |
| `meshes/Classic/MTM/Top_Panel.dae` | `meshes/arms/Classic/MTM/MTM_top_panel.dae` |
| `meshes/Classic/MTM/Top_Panel.stl` | `meshes/arms/Classic/MTM/MTM_top_panel.stl` |
| `meshes/Classic/MTM/WristPitch.dae` | `meshes/arms/Classic/MTM/MTM_wrist_pitch.dae` |
| `meshes/Classic/MTM/WristPitch.stl` | `meshes/arms/Classic/MTM/MTM_wrist_pitch.stl` |
| `meshes/Classic/MTM/WristPlatform.dae` | `meshes/arms/Classic/MTM/MTM_wrist_platform.dae` |
| `meshes/Classic/MTM/WristPlatform.stl` | `meshes/arms/Classic/MTM/MTM_wrist_platform.stl` |
| `meshes/Classic/MTM/WristRoll.dae` | `meshes/arms/Classic/MTM/MTM_wrist_roll.dae` |
| `meshes/Classic/MTM/WristRoll.stl` | `meshes/arms/Classic/MTM/MTM_wrist_roll.stl` |
| `meshes/Classic/MTM/WristYaw.dae` | `meshes/arms/Classic/MTM/MTM_wrist_yaw.dae` |
| `meshes/Classic/MTM/WristYaw.stl` | `meshes/arms/Classic/MTM/MTM_wrist_yaw.stl` |
| `meshes/Classic/MTM/mtm_omponents.blend` | `meshes/arms/Classic/MTM/MTM_components.blend` |
| `meshes/Classic/MTM/mtm_wrist_pitch.png` | `meshes/arms/Classic/MTM/mtm_wrist_pitch.png` | 
| `meshes/Classic/MTM/mtm_wrist_platform.png` | `meshes/arms/Classic/MTM/mtm_wrist_platform.png` |
| `meshes/Classic/MTM/mtm_wrist_yaw.png` | `meshes/arms/Classic/MTM/mtm_wrist_yaw.png` |

## Classic PSM

| Old mesh path | New mesh path |
| --- | --- |
| `meshes/Classic/PSM/outer_insertion.dae` | `meshes/arms/Classic/PSM/PSM_insertion.dae` |
<!--Order of occurrence in code: Back front bottom top-->
| `meshes/Classic/PSM/outer_pitch_back.dae` | `meshes/arms/Classic/PSM/PSM_pitch_2.dae` |
| `meshes/Classic/PSM/outer_pitch_bottom.dae` | `meshes/arms/Classic/PSM/PSM_pitch_4.dae` |
| `meshes/Classic/PSM/outer_pitch_front.dae` | `meshes/arms/Classic/PSM/PSM_pitch_3.dae` |
| `meshes/Classic/PSM/outer_pitch_top.dae` | `meshes/arms/Classic/PSM/PSM_pitch_5.dae` |
| `meshes/Classic/PSM/outer_yaw.dae` | `meshes/arms/Classic/PSM/PSM_yaw.dae` |
| `meshes/Classic/PSM/psm_base.dae` | `meshes/arms/Classic/PSM/PSM_base.dae` |
| `meshes/Classic/PSM/snake_tool/gripper_2.STL` | `meshes/arms/Classic/PSM/snake_tool/gripper_2.STL` |
| `meshes/Classic/PSM/snake_tool/gripper_3.STL` | `meshes/arms/Classic/PSM/snake_tool/gripper_3.STL` |
| `meshes/Classic/PSM/snake_tool/link_0.STL` | `meshes/arms/Classic/PSM/snake_tool/link_0.STL` |
| `meshes/Classic/PSM/snake_tool/link_1.STL` | `meshes/arms/Classic/PSM/snake_tool/link_1.STL` |
| `meshes/Classic/PSM/snake_tool/link_2.STL` | `meshes/arms/Classic/PSM/snake_tool/link_2.STL` |
| `meshes/Classic/PSM/snake_tool/link_3.STL` | `meshes/arms/Classic/PSM/snake_tool/link_3.STL` |
| `meshes/Classic/PSM/snake_tool/link_4.STL` | `meshes/arms/Classic/PSM/snake_tool/link_4.STL` |
| `meshes/Classic/PSM/tool_adapter.dae` | `meshes/arms/Classic/PSM/tool_adapter.dae` |
| `meshes/Classic/PSM/tool_main.dae` | `meshes/instruments/housing/Classic/housing_Classic.dae` |

Review:
| `meshes/Classic/PSM/tool_wrist_caudier_link_1.stl` | `meshes/instruments/wrist_pitch/0091/wrist_pitch_0091.stl` |
| `meshes/Classic/PSM/tool_wrist_caudier_link_1_shaft.stl` | `meshes/instruments/wrist_yaw/0091/wrist_yaw_0091.stl` |
| `meshes/Classic/PSM/tool_wrist_caudier_link_2.stl` | `meshes/instruments/tip/049/tip_049.stl` |
| `meshes/Classic/PSM/tool_wrist_link.dae` | `meshes/instruments/roll/4162/roll_4162.dae` |
| `meshes/Classic/PSM/tool_wrist_sca_link.dae` | `meshes/instruments/wrist_pitch/0091/wrist_pitch_0091.dae` |
| `meshes/Classic/PSM/tool_wrist_sca_link_2.dae` | `meshes/instruments/tip/006/Classic/tip_006.dae` |
| `meshes/Classic/PSM/tool_wrist_sca_shaft_link.dae` | `meshes/instruments/wrist_yaw/0091/wrist_yaw_0091.dae` |
| `meshes/Classic/PSM/tool_wrist_shaft_link.dae` | `meshes/instruments/roll/4162/roll_4162_yaw.dae` |

## Si SUJ

| Old mesh path | New mesh path |
| --- | --- |
| `meshes/Si/SUJ/ECM/link_0.STL` | `meshes/SUJ/Si/ECM/SUJ_ECM_L0.stl` |
| `meshes/Si/SUJ/ECM/link_1.STL` | `meshes/SUJ/Si/ECM/SUJ_ECM_L1.stl` |
| `meshes/Si/SUJ/ECM/link_2.STL` | `meshes/SUJ/Si/ECM/SUJ_ECM_L2.stl` |
| `meshes/Si/SUJ/ECM/link_3.STL` | `meshes/SUJ/Si/ECM/SUJ_ECM_L3.stl` |
| `meshes/Si/SUJ/PSM/12/link_0.STL` | `meshes/SUJ/Si/PSM/PSM12/SUJ_PSM12_L0.stl` |
| `meshes/Si/SUJ/PSM/12/link_1.STL` | `meshes/SUJ/Si/PSM/PSM12/SUJ_PSM12_L1.stl` |
| `meshes/Si/SUJ/PSM/12/link_2.STL` | `meshes/SUJ/Si/PSM/PSM12/SUJ_PSM12_L2.stl` |
| `meshes/Si/SUJ/PSM/12/link_3.STL` | `meshes/SUJ/Si/PSM/PSM12/SUJ_PSM12_L3.stl` |
| `meshes/Si/SUJ/PSM/3/link_0.STL` | `meshes/SUJ/Si/PSM/PSM3/SUJ_PSM3_L0.stl` |
| `meshes/Si/SUJ/PSM/3/link_1.STL` | `meshes/SUJ/Si/PSM/PSM3/SUJ_PSM3_L1.stl` |
| `meshes/Si/SUJ/PSM/3/link_2.STL` | `meshes/SUJ/Si/PSM/PSM3/SUJ_PSM3_L2.stl` |
| `meshes/Si/SUJ/PSM/3/link_3.STL` | `meshes/SUJ/Si/PSM/PSM3/SUJ_PSM3_L3.stl` |
| `meshes/Si/SUJ/PSM/3/link_4.STL` | `meshes/SUJ/Si/PSM/PSM3/SUJ_PSM3_L4.stl` |

## Si USM

| Old mesh path | New mesh path |
| --- | --- |
| `meshes/Si/PSM_ECM/link_0.STL` | `meshes/arms/Si/USM/USM_base.stl` |
| `meshes/Si/PSM_ECM/link_1.STL` | `meshes/arms/Si/USM/USM_yaw.stl` |
| `meshes/Si/PSM_ECM/link_2.STL` | `meshes/arms/Si/USM/USM_pitch.stl` |
| `meshes/Si/PSM_ECM/link_3.STL` | `meshes/arms/Si/USM/USM_pitch_3.stl` | <!-- originally bottom -->
| `meshes/Si/PSM_ECM/link_4.STL` | `meshes/arms/Si/USM/USM_pitch_4.stl` | <!-- originally top -->

## Si Other

| Old mesh path | New mesh path |
| --- | --- |
| `meshes/Si/tower.STL` | `meshes/tower.stl` |

## Instruments / Si

| Old mesh path | New mesh path |
| --- | --- |
| `meshes/instruments/420006/tool_main_link.STL` | `meshes/instruments/housing/Si/housing_Si_archived.stl` |
| `meshes/instruments/420006/tool_wrist_link.STL` | `meshes/instruments/roll/4670/roll_4670_archived.stl` |
| `meshes/instruments/420006/tool_wrist_sca_ee_link_1.STL` | `meshes/instruments/tip/006/tip_006_1_archived.stl` |
| `meshes/instruments/420006/tool_wrist_sca_ee_link_2.STL` | `meshes/instruments/tip/006/tip_006_2_archived.stl` |
| `meshes/instruments/420006/tool_wrist_sca_link.STL` | `meshes/instruments/wrist_pitch/0091/wrist_pitch_0091_archived.stl` |
| `meshes/instruments/420006/tool_wrist_sca_shaft_link.STL` | `meshes/instruments/wrist_yaw/0091/wrist_yaw_0091_archived.stl` |
| `meshes/instruments/420006/tool_wrist_scal_link.STL` | `meshes/_review/orphans/PSM_tip_006_left.stl` |
| `meshes/instruments/420006/tool_wrist_shaft_link.STL` | `meshes/instruments/roll/4670/roll_4670_connector.stl` |
| `meshes/instruments/SF0826001/tool_main_link.STL` | `meshes/instruments/housing/Si/housing_Si_ecm.stl` |
| `meshes/instruments/SF0826001/tool_roll_link.STL` | `meshes/instruments/roll/4670/roll_4670_ecm.stl` |


## Switch to devel
cd ~/dvrk_ws
# Disable yvonne-testing and main
touch src/dvrk/dvrk_model/COLCON_IGNORE
touch src/dvrk/dvrk_model_main/COLCON_IGNORE
# Enable devel
rm -f src/dvrk/dvrk_model_devel/COLCON_IGNORE
# Clean and rebuild
rm -rf build install log
colcon build
source install/setup.bash
echo "Active: devel"

## Switch to yvonne-testing
cd ~/dvrk_ws
# Enable yvonne-testing
rm -f src/dvrk/dvrk_model/COLCON_IGNORE
# Disable devel and main
touch src/dvrk/dvrk_model_devel/COLCON_IGNORE
touch src/dvrk/dvrk_model_main/COLCON_IGNORE
# Clean and rebuild
rm -rf build install log
colcon build
source install/setup.bash
echo "Active: yvonne-testing"

## Switch to main
cd ~/dvrk_ws
# Disable yvonne-testing and devel
touch src/dvrk/dvrk_model/COLCON_IGNORE
touch src/dvrk/dvrk_model_devel/COLCON_IGNORE
# Enable main
rm -f src/dvrk/dvrk_model_main/COLCON_IGNORE
# Clean and rebuild
rm -rf build install log
colcon build
source install/setup.bash
echo "Active: main"
