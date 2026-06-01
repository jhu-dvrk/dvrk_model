#!/usr/bin/env python3
"""
Step 1b: Complete the macro renames - fix PSM entry points and rename PSM_base files.
"""
import shutil
import os

# Rename PSM_base files to add _macros suffix
RENAMES = [
    ("urdf/Classic/PSM_base.urdf.xacro", "urdf/Classic/PSM_base_macros.urdf.xacro"),
    ("urdf/Si/PSM_base.urdf.xacro", "urdf/Si/PSM_base_macros.urdf.xacro"),
    ("urdf/Virtual/PSM_base_virtual.urdf.xacro", "urdf/Virtual/PSM_base_virtual_macros.urdf.xacro"),
]

# Update all callers
CALLERS = {
    # PSM entry points - update PSM_instrument reference
    "urdf/Classic/PSM1.urdf.xacro": [
        ('$(find dvrk_model)/urdf/common/PSM_instrument.urdf.xacro',
         '$(find dvrk_model)/urdf/common/PSM_instrument_macros.urdf.xacro'),
        ('$(find dvrk_model)/urdf/Classic/PSM_base.urdf.xacro',
         '$(find dvrk_model)/urdf/Classic/PSM_base_macros.urdf.xacro'),
    ],
    "urdf/Classic/PSM2.urdf.xacro": [
        ('$(find dvrk_model)/urdf/common/PSM_instrument.urdf.xacro',
         '$(find dvrk_model)/urdf/common/PSM_instrument_macros.urdf.xacro'),
        ('$(find dvrk_model)/urdf/Classic/PSM_base.urdf.xacro',
         '$(find dvrk_model)/urdf/Classic/PSM_base_macros.urdf.xacro'),
    ],
    "urdf/Classic/PSM3.urdf.xacro": [
        ('$(find dvrk_model)/urdf/common/PSM_instrument.urdf.xacro',
         '$(find dvrk_model)/urdf/common/PSM_instrument_macros.urdf.xacro'),
        ('$(find dvrk_model)/urdf/Classic/PSM_base.urdf.xacro',
         '$(find dvrk_model)/urdf/Classic/PSM_base_macros.urdf.xacro'),
    ],
    "urdf/Si/PSM1.urdf.xacro": [
        ('$(find dvrk_model)/urdf/common/PSM_instrument.urdf.xacro',
         '$(find dvrk_model)/urdf/common/PSM_instrument_macros.urdf.xacro'),
        ('$(find dvrk_model)/urdf/Si/PSM_base.urdf.xacro',
         '$(find dvrk_model)/urdf/Si/PSM_base_macros.urdf.xacro'),
    ],
    "urdf/Si/PSM2.urdf.xacro": [
        ('$(find dvrk_model)/urdf/common/PSM_instrument.urdf.xacro',
         '$(find dvrk_model)/urdf/common/PSM_instrument_macros.urdf.xacro'),
        ('$(find dvrk_model)/urdf/Si/PSM_base.urdf.xacro',
         '$(find dvrk_model)/urdf/Si/PSM_base_macros.urdf.xacro'),
    ],
    "urdf/Si/PSM3.urdf.xacro": [
        ('$(find dvrk_model)/urdf/common/PSM_instrument.urdf.xacro',
         '$(find dvrk_model)/urdf/common/PSM_instrument_macros.urdf.xacro'),
        ('$(find dvrk_model)/urdf/Si/PSM_base.urdf.xacro',
         '$(find dvrk_model)/urdf/Si/PSM_base_macros.urdf.xacro'),
    ],
    "urdf/Si/ECM.urdf.xacro": [
        ('$(find dvrk_model)/urdf/common/PSM_instrument.urdf.xacro',
         '$(find dvrk_model)/urdf/common/PSM_instrument_macros.urdf.xacro'),
        ('$(find dvrk_model)/urdf/Si/PSM_base.urdf.xacro',
         '$(find dvrk_model)/urdf/Si/PSM_base_macros.urdf.xacro'),
    ],
    "urdf/Virtual/PSM1.urdf.xacro": [
        ('$(find dvrk_model)/urdf/common/PSM_instrument.urdf.xacro',
         '$(find dvrk_model)/urdf/common/PSM_instrument_macros.urdf.xacro'),
        ('$(find dvrk_model)/urdf/Virtual/PSM_base_virtual.urdf.xacro',
         '$(find dvrk_model)/urdf/Virtual/PSM_base_virtual_macros.urdf.xacro'),
    ],
    "urdf/Virtual/PSM2.urdf.xacro": [
        ('$(find dvrk_model)/urdf/common/PSM_instrument.urdf.xacro',
         '$(find dvrk_model)/urdf/common/PSM_instrument_macros.urdf.xacro'),
        ('$(find dvrk_model)/urdf/Virtual/PSM_base_virtual.urdf.xacro',
         '$(find dvrk_model)/urdf/Virtual/PSM_base_virtual_macros.urdf.xacro'),
    ],
    "urdf/Virtual/PSM3.urdf.xacro": [
        ('$(find dvrk_model)/urdf/common/PSM_instrument.urdf.xacro',
         '$(find dvrk_model)/urdf/common/PSM_instrument_macros.urdf.xacro'),
        ('$(find dvrk_model)/urdf/Virtual/PSM_base_virtual.urdf.xacro',
         '$(find dvrk_model)/urdf/Virtual/PSM_base_virtual_macros.urdf.xacro'),
    ],
}

def move_file(src, dst):
    if not os.path.exists(src):
        print(f"⚠  Already renamed: {os.path.basename(src)}")
        return
    shutil.move(src, dst)
    print(f"✓ Renamed: {os.path.basename(src)} → {os.path.basename(dst)}")

def rewrite_file(filepath, replacements):
    if not os.path.exists(filepath):
        print(f"✗ Missing: {filepath}")
        return
    with open(filepath, 'r', encoding='utf-8') as f:
        content = f.read()
    original = content
    for old, new in replacements:
        content = content.replace(old, new)
    if content != original:
        with open(filepath, 'w', encoding='utf-8') as f:
            f.write(content)
        changes = sum(1 for o, _ in replacements if o in original)
        print(f"✓ Updated: {os.path.basename(filepath)} ({changes} includes)")
    else:
        print(f"ℹ  No changes: {os.path.basename(filepath)}")

print("="*80)
print("Step 1b: Complete PSM macro renames")
print("="*80)

print("\n[1/2] Renaming PSM_base* files...")
for src, dst in RENAMES:
    move_file(src, dst)

print("\n[2/2] Updating PSM entry points...")
for caller, replacements in CALLERS.items():
    rewrite_file(caller, replacements)

print("\n" + "="*80)
print("Step 1 Complete")
print("="*80)
print("\nRebuild and test:")
print("  cd ~/dvrk_ws && colcon build --packages-select dvrk_model --symlink-install")
print("  source install/setup.bash")
print("  cd ~/dvrk_ws/src/dvrk/dvrk_model")
print("  xacro urdf/Classic/PSM1.urdf.xacro > /dev/null && echo 'Classic PSM1 OK'")
print("  xacro urdf/Si/PSM1.urdf.xacro > /dev/null && echo 'Si PSM1 OK'")
print("  xacro urdf/Virtual/PSM1.urdf.xacro > /dev/null && echo 'Virtual PSM1 OK'")