import json
import math
import re
import os
import numpy as np

def rot_x(a):
    return np.array([[1, 0, 0, 0],
                     [0, math.cos(a), -math.sin(a), 0],
                     [0, math.sin(a), math.cos(a), 0],
                     [0, 0, 0, 1]])

def rot_z(a):
    return np.array([[math.cos(a), -math.sin(a), 0, 0],
                     [math.sin(a), math.cos(a), 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])

def trans_x(a):
    return np.array([[1, 0, 0, a],
                     [0, 1, 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])

def trans_z(d):
    return np.array([[1, 0, 0, 0],
                     [0, 1, 0, 0],
                     [0, 0, 1, d],
                     [0, 0, 0, 1]])

def rotation_matrix_to_rpy(R):
    sy = math.sqrt(R[0, 0]**2 + R[1, 0]**2)
    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0
    return x, y, z

def modified_dh_to_urdf(alpha, a, theta, d, offset, joint_type):
    """
    Modified DH Frame i-1 to Frame i (with joint variable q):
    T = RotX(alpha) * TransX(a) * RotZ(theta + q) * TransZ(d)
    Wait, in dVRK JSON, 'offset' for revolute is added to q (theta + offset + q).
    For prismatic, 'offset' is added to d (D + offset + q).
    """
    if joint_type == "revolute":
        effective_theta = theta + offset
        effective_d = d
    elif joint_type == "prismatic":
        effective_theta = theta
        effective_d = d + offset
    else:
        effective_theta = theta
        effective_d = d

    # T = RotX(alpha) * TransX(a) * RotZ(theta) * TransZ(d)
    T = rot_x(alpha) @ trans_x(a) @ rot_z(effective_theta) @ trans_z(effective_d)
    
    xyz = T[:3, 3]
    rpy = rotation_matrix_to_rpy(T[:3, :3])
    return xyz, rpy

def clean_json(file_path):
    with open(file_path, 'r') as f:
        content = f.read()
    content = re.sub(r'//.*', '', content)
    content = re.sub(r'/\*.*?\*/', '', content, flags=re.DOTALL)
    content = re.sub(r',\s*([\]}])', r'\1', content)
    return json.loads(content)

def update_urdf_with_dh(urdf_path, base_json, tool_json):
    base_data = clean_json(base_json)
    tool_data = clean_json(tool_json)
    all_dh_joints = base_data['DH']['joints'] + tool_data['DH']['joints']
    
    with open(urdf_path, 'r') as f:
        urdf_content = f.read()
        
    for dh in all_dh_joints:
        joint_name = dh['name'] + "_joint"
        joint_type = dh.get('type', 'revolute')
        
        xyz_vec, rpy_vec = modified_dh_to_urdf(
            dh['alpha'], dh['A'], dh['theta'], dh['D'], dh['offset'], joint_type
        )
        
        xyz_str = f'xyz="{xyz_vec[0]:.6g} {xyz_vec[1]:.6g} {xyz_vec[2]:.6g}"'
        rpy_str = f'rpy="{rpy_vec[0]:.6g} {rpy_vec[1]:.6g} {rpy_vec[2]:.6g}"'
        
        pattern = rf'(<joint\s+name="{joint_name}"[^>]*>.*?<origin\s+)([^/]+)(/>)'
        def replace_origin(match):
            return match.group(1) + f'{xyz_str} {rpy_str} ' + match.group(3)
        urdf_content = re.sub(pattern, replace_origin, urdf_content, flags=re.DOTALL)
        
        if 'qmin' in dh and 'qmax' in dh:
            limit_pattern = rf'(<joint\s+name="{joint_name}"[^>]*>.*?<limit\s+)([^/]+)(/>)'
            new_limit = f'lower="{dh["qmin"]}" upper="{dh["qmax"]}" effort="{dh.get("ftmax", 1.0)}" velocity="4.0" '
            urdf_content = re.sub(limit_pattern, lambda m: m.group(1) + new_limit + m.group(3), urdf_content, flags=re.DOTALL)

    with open(urdf_path, 'w') as f:
        f.write(urdf_content)
    print(f"Updated {urdf_path} with corrected prismatic offsets.")

if __name__ == "__main__":
    workspace_root = "/home/anton/devel/newton_test"
    update_urdf_with_dh(
        os.path.join(workspace_root, "src/psm_si_description/urdf/psm_si_lnd.urdf"),
        os.path.join(workspace_root, "PSM_Si.json"),
        os.path.join(workspace_root, "LARGE_NEEDLE_DRIVER_420006.json")
    )