import numpy as np

# Robot Dimensions from URDF
L1 = 0.0275  # Coxa length (local X)
L2 = 0.1000  # Femur length (local X)
L3 = 0.1210  # Tibia length (local X)
Z_OFFSET = -0.0100  # Z offset from coxa to femur joint

# Leg Coxa Joint Origins in Body Frame
# Joint 0 (fl): xyz="0.0525 -0.047 0.020" rpy="0 0 0"
# Joint 4 (rl): xyz="0.0525 0.047 0.020" rpy="0 0 0"
# Joint 8 (fr): xyz="-0.0525 -0.047 0.020" rpy="0 0 3.14159"
# Joint 12 (rr): xyz="-0.0525 0.047 0.020" rpy="0 0 3.14159"
LEG_ORIGINS = {
    'fl': np.array([0.0525, -0.047, 0.020]),
    'rl': np.array([0.0525, 0.047, 0.020]),
    'fr': np.array([-0.0525, -0.047, 0.020]),
    'rr': np.array([-0.0525, 0.047, 0.020])
}

# Left legs have 0 rotation, Right legs have 180 (pi) rotation around Z
LEG_ROTATIONS = {
    'fl': 0.0,
    'rl': 0.0,
    'fr': np.pi,
    'rr': np.pi
}

def leg_fk(leg, coxa, femur, tibia):
    """
    Computes Forward Kinematics for a specific leg.
    Returns the foot position in the main body frame.
    """
    # 2D angles in the local leg plane
    phi2 = -femur
    phi3 = -tibia
    
    # Foot position in local leg plane (starting from femur joint)
    r = L2 * np.cos(phi2) + L3 * np.cos(phi2 + phi3)
    z_proj = L2 * np.sin(phi2) + L3 * np.sin(phi2 + phi3)
    
    # Foot position in local coxa frame
    x_local = L1 + r
    y_local = 0.0
    z_local = z_proj + Z_OFFSET
    
    # Apply coxa rotation (around local Z axis)
    x_rot = x_local * np.cos(coxa) - y_local * np.sin(coxa)
    y_rot = x_local * np.sin(coxa) + y_local * np.cos(coxa)
    z_rot = z_local
    
    # Apply leg base rotation to body frame
    rot = LEG_ROTATIONS[leg]
    x_body_local = x_rot * np.cos(rot) - y_rot * np.sin(rot)
    y_body_local = x_rot * np.sin(rot) + y_rot * np.cos(rot)
    z_body_local = z_rot
    
    # Translate to body frame
    pos_body = LEG_ORIGINS[leg] + np.array([x_body_local, y_body_local, z_body_local])
    return pos_body

def leg_ik(leg, foot_target_body):
    """
    Computes Inverse Kinematics for a specific leg given foot target in main body frame.
    Returns (coxa, femur, tibia) angles in radians.
    """
    # 1. Translate target to leg coxa joint origin
    rel_pos = foot_target_body - LEG_ORIGINS[leg]
    
    # 2. Rotate target to leg's default frame
    rot = LEG_ROTATIONS[leg]
    x_leg = rel_pos[0] * np.cos(-rot) - rel_pos[1] * np.sin(-rot)
    y_leg = rel_pos[0] * np.sin(-rot) + rel_pos[1] * np.cos(-rot)
    z_leg = rel_pos[2]
    
    # 3. Coxa Angle (theta1)
    coxa = np.arctan2(y_leg, x_leg)
    
    # 4. Project target onto the leg plane
    r = np.sqrt(x_leg**2 + y_leg**2) - L1
    z_proj = z_leg - Z_OFFSET
    
    # 5. 2-Link Planar IK (for femur and tibia)
    # D = (r^2 + z_proj^2 - L2^2 - L3^2) / (2 * L2 * L3)
    D = (r**2 + z_proj**2 - L2**2 - L3**2) / (2.0 * L2 * L3)
    
    # Keep D within [-1, 1] bounds to prevent NaN in arccos/sqrt
    D = np.clip(D, -1.0, 1.0)
    
    # Choose negative sign for elbow down (tibia pointing inward)
    phi3 = np.arctan2(-np.sqrt(1.0 - D**2), D)
    
    phi2 = np.arctan2(z_proj, r) - np.arctan2(L3 * np.sin(phi3), L2 + L3 * np.cos(phi3))
    
    # Convert local planar angles back to joint angles
    femur = -phi2
    tibia = -phi3
    
    # Clip angles to joint limits
    # Coxa: [-1.0, 1.0]
    # Femur: [-1.2, 1.2]
    # Tibia: [-3.0, 3.0] (fr uses [-3.14159, 3.14159])
    coxa = np.clip(coxa, -1.0, 1.0)
    femur = np.clip(femur, -1.2, 1.2)
    tibia_limit = 3.14159 if leg == 'fr' else 3.0
    tibia = np.clip(tibia, -tibia_limit, tibia_limit)
    
    return coxa, femur, tibia
