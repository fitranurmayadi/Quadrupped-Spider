import os
import sys
import time
import numpy as np
import pybullet as p
import pybullet_data

# Ensure we can import src.gait_controller_v2 and spider_ik
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from src.gait_controller_v2 import GaitControllerV2

def cast_ray_recursive(client, start, end, robot_id, plane_id, max_depth=3):
    current_start = list(start)
    current_end = list(end)
    original_start = np.array(start)
    original_end = np.array(end)
    original_length = np.linalg.norm(original_end - original_start)
    direction = (original_end - original_start) / original_length
    
    remaining_length = original_length
    
    for depth in range(max_depth):
        res = p.rayTest(current_start, current_end, physicsClientId=client)[0]
        hit_object = res[0]
        hit_fraction = res[2]
        hit_pos = res[3]
        
        if hit_object == -1:
            return -1, 1.0, current_end
            
        if hit_object != robot_id:
            # Hit actual obstacle! Calculate fraction relative to original start/end
            total_dist = np.linalg.norm(np.array(hit_pos) - original_start)
            fraction = total_dist / original_length
            return hit_object, min(fraction, 1.0), hit_pos
            
        # Hit robot! Advance start point past the hit position
        dist_stepped = hit_fraction * remaining_length
        step_epsilon = 0.08  # step 8cm forward past the leg
        if dist_stepped + step_epsilon >= remaining_length:
            return -1, 1.0, end
            
        current_start = list(np.array(current_start) + (dist_stepped + step_epsilon) * direction)
        remaining_length = np.linalg.norm(original_end - np.array(current_start))
        
    return -1, 1.0, end

def main():
    print("==================================================")
    print("   V4: AUTONOMOUS NAVIGATION & OBSTACLE AVOIDANCE ")
    print("==================================================")
    print("Mission Profile:")
    print("  - Start Position : (-5.0, -5.0) m")
    print("  - Target Position: (5.0, 5.0) m")
    print("  - Environment    : 10x10m area with random block obstacles")
    print("  - Controller     : Raycast LiDAR + Potential Field Steering")
    print("  - Options        : --headless (run headless), --show-debug-lines (draw debug ray lines)")
    print("==================================================\n")

    headless = "--headless" in sys.argv
    show_debug_lines = "--show-debug-lines" in sys.argv

    # 1. Connect PyBullet GUI or DIRECT
    if headless:
        client = p.connect(p.DIRECT)
    else:
        client = p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
        
        # Position camera to view the entire arena
        p.resetDebugVisualizerCamera(
            cameraDistance=9.0,
            cameraYaw=45,
            cameraPitch=-55,
            cameraTargetPosition=[0, 0, 0],
            physicsClientId=client
        )

    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client)
    p.setGravity(0, 0, -9.81, physicsClientId=client)
    time_step = 1.0 / 240.0
    p.setTimeStep(time_step, physicsClientId=client)

    # 2. Load Ground plane
    plane_id = p.loadURDF("plane.urdf", physicsClientId=client)

    # Draw Arena boundary (10x10 meters: from -5 to 5 on X and Y)
    p.addUserDebugLine([-5, -5, 0.01], [5, -5, 0.01], [1, 1, 1], 3, physicsClientId=client)
    p.addUserDebugLine([5, -5, 0.01], [5, 5, 0.01], [1, 1, 1], 3, physicsClientId=client)
    p.addUserDebugLine([5, 5, 0.01], [-5, 5, 0.01], [1, 1, 1], 3, physicsClientId=client)
    p.addUserDebugLine([-5, 5, 0.01], [-5, -5, 0.01], [1, 1, 1], 3, physicsClientId=client)

    # 3. Procedurally generate random block obstacles
    np.random.seed(42)  # For reproducible obstacle course
    num_obstacles = 24
    obstacles = []
    
    start_point = np.array([-5.0, -5.0])
    target_point = np.array([5.0, 5.0])
    
    # Spawn box obstacles randomly
    for i in range(num_obstacles):
        while True:
            # Random position within the central region
            x = np.random.uniform(-4.0, 4.0)
            y = np.random.uniform(-4.0, 4.0)
            pos = np.array([x, y])
            
            # Ensure it is far enough from start and target
            if np.linalg.norm(pos - start_point) > 2.0 and np.linalg.norm(pos - target_point) > 2.0:
                # Ensure obstacles are spaced to leave ~100-110cm edge-to-edge gaps
                too_close = False
                for obs_pos in obstacles:
                    if np.linalg.norm(pos - obs_pos) < 0.75:
                        too_close = True
                        break
                if not too_close:
                    obstacles.append(pos)
                    break
                    
        # Dimensions (smaller blocks to ensure consistent 100-110cm gaps between them)
        w = np.random.uniform(0.3, 0.5)
        l = np.random.uniform(0.3, 0.5)
        h = np.random.uniform(0.18, 0.35)
        
        # Create block in PyBullet
        box_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[w/2, l/2, h/2], physicsClientId=client)
        box_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[w/2, l/2, h/2], rgbaColor=[0.7, 0.6, 0.5, 1.0], physicsClientId=client)
        p.createMultiBody(
            baseMass=0.0,  # Static
            baseCollisionShapeIndex=box_col,
            baseVisualShapeIndex=box_vis,
            basePosition=[x, y, h/2],
            physicsClientId=client
        )

    # 4. Summon visual Target Marker (Green beacon pole at 5,5)
    target_visual = p.createVisualShape(
        shapeType=p.GEOM_CYLINDER,
        radius=0.15,
        length=1.5,
        rgbaColor=[0.0, 0.8, 0.0, 0.8],
        physicsClientId=client
    )
    p.createMultiBody(
        baseMass=0.0,
        baseVisualShapeIndex=target_visual,
        basePosition=[target_point[0], target_point[1], 0.75],
        physicsClientId=client
    )
    # Draw nominal path (straight line from start to target)
    p.addUserDebugLine(
        [start_point[0], start_point[1], 0.05], 
        [target_point[0], target_point[1], 0.05], 
        [0, 1, 0], 3, physicsClientId=client
    )
    # 5. Summon Robot Spider at spawn position
    urdf_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "assets", "spider.urdf"))
    robot_start_pos = [-5.0, -5.0, 0.08]
    robot_start_orn = p.getQuaternionFromEuler([0, 0, 0])
    
    robot_id = p.loadURDF(
        urdf_path,
        basePosition=robot_start_pos,
        baseOrientation=robot_start_orn,
        flags=p.URDF_USE_INERTIA_FROM_FILE,
        physicsClientId=client
    )

    # Configure dynamics for robot
    num_joints = p.getNumJoints(robot_id, physicsClientId=client)
    joint_map = {}
    revolute_joints = []

    for i in range(num_joints):
        info = p.getJointInfo(robot_id, i, physicsClientId=client)
        j_name = info[1].decode('utf-8')
        link_name = info[12].decode('utf-8')
        joint_type = info[2]
        
        joint_map[j_name] = i
        if joint_type == p.JOINT_REVOLUTE:
            revolute_joints.append(i)
            
        # Use default dynamics values as loaded from URDF to prevent joint locks

    # Initialize V2 Trot Gait Controller (Stiff and fast)
    # Default height is 6.0 cm (body_z = -0.060)
    gait = GaitControllerV2(step_length=0.07, step_height=0.025, frequency=5.0, body_z=-0.060)

    # Reset pose helper
    def reset_pose():
        for j_name, j_idx in joint_map.items():
            if j_idx in revolute_joints:
                if 'femur' in j_name:
                    angle = -1.2
                elif 'tibia' in j_name:
                    angle = 2.7708
                else:
                    angle = 0.0
                p.resetJointState(robot_id, j_idx, targetValue=angle, targetVelocity=0, physicsClientId=client)

    reset_pose()

    # Teks Overlay Status
    status_text_id = None
    if not headless and show_debug_lines:
        p.addUserDebugText(
            text="V4 MISSION: Navigate to Green Beacon [5,5]",
            textPosition=[-3.0, -3.0, 1.8],
            textColorRGB=[0, 1, 0],
            textSize=1.5,
            physicsClientId=client
        )
        
        status_text_id = p.addUserDebugText(
            text="Dist to Target: 14.14 m | Heading: CALCULATING",
            textPosition=[-3.0, -3.0, 1.5],
            textColorRGB=[1, 1, 0],
            textSize=1.3,
            physicsClientId=client
        )

    # Raycast sensor parameters
    ray_length = 0.6
    # 16 rays distributed evenly 360 degrees around the robot
    ray_angles = np.linspace(-np.pi, np.pi, 16, endpoint=False)
    ray_line_ids = []

    # APF coefficients
    K_att = 1.0
    K_rep = 0.6

    # Stateful curl & low-pass steering variables
    curl_sign = None
    v_steer_prev = np.array([0.0, -1.0])
    dx, dy, yaw_rate = 0.0, 0.0, 0.0

    # Stuck detection & escape maneuver state
    last_check_pos = np.array([-5.0, -5.0])
    last_check_time = 5.0
    escape_timer = 0.0
    escape_mode = False
    escape_cooldown = 0.0

    sim_time = 0.0
    print("[INFO] Autopilot engaged. Starting navigation loop...")

    try:
        while p.isConnected(physicsClientId=client):
            # 1. Get current robot pose
            pos, orn = p.getBasePositionAndOrientation(robot_id, physicsClientId=client)
            euler = p.getEulerFromQuaternion(orn)
            yaw = euler[2]
            
            # Robot forward heading vector in world coordinates
            # Recall: dy = -1.0 moves along world -Y (when yaw = 0)
            # So forward heading is [-sin(yaw), -cos(yaw)]
            robot_heading_vec = np.array([np.sin(yaw), -np.cos(yaw)])
            robot_heading_angle = np.arctan2(robot_heading_vec[1], robot_heading_vec[0])

            # 2. Check distance to target
            dist_to_target = np.linalg.norm(np.array(pos[:2]) - target_point)
            
            # Update GUI text overlay and print console progress
            step_count = int(sim_time * 240)
            if step_count % 10 == 0:
                if not headless and show_debug_lines and status_text_id is not None:
                    p.addUserDebugText(
                        text=f"Dist to Target: {dist_to_target:.2f} m | Pos: [{pos[0]:.2f}, {pos[1]:.2f}]",
                        textPosition=[-3.0, -3.0, 1.5],
                        textColorRGB=[1, 1, 0],
                        textSize=1.3,
                        replaceItemUniqueId=status_text_id,
                        physicsClientId=client
                    )
            if step_count % 480 == 0:
                print(f"sim_time={sim_time:.1f}s | dist={dist_to_target:.2f}m | pos=[{pos[0]:.2f}, {pos[1]:.2f}]")

            if dist_to_target < 0.35:
                print("\n==================================================")
                print("   MISSION SUCCESS: Target coordinates reached!")
                print("==================================================")
                if not headless and show_debug_lines:
                    p.addUserDebugText(
                        text="MISSION ACCOMPLISHED! SUCCESS!",
                        textPosition=[pos[0]-1.0, pos[1]-1.0, 0.8],
                        textColorRGB=[0.0, 1.0, 0.0],
                        textSize=2.0,
                        physicsClientId=client
                    )
                time.sleep(3.0)
                break

            # 3. Raycasting LiDAR Simulation (Runs at 240 Hz for responsive control loop)
            v_rep = np.zeros(2)
            
            dist_left = ray_length
            dist_front = ray_length
            dist_right = ray_length
            
            if not headless and show_debug_lines:
                # Clear old debug lines
                for l_id in ray_line_ids:
                    p.removeUserDebugItem(l_id, physicsClientId=client)
                ray_line_ids.clear()
            
            for j, angle in enumerate(ray_angles):
                r_start = [pos[0], pos[1], pos[2] + 0.08]
                ray_yaw = yaw + angle
                r_end = [
                    pos[0] + ray_length * np.sin(ray_yaw),
                    pos[1] - ray_length * np.cos(ray_yaw),
                    pos[2] + 0.08
                ]
                
                # Call recursive raycaster
                hit_object, hit_fraction, hit_pos = cast_ray_recursive(client, r_start, r_end, robot_id, plane_id)
                
                if hit_object != -1 and hit_object != plane_id:
                    hit_dist = hit_fraction * ray_length
                    
                    if -np.pi/4 <= angle <= np.pi/4:
                        dist_front = min(dist_front, hit_dist)
                    elif np.pi/4 < angle <= 3*np.pi/4:
                        dist_left = min(dist_left, hit_dist)
                    elif -3*np.pi/4 <= angle < -np.pi/4:
                        dist_right = min(dist_right, hit_dist)
                    
                    if not headless and show_debug_lines:
                        # Highlight blocked path in Red
                        l_id = p.addUserDebugLine(r_start, [hit_pos[0], hit_pos[1], pos[2]], [1.0, 0.0, 0.0], 2, physicsClientId=client)
                        ray_line_ids.append(l_id)
                    
                    # Unit direction of the ray
                    u_ray = np.array([np.sin(ray_yaw), -np.cos(ray_yaw)])
                    # Repulsive force with 15 cm safety distance
                    rep_magnitude = (1.0 / max(hit_dist, 0.15) - 1.0 / ray_length) * (1.0 / max(hit_dist, 0.15)**2)
                    v_rep += -K_rep * rep_magnitude * u_ray
                else:
                    if not headless and show_debug_lines:
                        # Clear path in Green
                        l_id = p.addUserDebugLine(r_start, r_end, [0.0, 1.0, 0.0], 1, physicsClientId=client)
                        ray_line_ids.append(l_id)

            # 4. Calculate Attraction Vector field
            target_vec = target_point - np.array(pos[:2])
            v_att = target_vec / np.linalg.norm(target_vec) * K_att

            # 5. Add curl force with hysteresis to avoid local minima
            v_curl = np.zeros(2)
            if np.linalg.norm(v_rep) > 1e-5:
                v_curl = np.array([-v_rep[1], v_rep[0]])
                v_curl /= np.linalg.norm(v_curl)
                if curl_sign is None:
                    curl_sign = 1.0 if np.dot(v_curl, v_att) >= 0 else -1.0
                v_curl *= curl_sign
            else:
                if escape_cooldown <= 0.0:
                    curl_sign = None
            
            K_curl = 2.0
            v_steer_raw = v_att + v_rep + K_curl * v_curl
            
            # Apply low-pass filter (alpha = 0.08) to smooth steering transitions
            alpha = 0.08
            v_steer = alpha * v_steer_raw + (1.0 - alpha) * v_steer_prev
            v_steer_prev = v_steer
            
            desired_angle = np.arctan2(v_steer[1], v_steer[0])

            # Calculate heading error wrapped to [-pi, pi]
            heading_error = desired_angle - robot_heading_angle
            heading_error = (heading_error + np.pi) % (2.0 * np.pi) - np.pi

            # 6. Apply steering and gait controls with stuck detection
            # Check stuck condition every 3.0 seconds (moved less than 8cm)
            if sim_time - last_check_time >= 3.0:
                displacement = np.linalg.norm(np.array(pos[:2]) - last_check_pos)
                if displacement < 0.08:
                    escape_mode = True
                    escape_timer = 1.0
                    escape_cooldown = 6.0
                    curl_sign = -curl_sign if curl_sign is not None else 1.0
                    v_steer_prev = np.array([0.0, -1.0])  # Reset LPF
                    print(f"[ESCAPE] Stuck detected (moved only {displacement:.3f}m in 3s) at {pos[:2]}. Escape engaged!")
                last_check_pos = np.array(pos[:2])
                last_check_time = sim_time

            if escape_timer > 0.0:
                # ESCAPE MANEUVER: slight reverse + fast yaw for 1.0 seconds
                dy = 0.1   # move backward slightly
                yaw_rate = 3.5 * curl_sign if curl_sign is not None else 3.5
                dx = 0.0
                escape_timer -= time_step
                if escape_timer <= 0.0:
                    escape_mode = False
                    last_check_pos = np.array(pos[:2])
                    last_check_time = sim_time
            else:
                dx, dy = 0.0, 0.0
                yaw_rate = 0.0
                
                # Preemptive Obstacle Avoidance based on specific LiDAR sectors
                if dist_front < 0.60 or dist_left < 0.60 or dist_right < 0.60:
                    if dist_front < 0.30:
                        # ZONE 1: Critical Zone (< 30cm in front) -> Pivot turn quickly in place
                        dy = 0.0
                        yaw_rate = 3.5 if dist_right < dist_left else -3.5
                    else:
                        # ZONE 2: Warning Zone (30cm to 60cm front, or < 60cm sides) -> Walk forward while turning proportionally
                        active_dist = min(dist_front, dist_left, dist_right)
                        proximity_factor = np.clip((0.60 - active_dist) / 0.30, 0.0, 1.0)
                        dy = -0.5 * (1.0 - 0.3 * proximity_factor)  # slow down slightly as we get closer
                        yaw_rate = 3.0 * proximity_factor * (1.0 if dist_right < dist_left else -1.0)
                else:
                    # ZONE 3: Clear Zone (>= 60cm) -> Standard Steering
                    if np.abs(heading_error) > 1.2:
                        # Pivot turn in place only if heading error is very large (> ~70 deg)
                        yaw_rate = np.clip(3.5 * heading_error, -2.5, 2.5)
                        dx, dy = 0.0, 0.0
                    else:
                        # Walk forward while adjusting yaw (faster yaw correction)
                        yaw_rate = np.clip(3.0 * heading_error, -2.0, 2.0)
                        # Maintain forward speed while turning
                        forward_speed = 1.0 * max(0.3, np.cos(heading_error))
                        dy = -forward_speed

            # Compute joint targets
            joint_targets = gait.get_joint_targets(sim_time, direction=(dx, dy), yaw_rate=yaw_rate)
            
            for j_name, target_angle in joint_targets.items():
                if j_name in joint_map:
                    p.setJointMotorControl2(
                        bodyUniqueId=robot_id,
                        jointIndex=joint_map[j_name],
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=target_angle,
                        force=12.0,
                        maxVelocity=5.0,
                        physicsClientId=client
                    )

            # 7. Step simulation and adjust camera tracker
            p.stepSimulation(physicsClientId=client)
            sim_time += time_step
            if escape_cooldown > 0.0:
                escape_cooldown -= time_step
            
            # Headless safety break
            if headless and sim_time > 250.0:
                print("[WARNING] Headless simulation reached timeout of 250s.")
                break
                
            if not headless:
                # Smooth third-person chase camera
                p.resetDebugVisualizerCamera(
                    cameraDistance=2.2,
                    cameraYaw=-90.0,
                    cameraPitch=-30.0,
                    cameraTargetPosition=[pos[0], pos[1], pos[2] + 0.1],
                    physicsClientId=client
                )
                if not headless:
                    time.sleep(time_step / 2.0) # Run simulation 2x faster in GUI

    except KeyboardInterrupt:
        print("[INFO] Autopilot interrupted.")
    finally:
        if p.isConnected(physicsClientId=client):
            p.disconnect(physicsClientId=client)
            print("[INFO] PyBullet disconnected.")

if __name__ == "__main__":
    main()
