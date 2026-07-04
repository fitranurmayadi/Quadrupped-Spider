import os
import sys
import time
import numpy as np
import pybullet as p
import pybullet_data
from scipy.optimize import minimize

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



# MPC Parameters
N = 4 # Horizon length (reduced from 5)
dt = 0.6 # Prediction step (increased from 0.5)
v_max = 1.5 # Max linear speed (increased from 1.0)
omega_max = 2.0 # Max angular speed

def mpc_cost_function(u, current_pos, current_yaw, target_pos, obstacles):
    """
    Cost function for MPC.
    u is an array of [v0, w0, v1, w1, ..., vN-1, wN-1]
    """
    cost = 0.0
    x, y, yaw = current_pos[0], current_pos[1], current_yaw
    
    for i in range(N):
        v = u[2*i]
        w = u[2*i+1]
        
        # PyBullet kinematic update (yaw=0 is pointing along -Y, positive yaw turns CCW/left)
        x += v * np.sin(yaw) * dt
        y += -v * np.cos(yaw) * dt
        yaw += w * dt
        
        # Distance to goal cost
        dist_to_goal = np.sqrt((target_pos[0] - x)**2 + (target_pos[1] - y)**2)
        cost += dist_to_goal * 10.0
        
        # Obstacle avoidance cost (penalize getting closer than 0.6m)
        for obs in obstacles:
            obs_pos = obs[0]
            dist_to_obs = np.sqrt((obs_pos[0] - x)**2 + (obs_pos[1] - y)**2)
            if dist_to_obs < 0.6:
                cost += 150.0 / (dist_to_obs + 0.01)
                
    return cost

def solve_mpc(current_pos, current_yaw, target_pos, obstacles):
    # Initial guess: move straight to target, zero angular velocity
    u0 = np.zeros(2 * N)
    for i in range(N):
        u0[2*i] = 0.5 # small forward velocity
        
    bounds = [(-v_max, v_max) if i % 2 == 0 else (-omega_max, omega_max) for i in range(2 * N)]
    
    res = minimize(
        mpc_cost_function,
        u0,
        args=(current_pos, current_yaw, target_pos, obstacles),
        method='L-BFGS-B',
        bounds=bounds,
        options={'maxiter': 8} # reduced from 20 for much faster optimization
    )
    
    optimal_u = res.x
    
    # Calculate predicted states
    x, y, yaw = current_pos[0], current_pos[1], current_yaw
    trajectory = [(x, y)]
    for i in range(N):
        v = optimal_u[2*i]
        w = optimal_u[2*i+1]
        x += v * np.sin(yaw) * dt
        y += -v * np.cos(yaw) * dt
        yaw += w * dt
        trajectory.append((x, y))
        
    return optimal_u[0], optimal_u[1], trajectory

def main():
    import argparse
    parser = argparse.ArgumentParser(description="V7: Model Predictive Control (MPC)")
    parser.add_argument('--headless', action='store_true', help="Run without GUI")
    parser.add_argument('--show-debug-lines', action='store_true', help="Draw debug ray lines")
    args = parser.parse_args()

    mode = p.DIRECT if args.headless else p.GUI
    client = p.connect(mode)
    
    if mode == p.GUI:
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=client)
        p.resetDebugVisualizerCamera(cameraDistance=3.0, cameraYaw=-90, cameraPitch=-30, cameraTargetPosition=[0, 0, 0], physicsClientId=client)

    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client)
    p.setGravity(0, 0, -9.81, physicsClientId=client)
    time_step = 1.0 / 240.0
    p.setTimeStep(time_step, physicsClientId=client)
    plane_id = p.loadURDF("plane.urdf", physicsClientId=client)

    np.random.seed(42)
    num_obstacles = 24
    obstacles = []
    
    start_point = np.array([-5.0, -5.0])
    target_point = np.array([5.0, 5.0])
    
    # Spawn box obstacles randomly
    for i in range(num_obstacles):
        w = np.random.uniform(0.3, 0.5)
        l = np.random.uniform(0.3, 0.5)
        
        while True:
            x = np.random.uniform(-4.0, 4.0)
            y = np.random.uniform(-4.0, 4.0)
            pos = np.array([x, y])
            if np.linalg.norm(pos - start_point) > 2.0 and np.linalg.norm(pos - target_point) > 2.0:
                too_close = False
                for obs_data in obstacles:
                    if np.linalg.norm(pos - obs_data[0]) < 0.75:
                        too_close = True
                        break
                if not too_close:
                    obstacles.append((pos, w, l))
                    break
        h = np.random.uniform(0.18, 0.35)
        col_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[w/2, l/2, h/2], physicsClientId=client)
        vis_id = p.createVisualShape(p.GEOM_BOX, halfExtents=[w/2, l/2, h/2], rgbaColor=[0.8, 0.2, 0.2, 1], physicsClientId=client)
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col_id, baseVisualShapeIndex=vis_id, basePosition=[pos[0], pos[1], h/2], physicsClientId=client)

    
    # Raycast sensor parameters
    ray_length = 0.6
    # 16 rays distributed evenly 360 degrees around the robot
    ray_angles = np.linspace(-np.pi, np.pi, 16, endpoint=False)
    ray_line_ids = []

    # Summon visual Target Marker (Green beacon pole at 5,5)
    target_visual = p.createVisualShape(p.GEOM_CYLINDER, radius=0.15, length=0.8, rgbaColor=[0, 1, 0, 0.8], physicsClientId=client)
    p.createMultiBody(baseMass=0, baseVisualShapeIndex=target_visual, basePosition=[target_point[0], target_point[1], 0.4], physicsClientId=client)

    # Stateful variables for trajectory drawing
    traj_line_ids = []

    urdf_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "assets", "spider.urdf"))
    robot_id = p.loadURDF(urdf_path, [start_point[0], start_point[1], 0.15], p.getQuaternionFromEuler([0, 0, np.pi/4]), physicsClientId=client)
    gait = GaitControllerV2(step_length=0.07, step_height=0.025, frequency=5.0, body_z=-0.060)

    # Re-declare joint indices mapping
    joint_indices = {}
    for i in range(p.getNumJoints(robot_id, physicsClientId=client)):
        info = p.getJointInfo(robot_id, i, physicsClientId=client)
        joint_name = info[1].decode('utf-8')
        if info[2] == p.JOINT_REVOLUTE:
            joint_indices[joint_name] = i

    print("[INFO] Autopilot engaged. Starting navigation loop...")
    sim_time = 0.0
    last_mpc_time = -1.0
    
    opt_v = 0.0
    opt_w = 0.0
    
    while True:
        pos, ori = p.getBasePositionAndOrientation(robot_id, physicsClientId=client)
        yaw = p.getEulerFromQuaternion(ori)[2]
        
        dist_to_target = np.linalg.norm(np.array(pos[:2]) - target_point)
        if dist_to_target < 0.3:
            print("MISSION SUCCESS: Target coordinates reached!")
            break
            
        # Raycasting LiDAR Simulation (Runs at 240 Hz for responsive control loop)
        dist_left = ray_length
        dist_front = ray_length
        dist_right = ray_length
        
        # Clear old debug lines
        if args.show_debug_lines:
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
            
            hit_object, hit_fraction, hit_pos = cast_ray_recursive(client, r_start, r_end, robot_id, plane_id)
            
            if hit_object != -1 and hit_object != plane_id:
                hit_dist = hit_fraction * ray_length
                
                if -np.pi/4 <= angle <= np.pi/4:
                    dist_front = min(dist_front, hit_dist)
                elif np.pi/4 < angle <= 3*np.pi/4:
                    dist_left = min(dist_left, hit_dist)
                elif -3*np.pi/4 <= angle < -np.pi/4:
                    dist_right = min(dist_right, hit_dist)
                
                if args.show_debug_lines:
                    l_id = p.addUserDebugLine(r_start, [hit_pos[0], hit_pos[1], pos[2]], [1.0, 0.0, 0.0], 2, physicsClientId=client)
                    ray_line_ids.append(l_id)
            else:
                if args.show_debug_lines:
                    l_id = p.addUserDebugLine(r_start, r_end, [0.0, 1.0, 0.0], 1, physicsClientId=client)
                    ray_line_ids.append(l_id)

        # Run MPC at 3.3 Hz (every 0.3s instead of 0.2s)
        if sim_time - last_mpc_time >= 0.3:
            opt_v, opt_w, pred_trajectory = solve_mpc(np.array(pos[:2]), yaw, target_point, obstacles)
            last_mpc_time = sim_time
            print(f"sim_time={sim_time:.1f}s | dist={dist_to_target:.2f}m | opt_v={opt_v:.2f} | opt_w={opt_w:.2f}")
            
            # Clear old trajectory lines
            if mode == p.GUI:
                for l_id in traj_line_ids:
                    p.removeUserDebugItem(l_id, physicsClientId=client)
                traj_line_ids.clear()
                
                # Draw predicted trajectory in green
                for i in range(len(pred_trajectory) - 1):
                    l_id = p.addUserDebugLine(
                        [pred_trajectory[i][0], pred_trajectory[i][1], 0.05],
                        [pred_trajectory[i+1][0], pred_trajectory[i+1][1], 0.05],
                        [0, 1, 0], 3, physicsClientId=client
                    )
                    traj_line_ids.append(l_id)

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
            # ZONE 3: Clear Zone -> Use MPC outputs
            dx, dy = 0.0, -opt_v
            yaw_rate = opt_w

        
        joint_targets = gait.get_joint_targets(sim_time, direction=(dx, dy), yaw_rate=yaw_rate)
        
        for joint_name, target_angle in joint_targets.items():
            if joint_name in joint_indices:
                p.setJointMotorControl2(
                    robot_id,
                    joint_indices[joint_name],
                    p.POSITION_CONTROL,
                    targetPosition=target_angle,
                    force=1.2,
                    maxVelocity=3.0,
                    physicsClientId=client
                )
            
        p.stepSimulation(physicsClientId=client)
        if mode == p.GUI:
            time.sleep(time_step / 2.0) # Run simulation 2x faster in GUI
            p.resetDebugVisualizerCamera(cameraDistance=3.0, cameraYaw=-90, cameraPitch=-30, cameraTargetPosition=pos, physicsClientId=client)
            
        sim_time += time_step

    p.disconnect(physicsClientId=client)

if __name__ == "__main__":
    main()
